/* ********************************************************************************
 * author: chenyingbing
 * time: 20170627 19:46  in XIAMEN University
 * illustration:
 *      read the graph files. and extract the data.
 *      then graph optimizatoin and build map.
 *
 *
 * *******************************************************************************/

#include "buildmap.h"

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

FILE_HEAD file_head;
std::vector<FILE_NODE>  file_nodes;         // NODES PACK
std::vector<FILE_EDGE>  file_edges;         // EDGES PACK
std::vector<float>      file_grids;         // GRIDS PACK
FILE_TAIL file_tail;

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

void graph_globaldata_clear(void)
{
    file_nodes.clear();
    file_edges.clear();
    file_grids.clear();
}

bool graph_write_file(std::string catkin_route)
{
    std::string text_no = "no";
    std::string prefix_text = "/", cin_text, cin_filename;

    cout << "[ INFO ] IF SAVE THE MODIFIED GRAPH, INPUT THE FILE. OR INPUT 'no': ";
    cin >> cin_text;

    cin_filename = catkin_route + prefix_text + cin_text;

    if(cin_text == text_no)
    {

    }
    else{
        char *pack;
        int size_file_interval = sizeof(FILE_INTERVAL);
        FILE_INTERVAL file_interval_normal;

        //> alloc the data.
        pack = (char *) std::malloc(file_head.dtasize_total);

        memcpy(pack + 0, &file_head, sizeof(FILE_HEAD));                                                    // HEAD
        memcpy(pack + 0 + sizeof(FILE_HEAD), &file_interval_normal, size_file_interval);

        memcpy(pack + file_head.locate2nodes, file_nodes.data(), file_head.dtasize_nodes);                  // NODES
        memcpy(pack + file_head.locate2nodes + file_head.dtasize_nodes, &file_interval_normal, size_file_interval);

        memcpy(pack + file_head.locate2edges, file_edges.data(), file_head.dtasize_edges);                  // EDGES
        memcpy(pack + file_head.locate2edges + file_head.dtasize_edges, &file_interval_normal, size_file_interval);

        memcpy(pack + file_head.locate2grids, file_grids.data(), file_head.dtasize_grids);                  // GRIDS
        memcpy(pack + file_head.locate2grids + file_head.dtasize_grids, &file_interval_normal, size_file_interval);

        memcpy(pack + file_head.locate2tail, &file_tail, sizeof(FILE_TAIL));                                // TAIL

        bool sta = true;
        {
            int write_number;
            int file_fd = open(cin_filename.data(),
                              O_CREAT |     // if file did not exist, create it.
                              O_WRONLY |    // write only.
                              O_TRUNC,      // if file exist, empty it.
                              00700);

            if(file_fd == -1)
            {
                sta = false;
                ROS_ERROR(" FILE OPEN FAIL.");
            }

            write_number = write(file_fd, pack, file_head.dtasize_total);

            if(write_number != file_head.dtasize_total)
            {
                sta = false;
                ROS_ERROR(" FILE WRITE FAIL: WRITE NUMBER != FILE_TOTAL_SIZE");
            }
            close(file_fd);
        }

        if(sta)
        {
            ROS_INFO("WRITE FILE SUCCESS. ROUTE: %s.", cin_filename.data());
        }

        free(pack);
    }
}

bool graph_read_file(std::string catkin_route)
{
    static FILE_HEAD file_head_normal;

    /// *************************************************************************
    /// PREPARATION.
    std::string read_file_name = "/slam_graph.data";
    catkin_route = catkin_route + read_file_name;

    /// *************************************************************************
    /// FIRST READ FOR CHECK.
    /* **************************************************************************
    /// FILE_HEAD                       //          [Unit Type: FILE_HEAD]

    // > FILE_INTERVAL

    unsigned int  num_nodes;            // 4 bytes  [Unit Type: FILE_NODE]          yes
    unsigned int  locate2nodes;         // 4 bytes                                  yes

    // > FILE_INTERVAL

    unsigned int  num_edges;            // 4 bytes  [Unit Type: FILE_EDGE]          yes
    unsigned int  locate2edges;         // 4 bytes                                  yes

    // > FILE_INTERVAL

    unsigned int num_grids;             // 4 bytes  [Unit Type: float]              yes
    unsigned int locate2grids;          // 4 bytes                                  yes

    // > FILE_INTERVAL

    /// FILE_TAIL
    unsigned int  locate2tail;          // 4 bytes  [Unit Type: FILE_TAIL]
    ************************************************************************** */

    int file_fd = open(catkin_route.data(),
                       O_RDONLY      // read only.
                       );

    if(file_fd == -1)
    {
       ROS_ERROR(" FILE OPEN FAIL.");
       return false;
    }else
       ROS_INFO(" FILE OPEN SUCCESS.");

    int read_data_size;
    int size_file_interval = sizeof(FILE_INTERVAL);

    FILE_INTERVAL file_interval_read;
    bool check_sta = true;

    read_data_size = read(file_fd, &file_head, sizeof(FILE_HEAD));                      // HEAD

    check_sta &= (read_data_size == sizeof(FILE_HEAD));
    check_sta &= (file_head.check_char == file_head_normal.check_char);

    check_sta &= ((sizeof(FILE_NODE) * file_head.num_nodes) == file_head.dtasize_nodes);
    check_sta &= ((sizeof(FILE_EDGE) * file_head.num_edges) == file_head.dtasize_edges);
    check_sta &= ((sizeof(float) * file_head.num_grids) == file_head.dtasize_grids);

    if(!check_sta)
    {
        ROS_ERROR(" FILE HEAD CHECK FAIL.");
        return false;
    }else
        ROS_INFO(" FILE HEAD CHECK SUCCESS.");

    close(file_fd);

    cout << endl;

    /// *************************************************************************
    /// FULL READ FOR GETDATAS.

    file_fd = open(catkin_route.data(),
                   O_RDONLY      // read only.
                   );

    if(file_fd == -1)
    {
       ROS_ERROR(" FILE REOPEN FAIL.");
       return false;
    }else
       ROS_INFO(" FILE REOPEN SUCCESS.");

    char *read_pack;
    read_pack = (char *)malloc(file_head.dtasize_total);
    read_data_size = read(file_fd, read_pack, file_head.dtasize_total);

    if(read_data_size != file_head.dtasize_total)
    {
        ROS_ERROR(" FILE SIZE RECHECK FAIL: %d. ", read_data_size);
        return false;
    }else
        ROS_INFO(" FILE SIZE RECHECK SUCCESS: %d. ", read_data_size);

    memcpy(&file_head, read_pack + 0, sizeof(FILE_HEAD));                                                   // HEAD
    check_sta &= (file_head.check_char == file_head_normal.check_char);

    check_sta &= ((sizeof(FILE_NODE) * file_head.num_nodes) == file_head.dtasize_nodes);
    check_sta &= ((sizeof(FILE_EDGE) * file_head.num_edges) == file_head.dtasize_edges);
    check_sta &= ((sizeof(float) * file_head.num_grids) == file_head.dtasize_grids);

    memcpy(&file_interval_read, read_pack + 0 + sizeof(FILE_HEAD), size_file_interval);
    check_sta &= file_interval_read.check();

    if(check_sta)
        ROS_INFO(" GRAPH PACK DATA: HEAD READY");

    file_nodes.resize(file_head.num_nodes);
    memcpy(file_nodes.data(), read_pack + file_head.locate2nodes, file_head.dtasize_nodes);                 // NODES
    memcpy(&file_interval_read, read_pack + file_head.locate2nodes + file_head.dtasize_nodes, size_file_interval);
    check_sta &= file_interval_read.check();

    if(check_sta)
        ROS_INFO(" GRAPH PACK DATA: NODES READY");

    file_edges.resize(file_head.num_edges);
    memcpy(file_edges.data(), read_pack + file_head.locate2edges, file_head.dtasize_edges);                 // EDGES
    memcpy(&file_interval_read, read_pack + file_head.locate2edges + file_head.dtasize_edges, size_file_interval);
    check_sta &= file_interval_read.check();

    if(check_sta)
        ROS_INFO(" GRAPH PACK DATA: EDGES READY");

    file_grids.resize(file_head.num_grids);
    memcpy(file_grids.data(), read_pack + file_head.locate2grids, file_head.dtasize_grids);                 // GRIDS
    memcpy(&file_interval_read, read_pack + file_head.locate2grids + file_head.dtasize_grids, size_file_interval);
    check_sta &= file_interval_read.check();

    if(check_sta)
        ROS_INFO(" GRAPH PACK DATA: GRIDS READY");


    memcpy(&file_tail, read_pack + file_head.locate2tail, sizeof(FILE_TAIL));                               // TAIL
    check_sta &= file_tail.check();

    if(check_sta)
        ROS_INFO(" GRAPH PACK DATA: TAIL READY");

    if(check_sta)
        ROS_INFO(" PACK READ SUCCESSFUL.");
    else
        ROS_INFO(" PACK READ FAIL.");

    free(read_pack);

    close(file_fd);

    return check_sta;
}

/// *******************************************************************************
//  ************************** color line... beautiful!! **************************
/// *******************************************************************************

int main(int argc, char **argv)
{
    bool mode;

    ros::init(argc, argv, "xyi2_buildmap");
    ros::NodeHandle nstru("~");

    bool param_sta = false, bool_sta = false;
    std::string catkin_route;
    std::string basic_route = "/src/xyi2_bringup/graphfiles";

    MAP_PARAM_BLOCK map_param;

    /// mode == true, just draw the obstacle.
    /// mode == false, will draw the obstacle and path.
    mode = false;

    param_sta = nstru.getParam("workspace_route", catkin_route);

    map_param.map_lengthofsize = 102.4f;
    map_param.map_resolution = 0.05f;

    map_manage.func_map_param_init(map_param);

    if(param_sta)
        catkin_route = catkin_route + basic_route;
    else
        ROS_ERROR(" ERROR: WORKSPACE ROUTE.");

    if(param_sta)
        bool_sta = graph_read_file(catkin_route);

    if(bool_sta)
    {
        std::string text_yes = "yes", text_no = "no";
        std::string opt_text;
        cout << "[ INFO ] NEED GRAPH OPTIMIZATION? REPLY 'yes' or 'no': ";
        cin >> opt_text;

        if(opt_text == text_yes)
        {
            ROS_INFO(" GRAPH OPTIMIZATION BEGIN.");

            bool_sta = false;

            if(graph_optimizer())
            {
                graph_write_file(catkin_route);

                bool_sta = map_manage.build_map(mode);
            }else
                bool_sta = false;
        }
        else if(opt_text == text_no)
        {
            bool_sta = map_manage.build_map(mode);
        }

        else{
            ROS_INFO(" INPUT ERROR, EXIST THE PROGRAM.");
            bool_sta = false;
        }

        if(bool_sta)
        {
            while(ros::ok())
            {
                try
                {
                    boost::this_thread::interruption_point();
                    ////////////////////////////////////////////////////////////////

                    ROS_INFO("LOOP TO PUBLISH THE RESULT MAP.");

                    map_manage.nmap_pubcheck();
                    map_manage.bmap_pubcheck();
                    map_manage.graph_poses_pubcheck();
                    map_manage.graph_edges_pubcheck();

                    ////////////////////////////////////////////////////////////////
                    boost::this_thread::sleep(boost::posix_time::milliseconds(250));
                }
                catch(boost::thread_interrupted&)
                {

                }
            }
        }

    }else
        ROS_ERROR(" READ FILE FAIL.");

    graph_globaldata_clear();

    //ros::spin();

    return 0;
}



