#ifndef __graph_io_H
#define __graph_io_H

    #include "../struct_graph.h"

    #include <sys/types.h>
    #include <sys/stat.h>
    #include <sys/fcntl.h>

    struct FILE_HEAD
    {
    public:
        FILE_HEAD()
        {
            check_char = 0XAA;

            dtasize_total = 0;

            dtasize_nodes = 0;
            num_nodes = 0;
            locate2nodes = 0;

            dtasize_edges = 0;
            num_edges = 0;
            locate2edges = 0;

            dtasize_grids = 0;
            num_grids = 0;
            locate2grids = 0;

        }

        char check_char;                    // 1 byte

        /// FILE_HEAD

        unsigned int dtasize_total;         // 4bytes

        // > FILE_INTERVAL

        unsigned int  dtasize_nodes;        // 4 bytes
        unsigned int  num_nodes;            // 4 bytes  [Unit Type: FILE_NODE]
        unsigned int  locate2nodes;         // 4 bytes

        // > FILE_INTERVAL

        unsigned int  dtasize_edges;        // 4 bytes
        unsigned int  num_edges;            // 4 bytes  [Unit Type: FILE_EDGE]
        unsigned int  locate2edges;         // 4 bytes

        // > FILE_INTERVAL

        unsigned int dtasize_grids;         // 4 bytes
        unsigned int num_grids;             // 4 bytes  [Unit Type: float]
        unsigned int locate2grids;          // 4 bytes

        // > FILE_INTERVAL

        /// FILE_TAIL
        unsigned int  locate2tail;          // 4 bytes
    };

    struct FILE_TAIL
    {
    public:
        FILE_TAIL()
        {
            check_chars[0] = 0XFF;
            check_chars[1] = 0XAA;
            check_chars[2] = 0XFF;
        }

        bool check(void)
        {
            if((unsigned char)(check_chars[0]) != 0XFF)
                return false;
            if((unsigned char)(check_chars[1]) != 0XAA)
                return false;
            if((unsigned char)(check_chars[2]) != 0XFF)
                return false;

            return true;
        }

        char check_chars[3];                // 3 bytes
    };

    struct FILE_INTERVAL
    {
    public:
        FILE_INTERVAL()
        {
            check_chars[0] = 0XAA;
            check_chars[1] = 0XFF;
            check_chars[2] = 0XAA;
        }

        bool check(void)
        {
            if((unsigned char)(check_chars[0]) != 0XAA)
                return false;
            if((unsigned char)(check_chars[1]) != 0XFF)
                return false;
            if((unsigned char)(check_chars[2]) != 0XAA)
                return false;

            return true;
        }

        char check_chars[3];                // 3 bytes
    };

    struct FILE_NODE
    {
    public:
        FILE_NODE()
        {

        }

        unsigned int node_id;               // 4 bytes
        Eigen::Vector3f X;                  // 12 bytes

        unsigned int num_edges;             // 4 bytes
        Eigen::Vector2i edges_id_ft;        // 8 bytes (from (0) to (1))

        unsigned int   nmap_llength;        // 4 bytes
        unsigned int   nmap_size;           // 4 bytes
        float nmap_reso;                    // 4 bytes
        Eigen::Vector2f nmap_xiuz;          // 8 bytes
        Eigen::Vector2i nmap_gid_ft;        // 8 bytes (from (0) to (1))

    };

    struct FILE_EDGE
    {
    public:
        FILE_EDGE()
        {

        }

        unsigned int edge_id;               // 4 bytes
        Eigen::Vector2i nodes_id_ft;        // 8 bytes

        Eigen::Vector3f C;                  // 12 bytes
        Eigen::Vector3f D;                  // 12 bytes

    };

#endif
