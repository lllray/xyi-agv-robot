#ifndef __basic_quadtree_H
#define __basic_quadtree_H

    // Sys Part **********************************************/
    #include <ros/ros.h>
    #include <sstream>
    #include <iostream>
    #include <string>
    using namespace std;

    // C_Plus Standard Library *******************************/
    #include <algorithm>
    #include <signal.h>
    #include <vector>

    #include "eigen3/Eigen/Dense"
    #include "eigen3/Eigen/Geometry"
    using namespace Eigen;

    #include "struct_map_config.h"

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    /* **********************
        UL(0x03)|UR(0x01)
        --------|---------
        BL(0x02)|BR(0x00)
    ********************** */
    enum quadtype{NODE=0X00, LEAF=0xff, BR=0x01, UR=0x03, UL=0x07, BL=0x05};

    typedef class quadtree_ob
    {
    public:
        quadtree_ob(float _x,float _y,
                    float _h_width,float _h_height) : x(_x), y(_y), h_width(_h_width), h_height(_h_height)
        {
        }
        ~quadtree_ob(){}

    public:
        //对象的属性，例如坐标和长宽，以 中心点 为锚点
        float x;            // center posi.x
        float y;            // center posi.y
        float h_width;      // half width
        float h_height;     // half height
    }quadtree_ob;

    typedef class quadtree_node
    {
    public:
        quadtree_node(float _x, float _y, float _h_width, float _h_height,
                      int _depth, int _maxdepth,
                      quadtype _obtype,
                      quadtree_node *_parent
                      ) : object(_x, _y, _h_width, _h_height),
                          depth(_depth), maxdepth(_maxdepth),
                          ob_type(_obtype)
        {
            parentnode = _parent;

            childnodes[0] = NULL,
            childnodes[1] = NULL,
            childnodes[2] = NULL,
            childnodes[3] = NULL;
        }

        ~quadtree_node()
        {
            if (depth == maxdepth)
                return;

            // if not leaf, destroy parent?
            parentnode = NULL;
        }

        void insert_object(quadtree_ob *obe);

    private:
        quadtype ob_type;
        quadtree_ob object;
        float object_weight;

        quadtree_node *parentnode;
        quadtree_node *childnodes[4];

        int depth;
        int maxdepth;

        bool iscontain(float px, float py, float h_w, float h_h, quadtree_ob *obe) const;
        bool iscontain(float px, float py, float h_w, float h_h, quadtree_node *obe_node) const;

    }quadtree_node;


    typedef class quadtree_t
    {
    public:
        quadtree_t()
        {

        }

        ~quadtree_t(){}

        quadtree_node *roots;

        void init_tree(int map_whsize, float map_reso);

    private:

        int max_depth;

    }quadtree_t;

    //////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

