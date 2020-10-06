#include "basic_quadtree.h"
#include <stdlib.h>

/* ********************************************************************************
 * author: chenyingbing
 * time: 20170509   10:00   in XIAMEN University
 * illustration:
 *      the struct of quad tree.
 *
 *      [ERROR] Unfinished
 *
 * *******************************************************************************/

/// quadtree_node

void quadtree_node::insert_object(quadtree_ob *obe)
{

}

bool quadtree_node::iscontain(float px, float py, float h_w, float h_h, quadtree_ob *obe) const
{
    if( ( obe->x >= px) &&
        ((obe->x + obe->h_width) <= (px + h_w)) &&
        ( obe->y >= py) &&
        ((obe->y + obe->h_height) <= (px + h_h)))
            return true;


    return false;
}

bool quadtree_node::iscontain(float px, float py, float h_w, float h_h, quadtree_node *obe_node) const
{
    if( ( obe_node->object.x >= px) &&
        ((obe_node->object.x + obe_node->object.h_width) <= (px + h_w)) &&
        ( obe_node->object.y >= py) &&
        ((obe_node->object.y + obe_node->object.h_height) <= (px + h_h)))
            return true;

    return false;
}

/// quadtree_t
void quadtree_t::init_tree(int map_whsize, float map_reso)
{
    float range_x = map_whsize * map_reso * 0.5f;
    float range_y = map_whsize * map_reso * 0.5f;

    max_depth = std::log(map_whsize);

    roots = new quadtree_node(range_x, range_y, range_x, range_y,
                              0, max_depth,
                              NODE,
                              NULL
                              );
}





