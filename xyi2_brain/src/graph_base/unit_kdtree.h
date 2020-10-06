#ifndef __unit_kdtree_H
#define __unit_kdtree_H

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    #define KEY_TYPE_DIM    2
    typedef Eigen::Vector2i KEY_TYPE;

    typedef struct kdtree_node
    {
      // Depth in the tree
      bool isleaf;                      // leaf: is a leaf?,
      int depth;                        // depth: leaf's depth

      // Pivot dimension and value
      char pivot_dim;                   // split dimension
      float pivot_value;                // split value

      // The key for this node
      KEY_TYPE key;                     // key value

      // The value for this node
      int value;                        // value

      // Child nodes
      kdtree_node *children[2];

    } kdtree_node;

    typedef class kdtree_t
    {
    public:
      kdtree_t()
      {
          isinit = false;
      }

      ~kdtree_t()
      {
          free(root);
          free(queue_nodes);
      }

      bool isinit;

      // The root node of the tree
      kdtree_node *root;

      // The number of nodes in the tree
      int node_count, node_maxcount;
      int depth_count;

      // The number of leaf nodes in the tree
      int leaf_count;

      kdtree_node *queue_nodes;

    } kdtree_t;

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    kdtree_t *kdtree_init(int max_size, float *_key_reso);
    bool kdtree_state(kdtree_t *self);
    void kdtree_free(kdtree_t *self);
    void kdtree_clear(kdtree_t *self);

    kdtree_node *kd_tree_insert_node(kdtree_t *self_tree,
                                    kdtree_node *parent,
                                    kdtree_node *node,
                                    KEY_TYPE &key);

    kdtree_node *kd_tree_find_node(kdtree_node *parent,
                                   kdtree_node *node,
                                   KEY_TYPE &key);


    kdtree_node *kd_tree_find_add_node(kdtree_t *self_tree,
                                       kdtree_node *parent,
                                       kdtree_node *node,
                                       KEY_TYPE &key);

    int kd_tree_finddist_inrange_node(kdtree_t *self_tree,
                                      kdtree_node *parent,
                                      kdtree_node *node,
                                      KEY_TYPE &key);

    void kd_tree_print(kdtree_node *tree_node);

    //////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

