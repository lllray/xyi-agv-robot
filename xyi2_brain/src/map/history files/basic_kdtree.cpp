#include "basic_kdtree.h"
#include <stdlib.h>

/* ********************************************************************************
 * author: chenyingbing
 * time: 20170323   15:28   in XIAMEN University
 * illustration:
 *      the struct of kd tree copied from amcl pkg.
 *
 *
 * *******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// Create a tree
kdtree_t *kdtree_init(int max_size)
{
    kdtree_t *self;

    self = (kdtree_t *) std::malloc(sizeof(kdtree_t));

    self->root = NULL;

    self->leaf_count = 0;
    self->depth_count = 0;
    self->node_count = 0;
    self->node_maxcount = max_size;
    self->queue_nodes = (kdtree_node *) std::malloc(self->node_maxcount * sizeof(kdtree_node));

    self->isinit = true;

    return self;
}

bool kdtree_state(kdtree_t *self)
{
    return (self->node_count >= self->node_maxcount);
}

////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
void kdtree_free(kdtree_t *self)
{
  free(self->queue_nodes);
  free(self);
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
void kdtree_clear(kdtree_t *self)
{
  self->root = NULL;
  self->leaf_count = 0;
  self->node_count = 0;
  self->depth_count = 0;

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Recursive node insert
kdtree_node *kd_tree_insert_node(kdtree_t *self_tree,
                                kdtree_node *parent,
                                kdtree_node *node,
                                KEY_TYPE &key)
{
    if(self_tree->isinit)
    {
        // bug:
        if((self_tree->node_count > 0)&&(self_tree->root == NULL))
            self_tree->root = self_tree->queue_nodes;

        int i;
        int split, max_split;

        // If the node doesnt exist yet...
        if (node == NULL)
        {
          node = self_tree->queue_nodes + self_tree->node_count++;
          memset(node, 0, sizeof(kdtree_node));

          node->children[0] = NULL;
          node->children[1] = NULL;

          node->isleaf = true;

          if (parent == NULL)
            node->depth = 0;
          else
            node->depth = parent->depth + 1;

          node->key = key;
          node->value = 1;

          self_tree->leaf_count += 1;
          if(node->depth >= self_tree->depth_count)
          {
              self_tree->depth_count = node->depth;
          }
        }

        // If the node exists, and it is a leaf node...
        else if (node->isleaf)
        {
          // If the keys are equal, do nothing
          if (node->key == key)
          {
            node->value += 1;
          }

          // The keys are not equal, so split this node
          else
          {
            // Find the dimension with the largest variance and do a mean
            // split
            max_split = 0;
            node->pivot_dim = -1;
            for (i = 0; i < KEY_TYPE_DIM; i++)
            {
              split = abs(key.data()[i] - node->key.data()[i]);
              if (split > max_split)
              {
                max_split = split;
                node->pivot_dim = i;
              }
            }

            node->pivot_value = (key.data()[node->pivot_dim] + node->key.data()[node->pivot_dim]) / 2.0f;

            if (key.data()[node->pivot_dim] < node->pivot_value)
            {
              node->children[0] = kd_tree_insert_node(self_tree, node, NULL, key);
              node->children[1] = kd_tree_insert_node(self_tree, node, NULL, node->key);
            }
            else
            {
              node->children[0] = kd_tree_insert_node(self_tree, node, NULL, node->key);
              node->children[1] = kd_tree_insert_node(self_tree, node, NULL, key);
            }

            node->isleaf = false;
            self_tree->leaf_count -= 1;
          }
        }

        // If the node exists, and it has children...
        else
        {
          if (key.data()[node->pivot_dim] < node->pivot_value)
            kd_tree_insert_node(self_tree, node, node->children[0], key);
          else
            kd_tree_insert_node(self_tree, node, node->children[1], key);
        }

        return node;
    }

    return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Recursive node search
// Note:    the return node should be checked, if it is a leaf, we get the node, if not, we get the parent!

kdtree_node *kd_tree_find_node(kdtree_node *parent, kdtree_node *node, KEY_TYPE &key)
{
  if (node->isleaf)
  {
    // If the keys are the same...
    if (key == node->key)
      return node;
    else
      return parent;
  }
  else
  {
    // If the keys are different...
    if (key.data()[node->pivot_dim] < node->pivot_value)
      return kd_tree_find_node(node, node->children[0], key);
    else
      return kd_tree_find_node(node, node->children[1], key);
  }

  return NULL;
}

// find the node, if not existing, add it!, if existing, do nothing and return NULL.
kdtree_node *kd_tree_find_add_node(kdtree_t *self_tree,
                                      kdtree_node *parent,
                                      kdtree_node *node,
                                      KEY_TYPE &key)
{
    if(self_tree->isinit)
    {
        if (node->isleaf)
        {
        // If the keys are the same...
        if (key == node->key)
          return NULL;  // find!
        else
          return kd_tree_insert_node(self_tree,
                                    parent,
                                    node,
                                    key);
        }
        else
        {
        // If the keys are different...
        if (key.data()[node->pivot_dim] < node->pivot_value)
          return kd_tree_find_add_node(self_tree, node, node->children[0], key);
        else
          return kd_tree_find_add_node(self_tree, node, node->children[1], key);
        }

        return NULL;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Recursive print
void kd_tree_print(kdtree_node *tree_node)
{
    if(tree_node != NULL)
    {
        cout << "<>";
        if(tree_node->isleaf)
        {
            cout << "[leaf]" << "(" << tree_node->depth << ":"
                 << tree_node->key.data()[0] << ","
                 << tree_node->key.data()[1] << ","
                 << tree_node->key.data()[2] << ")";
        }else{
            cout << "[node]" << "(" << tree_node->depth << ":"
                 << tree_node->pivot_dim << "," << tree_node->pivot_value << ")";
            kd_tree_print(tree_node->children[0]);
            kd_tree_print(tree_node->children[1]);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


