#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include "radius_search.h"

/**
 * @brief return the indexes of 7 neighbor voxels of the reference point
 * @param trans_cloud_points_x (input) 1-D array of x value of input points (n elements)
 * @param trans_cloud_points_y (input) 1-D array of y value of input points (n elements)
 * @param trans_cloud_points_z (input) 1-D array of z value of input points (n elements)
 * @param map_points_x         (input) 1-D array of x value of map points (? elements)
 * @param map_points_y         (input) 1-D array of y value of map points (? elements)
 * @param map_points_z         (input) 1-D array of z value of map points (? elements)
 * @param node_indexes         (input) 1-D array of indexes of map points (? elements)
 * @param root_node            (input) kdtree root node
 * @param n                    (input) Number of input points
 * @param limit                (input) limit of neighbors
 * @param float                (input) search radius
 * @param neighbor_candidate_indexes (output) 1-D array of indexes of neighbor candidates (? elements)
 * @param neighbor_candidate_dists   (output) 1-D array of distances of neighbor candidates (? elements)
 */

void radiusSearchC(const float * trans_cloud_points_x,
                   const float * trans_cloud_points_y,
                   const float * trans_cloud_points_z,
                   const float * map_points_x,
                   const float * map_points_y,
                   const float * map_points_z,
                   const int * node_indexes,
                   const kdtree_node * root_node,
                   const int n,
                   const int limit,
                   const float radius,
                   int * neighbor_candidate_indexes,
                   float * neighbor_candidate_dists) {

  // 1d range kernel for the point cloud
  for (int item_index = 0; item_index < n; item_index++) {
    float reference_point [4] = {
      trans_cloud_points_x[item_index],
      trans_cloud_points_y[item_index],
      trans_cloud_points_z[item_index],
      0.0f};

    const kdtree_node * current_node = root_node;
    int neighbors_count = 0;

    //neighbor_candidate_dists[item_index] = 330;

    if (item_index == 0) {
      printf("item:%d, root_node (at :%p)(%f, %f, %f), child1:%p, child2:%p\n", item_index, current_node, current_node->location.x, current_node->location.y, current_node->location.z, current_node->child1, current_node->child2);
      printf("item:%d, map_points (%f, %f, %f), \n", item_index, map_points_x[item_index], map_points_y[item_index], map_points_z[item_index]);
    }

    // radius search
    float dist_square[4];
    float dist;
    while (true) {
      if ((current_node->child1 == NULL) && (current_node->child2 == NULL)) {
        // ここのprintfから判るが、常にleftからrightまでの幅的に常に1つしか近傍が返却されない。kdtree_constructionに想定違いがありそう。
        //printf("point %d (%d to %d)\n", item_index, current_node->left_index, current_node->right_index);
        for (int i = current_node->left_index; i < current_node->right_index; ++i) {
          int index = node_indexes[i];
          float map_point [4] = {
            map_points_x[index],
            map_points_y[index],
            map_points_z[index],
            0.0f};
          //float dist_square [4];
          for (int d = 0; d < 4; d++) {
            dist_square[d] = (map_point[d] - reference_point[d]) * (map_point[d] - reference_point[d]);
          }
          dist = sqrtf(dist_square[0] + dist_square[1] + dist_square[2]);
          if (dist < radius) {
            //printf("leaf node, get radius, %d\n", limit*item_index + neighbors_count);
            neighbor_candidate_indexes[limit*item_index + neighbors_count] = index;
            neighbor_candidate_dists[limit*item_index + neighbors_count] = dist;
            neighbors_count++;

          }
          if (neighbors_count >= limit){
            break;
          }
        }
        break;
      } else {
        int index = node_indexes[current_node->left_index + (current_node->right_index - current_node->left_index-1) / 2];
        float map_point [4] = {
          map_points_x[index],
          map_points_y[index],
          map_points_z[index],
          0.0f};
        for (int d = 0; d < 4; d++) {
          dist_square[d] = (map_point[d] - reference_point[d]) * (map_point[d] - reference_point[d]);
        }
        float dist = sqrtf(dist_square[0] + dist_square[1] + dist_square[2]);
        if (dist < radius) {
          //printf("non-leaf node, get radius, %d\n", limit*item_index + neighbors_count);
          neighbor_candidate_indexes[limit*item_index + neighbors_count] = index;
          neighbor_candidate_dists[limit*item_index + neighbors_count] = dist;
          neighbors_count++;
        }
        if (neighbors_count >= limit){
          break;
        }
        float val = reference_point[current_node->axis];
        float diff = val - current_node->axis_val;


        const kdtree_node * next;
        // 片方がNULLである場が、オリジナルのsearchLevelでは考慮されていないような気がする。kdtree_constructionに想定違いがありそう。
        if (diff < 0 && current_node->child1) {
          next = current_node->child1;
          // printf("%d in (%d), select child1\n", item_index, n);
        } else {
          next = current_node->child2;
          // printf("%d in (%d), select child2\n", item_index, n);
        }
        // 木のworstDistの更新から、戻る場合の処理って、結局必要なんでしょうかね...?
        current_node = next;
      }
    }
  }
}
