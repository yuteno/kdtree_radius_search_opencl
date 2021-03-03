#ifndef RADIUS_SAERCH_H
#define RADIUS_SEARCH_H

#define XAxis 0
#define YAxis 1
#define ZAxis 2

// 3-d position and id
typedef struct point {
  float x, y, z;
  int id;
} point;

typedef struct tag_kdtree_node {
  int depth; // nodeの深さ
  int left_index, right_index; // このnode以下に含まれる点群のindex
  float rightmost, leftmost, upmost, downmost, zlowmost, zupmost; // このnode以下に含まれる点群のx, y, zの最大・最小
  point location; // そのnodeに対応する分割点
  int axis; // このnodeに対応する分割軸
  float axis_val; // 代表点の分割軸の値
  struct tag_kdtree_node * child1; // 子node
  struct tag_kdtree_node * child2;
} kdtree_node;

kdtree_node * kdtree(kdtree_node * root_node,
                     unsigned * alloc_pointer,
                     struct point * pointlist,
                     int left, int right, unsigned depth,
                     int * node_indexes);

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
                   float * neighbor_candidate_dists);
#endif
