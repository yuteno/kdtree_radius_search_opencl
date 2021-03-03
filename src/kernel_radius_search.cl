// 3-d position and id
typedef struct point {
  float x, y, z;
  int id;
} point;

typedef struct tag_kdtree_node {
  int depth;
  int left_index, right_index;
  float rightmost, leftmost, upmost, downmost, zlowmost, zupmost;
  point location;
  int axis;
  float axis_val;
  __constant struct tag_kdtree_node * child1;
  __constant struct tag_kdtree_node * child2;
} kdtree_node;

/**
 * @brief return the indexes of 7 neighbor voxels of the reference point
 * @param lidar_points_x         (input) 1-D array of x value of input points (n elements)
 * @param lidar_points_y         (input) 1-D array of y value of input points (n elements)
 * @param lidar_points_z         (input) 1-D array of z value of input points (n elements)
 * @param map_points_x           (input) 1-D array of x value of map points (? elements)
 * @param map_points_y           (input) 1-D array of y value of map points (? elements)
 * @param map_points_z           (input) 1-D array of z value of map points (? elements)
 * @param node_indexes           (input) 1-D array of indexes of map points (? elements)
 * @param root_node              (input) kdtree root node
 * @param n                      (input) Number of input points
 * @param limit                  (input) limit of neighbors
 * @param radius                 (input) search radius
 * @param neighbor_candidate_indexes    (output) 1-D array of indexes of neighbor candidates (? elements)
 * @param neighbor_candidate_dists      (output) 1-D array of distances of neighbor candidates (? elements)
 */

__kernel void radiusSearchCL(const global float * lidar_points_x,
                        const global float * lidar_points_y,
                        const global float * lidar_points_z,
                        const global float * map_points_x,
                        const global float * map_points_y,
                        const global float * map_points_z,
                        const global int * node_indexes,
                        __constant kdtree_node * root_node,
                        const int n,
                        const int limit,
                        const float radius,
                        global int * neighbor_candidate_indexes,
                        global float * neighbor_candidate_dists) {

  // 1d range kernel for the point cloud
  int item_index = get_global_id(0);
  if (item_index >= n)
    return;

  float4 reference_point = (float4)(lidar_points_x[item_index],
                                    lidar_points_y[item_index],
                                    lidar_points_z[item_index],
                                    0);

  if (item_index == 0)
    printf("ref point (%f, %f, %d), radius %f, limit %d\n", lidar_points_x[item_index], lidar_points_y[item_index], lidar_points_z[item_index], radius, limit);

  __constant kdtree_node  *current_node = root_node;
  int neighbors_count = 0;

  int level = 0;
  float dist;
  float4 dist_square;
  // radius search
  while (true) {
    if ((current_node->child1 == NULL) && (current_node->child2 == NULL)) {
      //reached to leaf node

      for (int i = current_node->left_index; i < current_node->right_index; ++i) {
        int index = node_indexes[i];
        float4 map_point = (float4)(map_points_x[index],
                                    map_points_y[index],
                                    map_points_z[index],
                                    0);
        for (int d = 0; d < 4; d++) {
          dist_square[d] = (map_point[d] - reference_point[d]) * (map_point[d] - reference_point[d]);
        }
        //float4 dist_square = (map_point - reference_point) * (map_point - reference_point);
        dist = sqrt(dist_square[0] + dist_square[1] + dist_square[2]);
        if (dist < radius) {
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
      float4 map_point = (float4)(map_points_x[index],
                                  map_points_y[index],
                                  map_points_z[index],
                                  0.0f);
      //dist_square = (map_point - reference_point) * (map_point - reference_point);

      for (int d = 0; d < 4; d++) {
        dist_square[d] = (map_point[d] - reference_point[d]) * (map_point[d] - reference_point[d]);
      }
      dist = sqrt(dist_square.x + dist_square.y + dist_square.z);
      if (dist < radius) {
        neighbor_candidate_indexes[limit*item_index + neighbors_count] = index;
        neighbor_candidate_dists[limit*item_index + neighbors_count] = dist;
        neighbors_count++;
      }
      if (neighbors_count >= limit){
        break;
      }
      float val = reference_point[current_node->axis];
      float diff = val - current_node->axis_val;

      __constant kdtree_node * next;
      if (diff < 0 && current_node->child1) {
        next = current_node->child1;
        // printf("%d in (%d), select child1\n", item_index, n);
      } else {
        next = current_node->child2;
        // printf("%d in (%d), select child2\n", item_index, n);
      }
      current_node = next;
    }
  }
}
