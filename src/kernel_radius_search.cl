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
 * @param trans_cloud_points_x       (input) 1-D array of x value of input points (n elements)
 * @param trans_cloud_points_y       (input) 1-D array of y value of input points (n elements)
 * @param trans_cloud_points_z       (input) 1-D array of z value of input points (n elements)
 * @param map_points_x           (input) 1-D array of x value of map points (? elements)
 * @param map_points_y           (input) 1-D array of y value of map points (? elements)
 * @param map_points_z           (input) 1-D array of z value of map points (? elements)
 * @param node_indexes          (input) 1-D array of indexes of map points (? elements)
 * @param root_node            (input) kdtree root node
 * @param n                (input) Number of input points
 * @param limit              (input)
 * @param neighbor_candidate_indexes    (output) 1-D array of indexes of neighbor candidates (? elements)
 * @param neighbor_candidate_dists      (output) 1-D array of distances of neighbor candidates (? elements)
 */

__kernel void kdtreeNDT(const global float * trans_cloud_points_x,
                        const global float * trans_cloud_points_y,
                        const global float * trans_cloud_points_z,
                        const global float * map_points_x,
                        const global float * map_points_y,
                        const global float * map_points_z,
                        const global int * node_indexes,
                        __constant kdtree_node * root_node,
                        const int n,
                        const int limit,
                        global int * neighbor_candidate_indexes,
                        global float * neighbor_candidate_dists) {

  // 1d range kernel for the point cloud
  int item_index = get_global_id(0);
  if (item_index >= n)
    return;

  float4 reference_point = (float4)(trans_cloud_points_x[item_index],
                                    trans_cloud_points_y[item_index],
                                    trans_cloud_points_z[item_index],
                                    0);

  bool finished = false;
  __constant kdtree_node  *current_node = root_node;
  int neighbors_count = 0;

  //neighbor_candidate_dists[item_index] = 330;
  /*
    if (item_index == 0) {
    printf("item:%d, root_node (at :%lx)(%f, %f, %f), child1:%lx, child2:%lx\n", item_index, current_node, current_node->location.x, current_node->location.y, current_node->location.z, current_node->child1, current_node->child2);
    printf("item:%d, map_points (%f, %f, %f), \n", item_index, map_points_x[item_index], map_points_y[item_index], map_points_z[item_index]);
    }
  */

  int level = 0;
  // radius search
  while (true) {
    if ((current_node->child1 == NULL) && (current_node->child2 == NULL)) {
      for (int i = current_node->left_index; i < current_node->right_index; ++i) {
        int index = node_indexes[i];
        float4 map_point = (float4)(map_points_x[index],
                                    map_points_y[index],
                                    map_points_z[index],
                                    0);
        float4 dist_square = (map_point - reference_point) * (map_point - reference_point);
        float dist = sqrt(dist_square.x + dist_square.y + dist_square.z);
        neighbor_candidate_indexes[limit*item_index + neighbors_count] = current_node->location.id;
        //neighbor_candidate_indexes[limit*item_index + neighbors_count] = index;
        neighbor_candidate_dists[limit*item_index + neighbors_count] = dist;
        neighbors_count++;
        if (neighbors_count >= limit){
          break;
        }
      }
      return;
    } else {
      float val;
      switch (current_node->axis) {
      case 0:
        val = reference_point.x;
        break;
      case 1:
        val = reference_point.y;
        break;
      default:
        val = reference_point.z;
        break;
      }
      float diff = val - current_node->axis_val;
      current_node = (diff < 0) ? current_node->child1 : current_node->child2;
    }
  }

}
