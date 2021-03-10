#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <fstream>
#include <string>
#include "radius_search.h"


#define MAX_PLATFORMS (10)
#define MAX_DEVICES (10)
#define MAX_SOURCE_SIZE (100000)


size_t N, N_source;

float * map_points_x;
float * map_points_y;
float * map_points_z;
int * node_indexes; // 葉の通し番号とpoint.idの対応付を行う配列
float leaf_size_x, leaf_size_y, leaf_size_z;
kdtree_node * root_node;

int main(int argc, char ** argv) {
  const char * map_file = "./data/pointcloud_map.pcd";
  std::string filename = "./data/kdtree_binary.dat";
  std::ifstream iFstrm(filename.c_str(), std::ios::binary);
  if (iFstrm.fail()) {
    std::cerr << "Error Could not open" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, *p_cloud);
  printf("Map Point Cloud: %s %ld points\n", map_file, p_cloud->points.size());

  // voxel grid filterを使いdownsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(2.0f, 2.0f, 2.0f);

  voxelgrid.setInputCloud(p_cloud);
  printf("Downsampling mapfile to voxels...\n");
  voxelgrid.filter(*downsampled);
  printf("%ld voxels are generated.\n", downsampled->points.size());
  *p_cloud = *downsampled;

  N = p_cloud->points.size();

  // sizes of each arrays
  size_t map_size = N * sizeof(float);
  size_t kdtree_size = N * sizeof(kdtree_node);
  printf("TOTAL TREE MEMORY SIZE (%ld * %ld) = %ldB\n", N, sizeof(kdtree_node), kdtree_size);
  struct point datapoints[N]; // kdtreeの構築に使うpoint構造体の配列

  root_node = (kdtree_node *)malloc(kdtree_size);
  //printf("Use C radius search\n");

  // kdtreeの構築
  printf("read kd tree...\n");
  iFstrm.read((char*) root_node, kdtree_size);
  for (int i = 0; i < 10; i++) {
    printf("%d, %d, %f, (%f, %f, %f, %d), %d, %d, %p, %p\n", root_node[i].depth, root_node[i].axis, root_node[i].axis_val, root_node[i].location.x, root_node[i].location.y, root_node[i].location.z, root_node[i].location.id, root_node[i].left_index, root_node[i].right_index, root_node[i].child1, root_node[i].child2);
  }
  printf("done.\n");


  printf("pointer converting\n");
  for (int i = 0; i < N; i++) {
    root_node[i].child1 = (kdtree_node *)((long long int)root_node[i].child1 + (long long int)&root_node[0]);
    root_node[i].child2 = (kdtree_node *)((long long int)root_node[i].child2 + (long long int)&root_node[0]);
  }
  printf("done\n");


  // kdtreeの確認
  kdtree_node *temp = root_node;
  printf("tree: \n");
  for (int i = 0; i < 10; i++) {
    if (temp != NULL) {
      printf("%d, %d, %f, (%f, %f, %f, %d), %d, %d, %p, %p\n", temp->depth, temp->axis, temp->axis_val, temp->location.x, temp->location.y, temp->location.z, temp->location.id, temp->left_index, temp->right_index, temp->child1, temp->child2);
      temp = temp->child1;
    }
  }
  // release allocated memory
  free(node_indexes);
  free(root_node);
  iFstrm.close();
  return 0;
}


