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
#include "radius_search.h"

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl2.hpp>
#endif

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define CL_TARGET_OPENCL_VERSION 220

#define MAX_PLATFORMS (10)
#define MAX_DEVICES (10)
#define MAX_SOURCE_SIZE (100000)

static int setup_ocl(cl_uint, cl_uint, char *);
static void kdtree_calc(int isOCL);

cl_command_queue Queue;
cl_kernel k_kdtree;
cl_context context = NULL;
cl_program program = NULL;
size_t N, N_source;
int limit;
float radius;

float * trans_cloud_points_x;
float * trans_cloud_points_y;
float * trans_cloud_points_z;

float * map_points_x;
float * map_points_y;
float * map_points_z;
int * node_indexes; // 葉の通し番号とpoint.idの対応付を行う配列
int * neighbor_candidate_indexes;
float * neighbor_candidate_dists;
float leaf_size_x, leaf_size_y, leaf_size_z;
cl_mem d_points_x, d_points_y, d_points_z, d_neighbor_candidates;
cl_mem d_map_points_x, d_map_points_y, d_map_points_z, d_neighbor_candidate_dists;
cl_mem d_node_indexes;
kdtree_node * root_node;

int main(int argc, char ** argv) {
  int platform = 0;
  int device = 0;
  const char * map_file = "./data/pointcloud_map.pcd";
  const char * lidar_file = "./data/lidar_points.pcd";

  // configuration
  int OCL = 0;
  radius = 1.0f;
  limit = 5;
  // -------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, *p_cloud);
  printf("Map Point Cloud: %s %ld points\n", map_file, p_cloud->points.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(lidar_file, *p_cloud_source);
  printf("LiDAR Point Cloud: %s %ld points\n", lidar_file, p_cloud_source->points.size());

  N_source = p_cloud_source->points.size();

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
  size_t points_size = N_source * sizeof(float);
  size_t map_size = N * sizeof(float);
  size_t neighbor_candidates_size = N * limit * sizeof(int);
  size_t kdtree_size = N * sizeof(kdtree_node);
  printf("#neighbor voxels are limited %d for each LiDAR point.\n", limit);
  printf("TOTAL TREE MEMORY SIZE (%ld * %ld) = %ldB\n", N, sizeof(kdtree_node), kdtree_size);

  {
    // memory allocation
    // LiDAR points (x, y, z), input of radius search
    trans_cloud_points_x = (float *)malloc(points_size);
    trans_cloud_points_y = (float *)malloc(points_size);
    trans_cloud_points_z = (float *)malloc(points_size);
    // Map points (x, y, z), input of radius search
    map_points_x = (float *)malloc(map_size);
    map_points_y = (float *)malloc(map_size);
    map_points_z = (float *)malloc(map_size);
    // node indexes, output of kdtree construction
    node_indexes = (int *)malloc(sizeof(int) * N);
    // candidates and distances, output of radius search
    neighbor_candidate_indexes = (int *)malloc(neighbor_candidates_size);
    neighbor_candidate_dists = (float *)malloc(neighbor_candidates_size);
  }

  float x_min, x_max, y_min, y_max, z_min, z_max;
  float top_right;
  float bottom_left;
  int tr_index, bl_index;
  x_min = y_min = z_min = bottom_left = 10000000.0;
  x_max = y_max = z_max = top_right = -10000000.0;

  point rep_source_point, rep_neighbor_point;
  rep_source_point.x = p_cloud_source->points[0].x;
  rep_source_point.y = p_cloud_source->points[0].y;
  rep_source_point.z = p_cloud_source->points[0].z;
  float rep_dist = 100000.0;
  float temp_dist;
  float temp_tr, temp_bl;
  int rep_neighbor_index;

  struct point datapoints[N]; // kdtreeの構築に使うpoint構造体の配列
  for (int i = 0; i < N; i++) {
    // datapointsおよびmap_pointsにpclから読み込んだ値を格納
    datapoints[i].x = map_points_x[i] = p_cloud->points[i].x;
    datapoints[i].y = map_points_y[i] = p_cloud->points[i].y;
    datapoints[i].z = map_points_z[i] = p_cloud->points[i].z;
    datapoints[i].id = i;
    x_min = std::min(x_min, datapoints[i].x);
    y_min = std::min(y_min, datapoints[i].y);
    z_min = std::min(z_min, datapoints[i].z);

    x_max = std::max(x_max, datapoints[i].x);
    y_max = std::max(y_max, datapoints[i].y);
    z_max = std::max(z_max, datapoints[i].z);
    temp_dist = std::sqrt(std::pow(rep_source_point.x - datapoints[i].x, 2.0) + std::pow(rep_source_point.y - datapoints[i].y, 2.0) + std::pow(rep_source_point.z - datapoints[i].z, 2.0));
    if (temp_dist <= rep_dist) {
      rep_dist = temp_dist;
      rep_neighbor_index = i;
      rep_neighbor_point = datapoints[i];
    }

    temp_tr = temp_bl = datapoints[i].x + datapoints[i].y;
    if (temp_tr > top_right) {
      top_right = temp_tr;
      tr_index = i;
    }

    if (temp_bl < bottom_left) {
      bottom_left = temp_bl;
      bl_index = i;
    }

    // 入力点群として，pclで読み込んだ点群に乱数を加えたものを用いる
    /*
      trans_cloud_points_x[i] = p_cloud->points[i].x + (float) rand()/RAND_MAX;
      trans_cloud_points_y[i] = p_cloud->points[i].y + (float) rand()/RAND_MAX;
      trans_cloud_points_z[i] = p_cloud->points[i].z + (float) rand()/RAND_MAX;
    */
  }
  //printf("map (%f, %f, %f) to (%f, %f, %f)\n", x_min, y_min, z_min, x_max, y_max, z_max);
  printf("map (%f, %f, %f) to (%f, %f, %f)\n", datapoints[bl_index].x, datapoints[bl_index].y, datapoints[bl_index].z, datapoints[tr_index].x, datapoints[tr_index].y, datapoints[tr_index].z);

  x_min = y_min = z_min = 10000000.0;
  x_max = y_max = z_max = -10000000.0;
  for (int i = 0; i < N_source; i++) {
    trans_cloud_points_x[i] = p_cloud_source->points[i].x;
    trans_cloud_points_y[i] = p_cloud_source->points[i].y;
    trans_cloud_points_z[i] = p_cloud_source->points[i].z;
    x_min = std::min(x_min, p_cloud_source->points[i].x);
    y_min = std::min(y_min, p_cloud_source->points[i].y);
    z_min = std::min(z_min, p_cloud_source->points[i].z);

    x_max = std::max(x_max, p_cloud_source->points[i].x);
    y_max = std::max(y_max, p_cloud_source->points[i].y);
    z_max = std::max(z_max, p_cloud_source->points[i].z);
    /*
      trans_cloud_points_x[i] = p_cloud->points[i].x + (float) rand()/RAND_MAX *0.01;
      trans_cloud_points_y[i] = p_cloud->points[i].y + (float) rand()/RAND_MAX * 0.01;
      trans_cloud_points_z[i] = p_cloud->points[i].z + (float) rand()/RAND_MAX * 0.01;
    */
  }
  printf("source (%f, %f, %f) to (%f, %f, %f)\n", x_min, y_min, z_min, x_max, y_max, z_max);
  printf("index: %d, (%f, %f, %f) & (%f, %f, %f), dist = %f\n", rep_neighbor_index, rep_source_point.x, rep_source_point.y, rep_source_point.z, rep_neighbor_point.x, rep_neighbor_point.y, rep_neighbor_point.z, rep_dist);

  for (int i = 0; i < N * limit; i++) {
    neighbor_candidate_indexes[i] = 0;
    neighbor_candidate_dists[i] = 0.0;
  }

  // setup OpenCL
  if (OCL) {
    char msg[BUFSIZ];
    int ret = setup_ocl((cl_uint)platform, (cl_uint)device, msg);
    printf("Use OpenCL radius search\n");
    printf("Setting up OpenCL device %s\n", msg);
    if (ret) {
      printf("Failed to set up OpenCL device. quit\n");
      return 1;
    }
  } else {
    root_node = (kdtree_node *)malloc(kdtree_size);
    printf("Use C radius search\n");
  }

  // kdtreeの構築
  printf("constructing kd tree...\n");
  unsigned alloc_pointer = 0;
  kdtree(root_node, &alloc_pointer, datapoints, 0, N - 1, 0, node_indexes);
  // printf("MEMOFFSET: %p\n", root_node);
  // for (int i = 0; i < N; i++) {
  //   root_node[i];
  //   for (int j = 0; j < sizeof(kdtree_node); j++) {
  //     uint8_t c = ((char *)(&(root_node[i])))[j];
  //     printf("%02x ", c);
  //   }
  //   putchar('\n');
  // }
  printf("done.\n");

  printf("lidar: \n");
  for (int i = 0; i < 10; i++) {
    printf("%d, (%lf, %lf, %lf)\n", i, trans_cloud_points_x[i], trans_cloud_points_y[i], trans_cloud_points_z[i]);
  }
  printf("map: \n");
  for (int i = 0; i < 10; i++) {
    printf("%d, (%lf, %lf, %lf)\n", i, map_points_x[i], map_points_y[i], map_points_z[i]);
  }

  // kdtreeの確認
  kdtree_node *temp = root_node;
  printf("tree: \n");
  for (int i = 0; i < 10; i++) {
    if (temp != NULL) {
      printf("%d, %d, %f, (%f, %f, %f, %d), %d, %d\n", temp->depth, temp->axis, temp->axis_val, temp->location.x, temp->location.y, temp->location.z, temp->location.id, temp->left_index, temp->right_index);
      temp = temp->child1;
    }
  }

  /*
    temp = root_node;
    for (int i = 0; i < 100; i++){
    printf("axis: %d, axis_val: %f, left: (%f, %f, %f, %d), right: (%f, %f, %f, %d)\n", temp->axis, temp->axis_val, temp->child1->location.x, temp->child1->location.y, temp->child1->location.z, temp->child1->location.id, temp->child2->location.x, temp->child2->location.y, temp->child2->location.z, temp->child2->location.id);
    printf("child1 (%d to %d), child2 (%d to %d)\n", temp->child1->left_index, temp->child1->right_index, temp->child2->left_index, temp->child2->right_index);
    temp = temp->child1;
    }
  */

  // timer
  clock_t t0 = clock();
  // Writing Map & LiDAR points to device
  if (OCL) {
    printf("copy host to device\n");
    clEnqueueWriteBuffer(Queue, d_points_x, CL_TRUE, 0, points_size, trans_cloud_points_x, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_points_y, CL_TRUE, 0, points_size, trans_cloud_points_y, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_points_z, CL_TRUE, 0, points_size, trans_cloud_points_z, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_map_points_x, CL_TRUE, 0, map_size, map_points_x, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_map_points_y, CL_TRUE, 0, map_size, map_points_y, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_map_points_z, CL_TRUE, 0, map_size, map_points_z, 0, NULL, NULL);
    clEnqueueWriteBuffer(Queue, d_node_indexes, CL_TRUE, 0, points_size, node_indexes, 0, NULL, NULL);
    printf("end copy host to device\n");
  }

  // call radius search
  kdtree_calc(OCL);

  // Reading results from device
  if (OCL) {
    clEnqueueReadBuffer(Queue, d_neighbor_candidates, CL_TRUE, 0, neighbor_candidates_size, neighbor_candidate_indexes, 0, NULL, NULL);
    clEnqueueReadBuffer(Queue, d_neighbor_candidate_dists, CL_TRUE, 0, neighbor_candidates_size, neighbor_candidate_dists, 0, NULL, NULL);
  }
  // timer
  clock_t t1 = clock();
  double cpu = (double)(t1 - t0) / CLOCKS_PER_SEC;

  printf("elapsed time cpu[sec]=%.3f\n", cpu);

  for (int i = 0; i < 3000; i++) {
    if (neighbor_candidate_indexes[i] != 0) {
      int source_point_index = i / limit;
      int map_point_index = neighbor_candidate_indexes[i];
      printf("%d-th source_point (%f, %f, %f), neighbor map point (%f, %f, %f)\n", source_point_index, trans_cloud_points_x[source_point_index], trans_cloud_points_y[source_point_index], trans_cloud_points_z[source_point_index], map_points_x[map_point_index], map_points_y[map_point_index], map_points_z[map_point_index]);

      //printf("neighbor[%d] = %d (dist = %f)\n", i, neighbor_candidate_indexes[i], neighbor_candidate_dists[i]);
    }
  }

 CLEANUP:
  // release allocated CL memory
  if (OCL) {
    clReleaseMemObject(d_points_x);
    clReleaseMemObject(d_points_y);
    clReleaseMemObject(d_points_z);
    clReleaseMemObject(d_map_points_x);
    clReleaseMemObject(d_map_points_y);
    clReleaseMemObject(d_map_points_z);
    clReleaseMemObject(d_node_indexes);
    clReleaseMemObject(d_neighbor_candidates);
    clReleaseMemObject(d_neighbor_candidate_dists);
    clReleaseKernel(k_kdtree);
    clReleaseCommandQueue(Queue);
    clSVMFree(context, root_node);
  }

  // release allocated memory
  free(trans_cloud_points_x);
  free(trans_cloud_points_y);
  free(trans_cloud_points_z);
  free(node_indexes);
  free(neighbor_candidate_indexes);
  free(neighbor_candidate_dists);
  if (!OCL) {
    free(root_node);
  }
  return 0;
}

// setup OpenCL
static int setup_ocl(cl_uint platform, cl_uint device, char * msg) {
  const char * kernel_file = "kernel_radius_search.cl";
  const char * kernel_name = "kdtreeNDT";
  cl_platform_id platform_id[MAX_PLATFORMS];
  cl_device_id device_id[MAX_DEVICES];

  size_t source_size, ret_size, points_size, map_points_size, neighbor_candidates_size, kdtree_size;
  cl_uint num_platforms, num_devices;
  cl_int ret;

  // platform
  clGetPlatformIDs(MAX_PLATFORMS, platform_id, &num_platforms);
  if (platform >= num_platforms) {
    sprintf(msg, "[ERROR] platform = %d (limit = %d)", platform, num_platforms - 1);
    return 1;
  }

  // device
  clGetDeviceIDs(platform_id[platform], CL_DEVICE_TYPE_ALL, MAX_DEVICES, device_id, &num_devices);
  if (device >= num_devices) {
    sprintf(msg, "[ERROR] device = %d (limit = %d)", device, num_devices - 1);
    return 1;
  }

  // device name (optional information)
  {
    char str[BUFSIZ];
    clGetDeviceInfo(device_id[device], CL_DEVICE_NAME, sizeof(str), str, &ret_size);
    sprintf(msg, "[INFO] %s (platform = %d, device = %d)", str, platform, device);
  }

  // context
  context = clCreateContext(NULL, 1, &device_id[device], NULL, NULL, &ret);

  // command queue
  Queue = clCreateCommandQueue(context, device_id[device], 0, &ret);

  // source
  {
    FILE * fp;
    if ((fp = fopen(kernel_file, "r")) == NULL) {
      sprintf(msg, "[ERROR] could not open %s", kernel_file);
      return 1;
    }
    // alloc
    char * source_str = (char *)malloc(MAX_SOURCE_SIZE * sizeof(char));
    source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
    // create program
    program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &ret);
    if (ret != CL_SUCCESS) {
      sprintf(msg, "[ERROR] clCreateProgramWithSource() error %d", ret);
      return 1;
    }
    fclose(fp);
    free(source_str);
  }

  // build program
  if (clBuildProgram(program, 1, &device_id[device], NULL, NULL, NULL) != CL_SUCCESS) {
    sprintf(msg, "[ERROR] clBuildProgram() error");
    size_t logSize;
    clGetProgramBuildInfo(program, device_id[device], CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
    char * buildLog = (char *)malloc((logSize + 1));
    clGetProgramBuildInfo(program, device_id[device], CL_PROGRAM_BUILD_LOG, logSize, buildLog, NULL);
    printf("%s", buildLog);
    free(buildLog);
    return 1;
  }

  // kernel
  k_kdtree = clCreateKernel(program, kernel_name, &ret);
  if (ret != CL_SUCCESS) {
    sprintf(msg, "[ERROR] clCreateKernel() error");
    return 1;
  }

  // memory object
  points_size = N_source * sizeof(float);
  map_points_size = N * sizeof(float);
  neighbor_candidates_size = 7 * N * sizeof(float);
  kdtree_size = N * sizeof(kdtree_node);

  d_points_x = clCreateBuffer(context, CL_MEM_READ_WRITE, points_size, NULL, &ret);
  d_points_y = clCreateBuffer(context, CL_MEM_READ_WRITE, points_size, NULL, &ret);
  d_points_z = clCreateBuffer(context, CL_MEM_READ_WRITE, points_size, NULL, &ret);
  d_map_points_x = clCreateBuffer(context, CL_MEM_READ_WRITE, map_points_size, NULL, &ret);
  d_map_points_y = clCreateBuffer(context, CL_MEM_READ_WRITE, map_points_size, NULL, &ret);
  d_map_points_z = clCreateBuffer(context, CL_MEM_READ_WRITE, map_points_size, NULL, &ret);
  d_node_indexes = clCreateBuffer(context, CL_MEM_READ_WRITE, map_points_size, NULL, &ret);
  d_neighbor_candidates = clCreateBuffer(context, CL_MEM_READ_WRITE, neighbor_candidates_size, NULL, &ret);
  d_neighbor_candidate_dists = clCreateBuffer(context, CL_MEM_READ_WRITE, neighbor_candidates_size, NULL, &ret);
  // allocation of shared virtual memory
  root_node = static_cast<kdtree_node*>(clSVMAlloc(context, CL_MEM_READ_WRITE, kdtree_size, 0));

  // release
  // clReleaseProgram(program);
  // clReleaseContext(context);
  return 0;
}

// entry point
static void kdtree_calc(int isOCL) {
  if (isOCL == 1) {
    printf("preparing kernel arguments (opencl)\n");
    size_t global_item_size, local_item_size;
    cl_int ret;

    // set arguments
    clSetKernelArg(k_kdtree, 0, sizeof(cl_mem), (void *)&d_points_x);
    clSetKernelArg(k_kdtree, 1, sizeof(cl_mem), (void *)&d_points_y);
    clSetKernelArg(k_kdtree, 2, sizeof(cl_mem), (void *)&d_points_z);
    clSetKernelArg(k_kdtree, 3, sizeof(cl_mem), (void *)&d_map_points_x);
    clSetKernelArg(k_kdtree, 4, sizeof(cl_mem), (void *)&d_map_points_y);
    clSetKernelArg(k_kdtree, 5, sizeof(cl_mem), (void *)&d_map_points_z);
    clSetKernelArg(k_kdtree, 6, sizeof(cl_mem), (void *)&d_node_indexes);
    clSetKernelArgSVMPointer(k_kdtree, 7, root_node);
    clSetKernelArg(k_kdtree, 8, sizeof(int), (void *)&N);
    clSetKernelArg(k_kdtree, 9, sizeof(int), (void *)&limit);
    clSetKernelArg(k_kdtree, 10, sizeof(cl_mem), (void *)&d_neighbor_candidates);
    clSetKernelArg(k_kdtree, 11, sizeof(cl_mem), (void *)&d_neighbor_candidate_dists);

    // work item
    local_item_size = 256;
    global_item_size = ((N + local_item_size - 1) / local_item_size) * local_item_size;

    // kicking the kernel
    printf("run radius_search (OpenCL)\n");
    ret = clEnqueueNDRangeKernel(Queue, k_kdtree, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
    if (CL_SUCCESS != ret) {
      fprintf(stderr, "[ERROR] error code %d\n", ret);
    }
  } else {
    printf("run radius_search (C)\n");
    radiusSearchC(trans_cloud_points_x, trans_cloud_points_y, trans_cloud_points_z,
                  map_points_x, map_points_y, map_points_z, node_indexes,
                  root_node, N, limit, radius, neighbor_candidate_indexes, neighbor_candidate_dists);
  }
  return;
}
