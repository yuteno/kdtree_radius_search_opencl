#include "radius_search.h"
#include <stdlib.h>
#include <stdio.h>

// util function
static int sign(float a) {
  return (a > 0) - (a < 0);
}

static int comparex(const void * a, const void * b) {
  return sign(((struct point *)a)->x - ((struct point *)b)->x);
}

static int comparey(const void * a, const void * b) {
  return sign(((struct point *)a)->y - ((struct point *)b)->y);
}

static int comparez(const void * a, const void * b) {
  return sign(((struct point *)a)->z - ((struct point *)b)->z);
}

static int min(int a, int b) {
  return (a > b) ? b : a;
}

static int max(int a, int b) {
  return (a < b) ? b : a;
}
const int MAX_DEPTH = 9;

kdtree_node * kdtree(kdtree_node * root_node,
                     unsigned * alloc_pointer,
                     struct point * pointlist,
                     int left, int right, unsigned depth,
                     int * node_indexes) {
  if (right < 0) {
    // not allocate
    return NULL;
  }
  // allocate
  kdtree_node * new_node = &root_node[*alloc_pointer];
  *alloc_pointer = *alloc_pointer + 1; // count up

  // index list included in this node
  new_node->left_index = left;
  new_node->right_index = left + right + 1;
  // tree's depth
  new_node->depth = depth;
  int median = right / 2;

  if (right == 0 || depth > MAX_DEPTH) {
    // new_node is leaf
    new_node->location = pointlist[median];
    // no child node
    new_node->child1 = NULL;
    new_node->child2 = NULL;

    qsort(pointlist, right + 1, sizeof(struct point), comparex);
    new_node->leftmost = pointlist[0].x;
    new_node->rightmost = pointlist[right].x;

    qsort(pointlist, right + 1, sizeof(struct point), comparey);
    new_node->downmost = pointlist[0].y;
    new_node->upmost = pointlist[right].y;

    qsort(pointlist, right + 1, sizeof(struct point), comparez);
    new_node->zlowmost = pointlist[0].z;
    new_node->zupmost = pointlist[right].z;

    for (int i = 0; i < right+1; i++) {
      node_indexes[new_node->left_index+i] = pointlist[i].id;
    }

    return new_node;
  }

  // change the dividing dirction
  new_node->axis = depth % 3;

  // sorting by space (1 direction) for binary division
  if (new_node->axis == XAxis) {
    // ascending sorting by point.x
    qsort(pointlist, right + 1, sizeof(struct point), comparex);
    new_node->leftmost = pointlist[0].x;
    new_node->rightmost = pointlist[right].x;
  } else if (new_node->axis == YAxis) {
    // by point.y
    qsort(pointlist, right + 1, sizeof(struct point), comparey);
    new_node->downmost = pointlist[0].y;
    new_node->upmost = pointlist[right].y;
  } else if (new_node->axis == ZAxis) {
    // by point.z
    qsort(pointlist, right + 1, sizeof(struct point), comparez);
    new_node->zlowmost = pointlist[0].z;
    new_node->zupmost = pointlist[right].z;
  }


  node_indexes[new_node->left_index+median] = pointlist[median].id;
  //printf("node->left_index+median: %d\n", new_node->left_index+median);
  new_node->location = pointlist[median];
  new_node->child2 = kdtree(root_node, alloc_pointer,
                            pointlist + median + 1,
                            /*left  =*/ left + median + 1,
                            /*right =*/ right - (median + 1),
                            /*depth =*/ depth + 1, node_indexes);

  new_node->child1 = kdtree(root_node, alloc_pointer,
                            pointlist,
                            /*left  =*/ left,
                            /*right =*/ median - 1,
                            /*depth =*/ depth + 1, node_indexes);

  // calculate bounding box
  switch (new_node->axis) {
  case XAxis:
    new_node->axis_val = new_node->location.x;
    if (new_node->child2 != NULL && new_node->child1 != NULL) {
      new_node->upmost = max(new_node->child2->upmost,
                             new_node->child1->upmost);
      new_node->downmost = min(new_node->child2->downmost,
                               new_node->child1->downmost);
      new_node->zupmost = max(new_node->child2->zupmost,
                              new_node->child1->zupmost);
      new_node->zlowmost = min(new_node->child2->zlowmost,
                               new_node->child1->zlowmost);
    } else if (new_node->child2 != NULL) {
      new_node->upmost = new_node->child2->upmost;
      new_node->downmost = new_node->child2->downmost;
      new_node->zupmost = new_node->child2->zupmost;
      new_node->zlowmost = new_node->child2->zlowmost;
    } else if (new_node->child1 != NULL) {
      new_node->upmost = new_node->child1->upmost;
      new_node->downmost = new_node->child1->downmost;
      new_node->zupmost = new_node->child1->zupmost;
      new_node->zlowmost = new_node->child1->zlowmost;
    } else {
      new_node->upmost = new_node->location.y;
      new_node->downmost = new_node->location.y;
      new_node->zupmost = new_node->location.z;
      new_node->zlowmost =  new_node->location.z;
    }
    break;
  case YAxis:
    new_node->axis_val = new_node->location.y;
    if (new_node->child2 != NULL && new_node->child1 != NULL) {
      new_node->rightmost = max(new_node->child2->rightmost,
                                new_node->child1->rightmost);
      new_node->leftmost = min(new_node->child2->leftmost,
                               new_node->child1->leftmost);
      new_node->zupmost = max(new_node->child2->zupmost,
                              new_node->child1->zupmost);
      new_node->zlowmost = min(new_node->child2->zlowmost,
                               new_node->child1->zlowmost);
    } else if (new_node->child2 != NULL) {
      new_node->rightmost = new_node->child2->rightmost;
      new_node->leftmost = new_node->child2->leftmost;
      new_node->zupmost = new_node->child2->zupmost;
      new_node->zlowmost = new_node->child2->zlowmost;
    } else if (new_node->child1 != NULL) {
      new_node->rightmost = new_node->child1->rightmost;
      new_node->leftmost = new_node->child1->leftmost;
      new_node->zupmost = new_node->child1->zupmost;
      new_node->zlowmost = new_node->child1->zlowmost;
    } else {
      new_node->rightmost = new_node->location.x;
      new_node->leftmost = new_node->location.x;
      new_node->zupmost = new_node->location.z;
      new_node->zlowmost = new_node->location.z;
    }
    break;
  case ZAxis:
  default:
    new_node->axis_val = new_node->location.z;
    if (new_node->child2 != NULL && new_node->child1 != NULL) {
      new_node->rightmost = max(new_node->child2->rightmost,
                                new_node->child1->rightmost);
      new_node->leftmost = min(new_node->child2->leftmost,
                               new_node->child1->leftmost);
      new_node->upmost = max(new_node->child2->upmost,
                             new_node->child1->upmost);
      new_node->downmost = min(new_node->child2->downmost,
                               new_node->child1->downmost);
    } else if (new_node->child2 != NULL) {
      new_node->rightmost = new_node->child2->rightmost;
      new_node->leftmost = new_node->child2->leftmost;
      new_node->upmost = new_node->child2->upmost;
      new_node->downmost = new_node->child2->downmost;
    } else if (new_node->child1 != NULL) {
      new_node->rightmost = new_node->child1->rightmost;
      new_node->leftmost = new_node->child1->leftmost;
      new_node->upmost = new_node->child1->upmost;
      new_node->downmost = new_node->child1->downmost;
    } else {
      new_node->rightmost = new_node->location.x;
      new_node->leftmost = new_node->location.x;
      new_node->upmost = new_node->location.y;
      new_node->downmost = new_node->location.y;
    }
    break;
  }
  return new_node;
}
