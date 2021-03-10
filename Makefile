radius_search:
	g++ -std=c++14 src/host_kdtree_radius_search.cpp src/kdtree_construction.c src/radius_search.c `pkg-config --cflags --libs pcl_io-1.10 --libs pcl_filters-1.10` -lboost_system -lOpenCL -g3 -O0

kdtree_save:
	g++ -std=c++14 src/kdtree_save.cpp src/kdtree_construction.c src/radius_search.c `pkg-config --cflags --libs pcl_io-1.10 --libs pcl_filters-1.10` -lboost_system -g3 -O0

load_kdtree:
	g++ -std=c++14 src/kdtree_binary_load.cpp src/kdtree_construction.c src/radius_search.c `pkg-config --cflags --libs pcl_io-1.10 --libs pcl_filters-1.10` -lboost_system -g3 -O0