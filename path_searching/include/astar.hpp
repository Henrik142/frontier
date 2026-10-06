#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <Eigen/Eigen>
#include <chrono>
#include <cmath>

#include "matrix_hash.hpp"
#include "map.hpp"

class Node {
public:
  Eigen::Vector2i index;
  Eigen::Vector2d position;
  double g_score, f_score;
  Node* parent;

  /* -------------------- */
  Node() {
    parent = NULL;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class Astar {
public:
  Astar();
  ~Astar();

  enum {  REACH_END = 1,          // path is successfully found
          NO_PATH_TIMEOUT = 2,    // search timed out
          NO_PATH_MEMORY = 3,     // memory limit reached
          NO_PATH_INVALID = 4     // no valid path exists in the graph
  };

  struct InitAstarParams {
        double resolution;
        double lambda_heu;
        double max_search_time;
        int allocate_num;
  };

  void init(const InitAstarParams& params, const std::shared_ptr<Map>& map);
  void reset();
  int search(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt);
  void setResolution(const double& res);
  static double pathLength(const std::vector<Eigen::Vector2d>& path);

  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector2d> getVisited();
  double getEarlyTerminateCost();

  double lambda_heu_;
  double max_search_time_;

private:
  bool checkLineOfSight(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
  void backtrack(const NodePtr& end_node, const Eigen::Vector2d& end);
  void posToIndex(const Eigen::Vector2d& pt, Eigen::Vector2i& idx);
  double getDiagHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);
  double getManhHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);
  double getEuclHeu(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);

  /* Wrappers of Map class */
  double getMapResolution();
  void getOrigin(Eigen::Vector2d& origin);
  void getSize(Eigen::Vector2d& size);
  bool isInMap(const Eigen::Vector2d& pos);
  int getInflatedOccupancy(const Eigen::Vector2d& pos);

  // main data structure
  std::vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>> open_set_map_;
  std::unordered_map<Eigen::Vector2i, int, matrix_hash<Eigen::Vector2i>> close_set_map_;
  std::vector<Eigen::Vector2d> path_nodes_;
  double early_terminate_cost_;

  std::shared_ptr<Map> map_;

  // parameter
  int allocate_num_;
  double tie_breaker_;
  double resolution_, inv_resolution_;
  Eigen::Vector2d map_size_, origin_;
};