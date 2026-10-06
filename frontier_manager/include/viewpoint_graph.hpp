#pragma once

#include <Eigen/Eigen>

#include "map.hpp"
#include "raycast.hpp"
#include "astar.hpp"

// Basic noded type containing only general artributes required by graph search
class BaseNode {
public:
  typedef std::shared_ptr<BaseNode> Ptr;
  BaseNode() {
    g_value_ = 1000000;
    closed_ = false;
  }
  ~BaseNode() {
  }

  virtual void print() {
    std::cout << "Base node" << std::endl;
  }

  int id_;
  bool closed_;
  double g_value_;
};

class ViewNode : public BaseNode {
public:
  typedef std::shared_ptr<ViewNode> Ptr;
  ViewNode(const Eigen::Vector2d& p, const double& y);
  ViewNode() {
  }
  ~ViewNode() {
  }

  virtual void print() {
    std::cout << "View node" << yaw_ << std::endl;
  }

  void printNeighbors() {
    for (auto v : neighbors_)
      v->print();
  }

  double costTo(const ViewNode::Ptr& node);
  static double computeCost(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const double& y1, const double& y2,
                            const Eigen::Vector2d& v1, std::vector<Eigen::Vector2d>& path);
  // Coarse to fine path searching
  static double searchPath(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::vector<Eigen::Vector2d>& path);

  // Data
  std::vector<ViewNode::Ptr> neighbors_;
  ViewNode::Ptr parent_;
  Eigen::Vector2d pos_, vel_;
  double yaw_;

  // Parameters shared among nodes
  static double vm_, yd_, w_dir_;
  static std::shared_ptr<Astar> astar_;
  static std::shared_ptr<RayCaster> caster_;
  static std::shared_ptr<Map> map_;
};