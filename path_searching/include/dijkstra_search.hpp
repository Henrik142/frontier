#pragma once

#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <list>
#include <memory>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Eigen>

template <typename NodeT>
class GraphSearch {
public:
  GraphSearch() {
    node_num_ = 0;
    edge_num_ = 0;
  }
  ~GraphSearch() {
  }

  void print();
  void addNode(const std::shared_ptr<NodeT>& node);
  void addEdge(const int& from, const int& to);
  void DijkstraSearch(const int& start, const int& goal, std::vector<std::shared_ptr<NodeT>>& path);
  void FullDijkstraSearch(const int& start, std::vector<std::shared_ptr<NodeT>>& path);

private:
  std::vector<std::shared_ptr<NodeT>> nodes_;
  int node_num_;
  int edge_num_;
};

template <typename NodeT>
class NodeCompare {
public:
  bool operator()(const std::shared_ptr<NodeT>& node1, const std::shared_ptr<NodeT>& node2) {
    return node1->g_value_ > node2->g_value_;
  }
};

template <typename NodeT>
void GraphSearch<NodeT>::print() {
  for (auto v : nodes_) {
    v->print();
    v->printNeighbors();
  }
}

template <typename NodeT>
void GraphSearch<NodeT>::addNode(const std::shared_ptr<NodeT>& node) {
  nodes_.push_back(node);
  nodes_.back()->id_ = node_num_++;
}

template <typename NodeT>
void GraphSearch<NodeT>::addEdge(const int& from, const int& to) {
  nodes_[from]->neighbors_.push_back(nodes_[to]);
  ++edge_num_;
}

template <typename NodeT>
void GraphSearch<NodeT>::DijkstraSearch(const int& start, const int& goal,
                                        std::vector<std::shared_ptr<NodeT>>& path) {
  // Basic structure used by Dijkstra
  std::priority_queue<std::shared_ptr<NodeT>,
                      std::vector<std::shared_ptr<NodeT>>,
                      NodeCompare<NodeT>> open_set;

  std::shared_ptr<NodeT> start_v = nodes_[start];
  std::shared_ptr<NodeT> end_v = nodes_[goal];
  start_v->g_value_ = 0.0;
  open_set.push(start_v);

  while (!open_set.empty()) {
    auto vc = open_set.top();
    open_set.pop();
    vc->closed_ = true;
    // close_set[vc->id_] = 1;

    // Check if reach target
    if (vc == end_v) {
      std::shared_ptr<NodeT> vit = vc;
      while (vit != nullptr) {
        path.push_back(vit);
        vit = vit->parent_;
      }
      std::reverse(path.begin(), path.end());
      return;
    }
    for (auto vb : vc->neighbors_) {
      // Check if in close set
      if (vb->closed_) continue;

      // Add new node or updated node in open set
      double g_tmp = vc->g_value_ + vc->costTo(vb);
      if (g_tmp < vb->g_value_) {
        vb->g_value_ = g_tmp;
        vb->parent_ = vc;
        open_set.push(vb);
      }
    }
  }
}

template <typename NodeT>
void GraphSearch<NodeT>::FullDijkstraSearch(const int& start,
                                        std::vector<std::shared_ptr<NodeT>>& path) {
  // Modified Dijkstra search that finds the shortest path to all reachable nodes
  // returns the shortest path to among all paths to reachable leaf nodes
  
  // Basic structure used by Dijkstra
  std::priority_queue<std::shared_ptr<NodeT>,
                      std::vector<std::shared_ptr<NodeT>>,
                      NodeCompare<NodeT>> open_set;

  std::shared_ptr<NodeT> start_v = nodes_[start];
  start_v->g_value_ = 0.0;
  open_set.push(start_v);

  while (!open_set.empty()) {
    auto vc = open_set.top();
    open_set.pop();
    vc->closed_ = true;

    for (auto vb : vc->neighbors_) {
      // Check if in close set
      if (vb->closed_) continue;

      // Add new node or updated node in open set
      double g_tmp = vc->g_value_ + vc->costTo(vb);
      if (g_tmp < vb->g_value_) {
        vb->g_value_ = g_tmp;
        vb->parent_ = vc;
        open_set.push(vb);
      }
    }
  }

  // Pick the reached leaf node (no children) with the lowest g_value_
  std::shared_ptr<NodeT> vlast = nullptr;
  for (const auto& v : nodes_) {
    if (!v->neighbors_.empty() || !v->closed_) continue;
    if (vlast == nullptr || v->g_value_ < vlast->g_value_) vlast = v;
  }
  if (vlast == nullptr) return;

  std::shared_ptr<NodeT> vit = vlast;
  while (vit != nullptr) {
    path.push_back(vit);
    vit = vit->parent_;
  }
  std::reverse(path.begin(), path.end());
  return;
}