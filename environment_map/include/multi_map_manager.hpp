#pragma once

#include <memory>
#include <random>
#include <vector>
#include <unordered_map>
#include <Eigen/Eigen>

#include "map.hpp"
#include "environment_map_msgs/msg/chunk_data.hpp"
#include "environment_map_msgs/msg/chunk_stamps.hpp"
#include "environment_map_msgs/msg/idx_list.hpp"

// A map chunk, the elementary exchange unit between robots
struct MapChunk {
  uint32_t idx_;  // Start from 1
  std::vector<uint32_t> voxel_adrs_;
  std::vector<uint8_t> voxel_occ_;

  // Whether the chunk needs to be queried for occupancy information
  bool need_query_;

  // Whether the chunk has already been inserted into the map or not
  bool empty_;
};

struct ChunksData {
  std::vector<MapChunk> chunks_;
  // An idx_list is a flat list of inclusive ranges, [start0, end0, start1, end1, ...].
  // For example, [1,5, 8,10] means the chunks 1–5 and 8–10 are present.
  // Chunks are numbered starting from 1
  std::vector<int> idx_list_;
};

struct ChunksBox {
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;
  bool valid_;
};

class MultiMapManager {
public:
  MultiMapManager();
  ~MultiMapManager();
  void setMap(std::shared_ptr<Map> map);
  void init(int robot_id, int map_num, int chunk_size);

  void updateMapChunk(const std::vector<uint32_t>& adrs);
  void sendStamp(double time, environment_map_msgs::msg::ChunkStamps& msg);
  void receiveStamp(const environment_map_msgs::msg::ChunkStamps& msg_in,
                    std::vector<environment_map_msgs::msg::ChunkData>& msgs_out);
  void receiveChunk(const environment_map_msgs::msg::ChunkData& msg);
  void processChunkBuffer(double inflation_radius);
  void getChunkBoxes(std::vector<Eigen::Vector2d>& mins,
                     std::vector<Eigen::Vector2d>& maxs,
                     bool reset = true);

private:
  void sendChunks(const int& chunk_robot_id,
                  const int& to_robot_id,
                  const std::vector<int>& idx_list,
                  std::vector<environment_map_msgs::msg::ChunkData>& msgs);
  void getOccOfChunk(const std::vector<uint32_t>& adrs, std::vector<uint8_t>& occs);
  void insertChunkToMap(const MapChunk& chunk, const int& robot_id, double inflation_radius);

  // Operations on the chunk idx list
  void findMissedChunkIds(const std::vector<int>& self_idx_list,
                          const std::vector<int>& other_idx_list,
                          std::vector<int>& miss_ids);
  bool findIntersect(const int& min1, const int& max1, const int& min2,
                     const int max2, int& minr, int& maxr);
  void mergeChunkIds(const std::vector<int>& list1, const std::vector<int>& list2,
                     std::vector<int>& output);

  // ID of robot, starting from 0
  int robot_id_;
  // Total number of robots/maps (one map for each robot)
  int map_num_;
  // Size of each map chunk in voxels
  int chunk_size_;

  std::shared_ptr<Map> map_;

  std::vector<ChunksData> multi_map_chunks_;               // Main map data
  std::vector<uint32_t> adr_buffer_;                       // Buffer for chunks of this map
  std::vector<std::vector<environment_map_msgs::msg::ChunkData>> chunk_buffer_;  // Buffer for chunks of external map
  std::vector<std::unordered_map<int, char>> buffer_map_;  // Hash map to avoid repeated insertion of chunk msg
  std::vector<double> last_chunk_stamp_time_;

  // Bounding box of map chunks of swarm
  std::vector<ChunksBox> chunk_boxes_;

  // std::vector<Eigen::Vector2i> tmp_ids_;
  // Eigen::Vector2d robot_pos_;
};