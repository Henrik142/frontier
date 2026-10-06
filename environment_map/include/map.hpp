#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <vector>
#include <queue>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RayCaster;

struct MapParam;
struct MapData;

class Map {
public:
  Map();
  ~Map();

  // External parameters needed for initialization
  struct InitMapParams {
    double xsize;
    double ysize;
    double resolution;
    double p_hit;
    double p_miss;
    double p_min;
    double p_max;
    double p_occ;
    double max_ray_length;
    double obstacles_inflation_radius;
    double unknown_inflation_radius;
    double esdf_inflation;
    double esdf_default_dist;
    bool esdf_optimistic;
    bool esdf_signed_dist;
  };

  enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED };

  void initMap(const InitMapParams& map_params);

  double getResolution() const;
  int getVoxelNum() const;
  int getVoxelNumX() const;
  int getVoxelNumY() const;
  double getOriginX() const;
  double getOriginY() const;
  double getSizeX() const;
  double getSizeY() const;
  void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) const;
  void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) const;
  int toAddress(const Eigen::Vector2i& id) const;
  int toAddress(const int& x, const int& y) const;
  bool isInMap(const Eigen::Vector2d& pos) const;
  bool isInMap(const Eigen::Vector2i& idx) const;
  int getOccupancy(const Eigen::Vector2d& pos) const;
  int getOccupancy(const Eigen::Vector2i& id) const;
  int getInflatedOccupancy(const Eigen::Vector2d& pos) const;
  int getInflatedOccupancy(const Eigen::Vector2i& id) const;
  double getDistance(const Eigen::Vector2d& pos) const;
  double getDistance(const Eigen::Vector2i& id) const;
  double getDistWithGrad(const Eigen::Vector2d& pos, Eigen::Vector2d& grad) const;

  void setOccupancy(const Eigen::Vector2d& pos, int occ);
  void setOccupancy(const Eigen::Vector2i& id, int occ);

  void getMapBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax) const;
  void getUpdatedBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, bool reset = false);
  void inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2i& inflation_size) const;
  void inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2d& inflation_size) const;
  void fitToBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2i& fitMin,
                const Eigen::Vector2i& fitMax) const;
  void fitToMap(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax) const;

  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const int& point_num,
                       const Eigen::Vector2d& camera_pos);

  void inflateObstacles();

  void updateESDF2d();
  
  void manualUpdate(const std::vector<Eigen::Vector2i> updated_cells,
                    const std::vector<int> updated_occupancies);

private:
  void setOccupancyValue(const Eigen::Vector2i& id, double occ);
  Eigen::Vector2d closestPointInMap(const Eigen::Vector2d& pt, const Eigen::Vector2d& camera_pt);
  void setCacheOccupancy(const int& adr, const int& occ);
  void inflatePoint(const Eigen::Vector2i& pt, const double inflation_radius,
                    std::vector<Eigen::Vector2i>& pts);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  std::unique_ptr<MapParam> mp_;
  std::unique_ptr<MapData> md_;
  std::unique_ptr<RayCaster> caster_;
};

struct MapParam {
  // map properties
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_;
  Eigen::Vector2i map_box_min_, map_box_max_; // bounding box of the map in voxel indices

  // number of voxels in x and y directions
  Eigen::Vector2i map_voxel_num_;

  // map resolution (i.e. cell size) and its inverse
  double resolution_, resolution_inv_;

  // distance for inflating obstacles and unknown cells
  double obstacles_inflation_radius_, unknown_inflation_radius_;

  // Controls whether unknown space is treated as free or occupied when calculating
  // distances for the ESDF (Euclidean Signed Distance Field)
  bool optimistic_;

  // Controls whether negative distances are computed or not
  bool signed_dist_;

  // default distance value for ESDF initialization
  double default_dist_;

  // inflation size (in meters) for ESDF update
  double esdf_inflation_;

  // map fusion parameters
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;  // logit
  double max_ray_length_;  // maximum length of a ray for raycasting-based fusion
  double unknown_flag_;  // threshold for unknown occupancy
};

struct MapData {
  // main map data, occupancy of each voxel
  std::vector<double> occupancy_buffer_;
  std::vector<int> occupancy_buffer_inflate_;

  // distance buffers for ESDF computation
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;

  // bounding box of the updated area in the map in voxel indices
  Eigen::Vector2i update_box_min_, update_box_max_;

  // data for updating
  std::vector<short> count_hit_, count_miss_;
  std::vector<uint32_t> flag_rayend_, flag_visited_;
  uint32_t raycast_num_;
  std::queue<int> cache_voxel_;
  bool reset_updated_box_; // stores wether the update box needs to be reset on the next call of inputPointCloud
};