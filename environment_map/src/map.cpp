#include "map.hpp"
#include "raycast.hpp"

Map::Map() {
}

Map::~Map() {
}

void Map::initMap(const InitMapParams& map_params) {
    mp_.reset(new MapParam);
    md_.reset(new MapData);    
    
    // Initialize map parameters
    mp_->map_origin_ = Eigen::Vector2d(-map_params.xsize/2, -map_params.ysize/2);
    mp_->map_size_ = Eigen::Vector2d(map_params.xsize, map_params.ysize);
    mp_->resolution_ = map_params.resolution;
    mp_->resolution_inv_ = 1 / mp_->resolution_;
    mp_->map_voxel_num_(0) = ceil(mp_->map_size_(0) / mp_->resolution_);
    mp_->map_voxel_num_(1) = ceil(mp_->map_size_(1) / mp_->resolution_);
    mp_->map_min_boundary_ = mp_->map_origin_;
    mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
    posToIndex(mp_->map_min_boundary_, mp_->map_box_min_);
    posToIndex(mp_->map_max_boundary_, mp_->map_box_max_);
    mp_->map_box_max_ -= Eigen::Vector2i::Ones();
    mp_->obstacles_inflation_radius_ = map_params.obstacles_inflation_radius;
    mp_->unknown_inflation_radius_ = map_params.unknown_inflation_radius;

    // Params of raycasting-based fusion
    mp_->p_hit_ = map_params.p_hit;
    mp_->p_miss_ = map_params.p_miss;
    mp_->p_min_ = map_params.p_min;
    mp_->p_max_ = map_params.p_max;
    mp_->p_occ_ = map_params.p_occ;
    mp_->max_ray_length_ = map_params.max_ray_length;

    // ESDF params
    mp_->esdf_inflation_ = map_params.esdf_inflation;
    mp_->default_dist_ = map_params.esdf_default_dist;
    mp_->optimistic_ = map_params.esdf_optimistic;
    mp_->signed_dist_ = map_params.esdf_signed_dist;

    auto logit = [](const double& x) { return log(x / (1 - x)); };
    mp_->prob_hit_log_ = logit(mp_->p_hit_);
    mp_->prob_miss_log_ = logit(mp_->p_miss_);
    mp_->clamp_min_log_ = logit(mp_->p_min_);
    mp_->clamp_max_log_ = logit(mp_->p_max_);
    mp_->min_occupancy_log_ = logit(mp_->p_occ_);
    mp_->unknown_flag_ = 0.01;

    // Initialize data buffer of map
    int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1);
    md_->occupancy_buffer_ = std::vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_); // all unknown
    md_->occupancy_buffer_inflate_ = std::vector<int>(buffer_size, Map::UNKNOWN);
    md_->distance_buffer_neg_ = std::vector<double>(buffer_size, mp_->default_dist_);
    md_->distance_buffer_ = std::vector<double>(buffer_size, mp_->default_dist_);
    md_->tmp_buffer1_ = std::vector<double>(buffer_size, 0.0);
    md_->tmp_buffer2_ = std::vector<double>(buffer_size, 0.0);
    md_->count_hit_ = std::vector<short>(buffer_size, 0);
    md_->count_miss_ = std::vector<short>(buffer_size, 0);
    md_->flag_rayend_ = std::vector<uint32_t>(buffer_size, -1);
    md_->flag_visited_ = std::vector<uint32_t>(buffer_size, -1);
    md_->raycast_num_ = 0;
    md_->reset_updated_box_ = true;

    md_->update_box_min_ = Eigen::Vector2i(0, 0);
    md_->update_box_max_ = Eigen::Vector2i(0, 0);

    caster_.reset(new RayCaster);
    caster_->setParams(mp_->resolution_, mp_->map_origin_);
}

double Map::getResolution() const {
    return mp_->resolution_;
}

int Map::getVoxelNum() const {
    return mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1);
}

int Map::getVoxelNumX() const {
    return mp_->map_voxel_num_(0);
}

int Map::getVoxelNumY() const {
    return mp_->map_voxel_num_(1);
}

double Map::getOriginX() const {
    return mp_->map_origin_(0);
}

double Map::getOriginY() const {
    return mp_->map_origin_(1);
}

double Map::getSizeX() const {
    return mp_->map_size_(0);
}

double Map::getSizeY() const {
    return mp_->map_size_(1);
}

void Map::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) const {
  // id: (x,y) index of one voxel in the map, starting from 0
  // pos: (x,y) coordinates of one point in the map
  // calculates the index of the voxel in which the point is located
  for (int i = 0; i < 2; ++i)
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

void Map::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) const {
  // calculates the (x,y) coordinates of the center of the voxel with index id
  for (int i = 0; i < 2; ++i)
    pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

int Map::toAddress(const int& x, const int& y) const {
  // returns the address of the voxel with index (x,y) in the occupancy buffer
  // the address is the index of the voxel in the 1D occupancy buffer
  int X = getVoxelNumX();
  return y * X + x;
}

int Map::toAddress(const Eigen::Vector2i& id) const {
  return toAddress(id[0], id[1]);
}

bool Map::isInMap(const Eigen::Vector2d& pos) const {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4)
    return false;
  return true;
}

bool Map::isInMap(const Eigen::Vector2i& idx) const {
  if (idx(0) < 0 || idx(1) < 0) return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1)
    return false;
  return true;
}

int Map::getOccupancy(const Eigen::Vector2i& id) const {
  if (!isInMap(id)) return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_->min_occupancy_log_) return OCCUPIED;
  return FREE;
}

int Map::getOccupancy(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

int Map::getInflatedOccupancy(const Eigen::Vector2i& id) const {
  if (!isInMap(id)) return -1;
  int occ = md_->occupancy_buffer_inflate_[toAddress(id)];
  return occ;
}

int Map::getInflatedOccupancy(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  return getInflatedOccupancy(id);
}

double Map::getDistance(const Eigen::Vector2i& id) const {
  if (!isInMap(id)) return -1;
  return md_->distance_buffer_[toAddress(id)];
}

double Map::getDistance(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  return getDistance(id);
}

double Map::getDistWithGrad(const Eigen::Vector2d& pos, Eigen::Vector2d& grad) const {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* bilinear interpolation */
  Eigen::Vector2d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector2d::Ones();
  Eigen::Vector2i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector2d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) +
             diff[1] * (values[1][1] - values[0][1])) * mp_->resolution_inv_;
  grad[1] = (v1 - v0) * mp_->resolution_inv_;

  return dist;
}

void Map::setOccupancy(const Eigen::Vector2d& pos, int occ) {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  setOccupancy(id, occ);
}

void Map::setOccupancy(const Eigen::Vector2i& id, int occ) {
  if (!isInMap(id)) return;
  if (occ == UNKNOWN) setOccupancyValue(id, mp_->clamp_min_log_ - mp_->unknown_flag_);
  else if (occ == FREE) setOccupancyValue(id, mp_->clamp_min_log_);
  else if (occ == OCCUPIED) setOccupancyValue(id, mp_->clamp_max_log_);
}

void Map::setOccupancyValue(const Eigen::Vector2i& id, double occ) {
  if (!isInMap(id)) return;
  md_->occupancy_buffer_[toAddress(id)] = occ;
  if (md_->reset_updated_box_) {
    md_->update_box_min_ = id;
    md_->update_box_max_ = id;
    md_->reset_updated_box_ = false;
  } else {
    for (int k = 0; k < 2; ++k) {
      md_->update_box_min_[k] = std::min(md_->update_box_min_[k], id[k]);
      md_->update_box_max_[k] = std::max(md_->update_box_max_[k], id[k]);
    }
  }
}

void Map::getMapBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax) const {
  // returns the bounding box of the map in voxel indices
  bmin = mp_->map_box_min_;
  bmax = mp_->map_box_max_;
}

void Map::getUpdatedBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, bool reset) {
  // returns the bounding box of the updated area in the map
  bmin = md_->update_box_min_;
  bmax = md_->update_box_max_;
  if (reset) md_->reset_updated_box_ = true;
}

void Map::inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2i& inflation_size) const {
  bmin -= inflation_size;
  bmax += inflation_size;
}

void Map::inflateBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2d& inflation_size) const {
  Eigen::Vector2i inflation_voxel;
  for (int i = 0; i < 2; ++i)
    inflation_voxel(i) = ceil((inflation_size(i) * mp_->resolution_inv_));
  inflateBox(bmin, bmax, inflation_voxel);
}

void Map::fitToBox(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax, const Eigen::Vector2i& fitMin, const Eigen::Vector2i& fitMax) const {
  for (int i = 0; i < 2; ++i) {
        bmin[i] = std::max(bmin[i], fitMin[i]);
        bmax[i] = std::min(bmax[i], fitMax[i]);
    }
}

void Map::fitToMap(Eigen::Vector2i& bmin, Eigen::Vector2i& bmax) const {
  Eigen::Vector2i fitMin, fitMax;
  getMapBox(fitMin, fitMax);
  fitToBox(bmin, bmax, fitMin, fitMax);
}

void Map::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const int& point_num,
                          const Eigen::Vector2d& camera_pos)
{
  // takes an XYZ point cloud as input, but ignores z (2D map)
  // the points are expected to be expressed in the map/world coordinate frame
  // so any transformation from camera frame to map frame must happen before calling this function
  if (point_num == 0) return;
  md_->raycast_num_ += 1;

  Eigen::Vector2d update_min = camera_pos;
  Eigen::Vector2d update_max = camera_pos;

  // only reset the updated box if the flag is set to true
  // this allows the updated box to accumulate changes over multiple point cloud inputs
  // so that the frontiers don't necessarily have to be recalculated after every single point cloud input
  if (md_->reset_updated_box_) {
    posToIndex(camera_pos, md_->update_box_min_);
    md_->update_box_max_ = md_->update_box_min_;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector2d pt_w, tmp;
  Eigen::Vector2i idx;
  int vox_adr;
  double length;
  bool all_points_invalid = true;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = point_cloud.points[i];

    // Skip invalid points, return if all points are invalid
    if (!(std::isfinite(pt.x) && std::isfinite(pt.y)))
      continue;
    all_points_invalid = false;

    pt_w << pt.x, pt.y;
    // Set flag for projected point: 0 for hit, 1 for miss
    int tmp_flag;
    if (!isInMap(pt_w)) {
      // If not in map, find closest point in map and set "miss"
      pt_w = closestPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      // If the ray length exceeds the maximum, that means no obstacle was hit
      // In that case, the point is put closer to the camera and the point is considered a miss
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    setCacheOccupancy(vox_adr, tmp_flag);

    for (int k = 0; k < 2; ++k) {
      update_min[k] = std::min(update_min[k], pt_w[k]);
      update_max[k] = std::max(update_max[k], pt_w[k]);
    }

    // prevent duplicate raycasts to the same endpoint voxel
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    // raycast
    // record all voxels that are between the camera and the point as a miss
    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);
    while (caster_->nextId(idx))
      setCacheOccupancy(toAddress(idx), 0);
  }
  if (all_points_invalid)
    return;

  // Bounding box for subsequent updating
  Eigen::Vector2i update_min_id, update_max_id;
  posToIndex(update_min, update_min_id);
  posToIndex(update_max, update_max_id);
  for (int k = 0; k < 2; ++k) {
    md_->update_box_min_[k] = std::min(update_min_id[k], md_->update_box_min_[k]);
    md_->update_box_max_[k] = std::max(update_max_id[k], md_->update_box_max_[k]);
  }

  // Go through all cached voxels and update their occupancy based on hit/miss counts
  while (!md_->cache_voxel_.empty()) {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();

    // Compute the log-odds update for this voxel based on hit/miss counts
    double log_odds_update =
        md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
    
    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;

    // If cell is unknown, set it to the minimum occupancy log value
    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

    // Write to occupancy buffer, ensure value remains within bounds of clamp_min_log_ and clamp_max_log_
    md_->occupancy_buffer_[adr] = std::min(
        std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);
  }

}

Eigen::Vector2d Map::closestPointInMap(const Eigen::Vector2d& pt, const Eigen::Vector2d& camera_pt) {
  Eigen::Vector2d diff = pt - camera_pt;
  Eigen::Vector2d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector2d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 2; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void Map::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;
}

void Map::inflateObstacles() {
  // reset occupancy buffer in updated region
  for (int x = md_->update_box_min_(0); x <= md_->update_box_max_(0); ++x) {
    for (int y = md_->update_box_min_(1); y <= md_->update_box_max_(1); ++y) {
        Eigen::Vector2i id = Eigen::Vector2i(x, y);
        int current_state = getOccupancy(id);
        md_->occupancy_buffer_inflate_[toAddress(x, y)] = current_state;
    }
  }

  int inflation_voxel_num = ceil(
    std::max(mp_->obstacles_inflation_radius_, mp_->unknown_inflation_radius_) / mp_->resolution_);

  // Define bounds for obstacle inflation update: larger than the updated box
  // by the inflation radius plus one
  Eigen::Vector2i inflation_size = Eigen::Vector2i(inflation_voxel_num + 1, 
                                                   inflation_voxel_num + 1);
  Eigen::Vector2i inflation_box_min = md_->update_box_min_ - inflation_size;
  Eigen::Vector2i inflation_box_max = md_->update_box_max_ + inflation_size;
  fitToMap(inflation_box_min, inflation_box_max);

  std::vector<Eigen::Vector2i> inf_pts;

  // inflate unknown cells
  for (int x = inflation_box_min(0); x <= inflation_box_max(0); ++x) {
    for (int y = inflation_box_min(1); y <= inflation_box_max(1); ++y) {
        Eigen::Vector2i id1 = Eigen::Vector2i(x,y);
        if (getOccupancy(id1) == Map::UNKNOWN) {
          inflatePoint(Eigen::Vector2i(x, y), mp_->unknown_inflation_radius_, inf_pts);

          for (auto inf_pt : inf_pts) {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 &&
                idx_inf < md_->occupancy_buffer_inflate_.size()) {
              md_->occupancy_buffer_inflate_[idx_inf] = Map::UNKNOWN;
            }
          }
        }
    }
  }

  // inflate newest occupied cells
  for (int x = inflation_box_min(0); x <= inflation_box_max(0); ++x) {
    for (int y = inflation_box_min(1); y <= inflation_box_max(1); ++y) {
        Eigen::Vector2i id1 = Eigen::Vector2i(x,y);
        if (getOccupancy(id1) == Map::OCCUPIED) {
          inflatePoint(Eigen::Vector2i(x, y), mp_->obstacles_inflation_radius_, inf_pts);

          for (auto inf_pt : inf_pts) {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 &&
                idx_inf < md_->occupancy_buffer_inflate_.size()) {
              md_->occupancy_buffer_inflate_[idx_inf] = Map::OCCUPIED;
            }
          }
        }
    }
  }
}

void Map::inflatePoint(const Eigen::Vector2i& pt, const double inflation_radius,
                        std::vector<Eigen::Vector2i>& pts) {
  // Inflate the point by the obstacle inflation radius in voxel units
  pts.clear();
  double radius_voxel = inflation_radius / mp_->resolution_;
  int step = ceil(radius_voxel);

  for (int x = -step; x <= step; ++x) {
    for (int y = -step; y <= step; ++y) {
      Eigen::Vector2i inf_pt(pt(0) + x, pt(1) + y);
      if (x*x + y*y <= radius_voxel * radius_voxel && isInMap(inf_pt)) {
        pts.push_back(inf_pt);
      }
    }
  }
}

void Map::manualUpdate(const std::vector<Eigen::Vector2i> updated_cells, const std::vector<int> updated_occupancies) {
  Eigen::Vector2i update_box_min = Eigen::Vector2i(0,0);
  Eigen::Vector2i update_box_max = Eigen::Vector2i(0,0);

  for (size_t i = 0; i < updated_cells.size(); ++i) {
    setOccupancy(updated_cells[i], updated_occupancies[i]);
    if (i == 0) {
      update_box_min = updated_cells[i];
      update_box_max = updated_cells[i];
    } else {
      update_box_min(0) = std::min(update_box_min(0), updated_cells[i](0));
      update_box_min(1) = std::min(update_box_min(1), updated_cells[i](1));
      update_box_max(0) = std::max(update_box_max(0), updated_cells[i](0));
      update_box_max(1) = std::max(update_box_max(1), updated_cells[i](1));
    }
  }

  md_->update_box_min_ = update_box_min;
  md_->update_box_max_ = update_box_max;
}

void Map::updateESDF2d() {
  // Update the ESDF only for the updated box
  // Eigen::Vector2i min_esdf = md_->update_box_min_;
  // Eigen::Vector2i max_esdf = md_->update_box_max_;

  // Update the ESDF for the entire map
  // Eigen::Vector2i min_esdf = mp_->map_box_min_;
  // Eigen::Vector2i max_esdf = mp_->map_box_max_;

  // Update the ESDF for an enlarged updated box
  const int inflation_voxels = 
    static_cast<int>(std::ceil(mp_->esdf_inflation_ / mp_->resolution_));
  Eigen::Vector2i inflation_vec_voxel(inflation_voxels, inflation_voxels);

  Eigen::Vector2i min_esdf = md_->update_box_min_ - inflation_vec_voxel;
  Eigen::Vector2i max_esdf = md_->update_box_max_ + inflation_vec_voxel;

  fitToMap(min_esdf, max_esdf);
  
  // Seed cells: occupied cells, plus unknown cells when optimistic_ is false.
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      const int adr = toAddress(x, y);
      const bool seed =
          getInflatedOccupancy(Eigen::Vector2i(x, y)) == Map::OCCUPIED ||
          (!mp_->optimistic_ && getInflatedOccupancy(Eigen::Vector2i(x, y)) == UNKNOWN);
      md_->tmp_buffer1_[adr] = seed ? 0.0 : std::numeric_limits<double>::max();
    }
  }

  // First 1D pass: x.
  for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
    fillESDF(
        [&](int x) { return md_->tmp_buffer1_[toAddress(x, y)]; },
        [&](int x, double val) { md_->tmp_buffer2_[toAddress(x, y)] = val; },
        min_esdf[0], max_esdf[0], 0);
  }

  // Second 1D pass: y; convert squared voxel distance to world distance.
  for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
    fillESDF(
        [&](int y) { return md_->tmp_buffer2_[toAddress(x, y)]; },
        [&](int y, double val) {
          md_->distance_buffer_[toAddress(x, y)] =
              mp_->resolution_ * std::sqrt(val);
        },
        min_esdf[1], max_esdf[1], 1);
  }

  // Compute negative distance
  if (mp_->signed_dist_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
        const int adr = toAddress(x, y);
        const bool seed =
            getInflatedOccupancy(Eigen::Vector2i(x, y)) == Map::FREE;
        md_->tmp_buffer1_[adr] = seed ? 0.0 : std::numeric_limits<double>::max();
      }
    }

    // First 1D pass: x.
    for (int y = min_esdf[1]; y <= max_esdf[1]; ++y) {
      fillESDF(
          [&](int x) { return md_->tmp_buffer1_[toAddress(x, y)]; },
          [&](int x, double val) { md_->tmp_buffer2_[toAddress(x, y)] = val; },
          min_esdf[0], max_esdf[0], 0);
    }

    // Second 1D pass: y; convert squared voxel distance to world distance.
    for (int x = min_esdf[0]; x <= max_esdf[0]; ++x) {
      fillESDF(
          [&](int y) { return md_->tmp_buffer2_[toAddress(x, y)]; },
          [&](int y, double val) {
            md_->distance_buffer_neg_[toAddress(x, y)] =
                mp_->resolution_ * std::sqrt(val);
          },
          min_esdf[1], max_esdf[1], 1);
    }

    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x) {
      for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
          int idx = toAddress(x, y);
          if (md_->distance_buffer_neg_[idx] > 0.0)
            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void Map::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  std::vector<int> v(mp_->map_voxel_num_(dim));
  std::vector<double> z(mp_->map_voxel_num_(dim) + 1.0);

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}