#include "multi_map_manager.hpp"

MultiMapManager::MultiMapManager() {
}

MultiMapManager::~MultiMapManager() {
}

void MultiMapManager::setMap(std::shared_ptr<Map> map) {
    map_ = map;
}

void MultiMapManager::init(int robot_id, int map_num, int chunk_size) {
    robot_id_ = robot_id;
    map_num_ = map_num;
    chunk_size_ = chunk_size;

    multi_map_chunks_.resize(map_num_);
    for (auto& data : multi_map_chunks_) {
        data.idx_list_ = {};
    }
    
    chunk_boxes_.resize(map_num_);
    for (auto& box : chunk_boxes_) {
        box.valid_ = false;
    }

    chunk_buffer_.resize(map_num_);
    buffer_map_.resize(map_num_);
    last_chunk_stamp_time_.resize(map_num_);
    for (auto& time : last_chunk_stamp_time_) time = 0.0;
}

void MultiMapManager::updateMapChunk(const std::vector<uint32_t>& adrs) {
    // This is called once after each point cloud update

    // Append new addresses to the buffer
    adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());

    // Check if the buffer has enough addresses to form a chunk
    if (adr_buffer_.size() >= chunk_size_) {
        // Insert chunk from too long buffer
        int i = 0;
        // Find out how many chunks need to be created from the current buffer
        // Then populate chunks from the buffer and add them to the multi_map_chunks_ for this robot
        for (; i + chunk_size_ < adr_buffer_.size(); i += chunk_size_) {
            MapChunk chunk;
            chunk.voxel_adrs_.insert(
                chunk.voxel_adrs_.end(), adr_buffer_.begin() + i, adr_buffer_.begin() + i + chunk_size_);
            chunk.idx_ = multi_map_chunks_[robot_id_].chunks_.size() + 1;
            chunk.need_query_ = true;
            chunk.empty_ = false;
            multi_map_chunks_[robot_id_].chunks_.push_back(chunk);
        }

        if (multi_map_chunks_[robot_id_].idx_list_.empty()) {
            multi_map_chunks_[robot_id_].idx_list_ = { 1, 1 };
        }
        multi_map_chunks_[robot_id_].idx_list_.back() =
            multi_map_chunks_[robot_id_].chunks_.back().idx_;

        // Remove already inserted data
        std::vector<uint32_t> tmp;
        tmp.insert(tmp.end(), adr_buffer_.begin() + i, adr_buffer_.end());
        adr_buffer_ = tmp;
    }
}

void MultiMapManager::sendStamp(double time,
                                    environment_map_msgs::msg::ChunkStamps& msg) {
    // Send stamp of chunks to other robots
    msg.from_robot_id = robot_id_;
    msg.time = time;

    for (auto chunks : multi_map_chunks_) {
        environment_map_msgs::msg::IdxList idx_list;
        idx_list.ids = chunks.idx_list_;
        msg.idx_lists.push_back(idx_list);
    }
}

void MultiMapManager::receiveStamp(const environment_map_msgs::msg::ChunkStamps& msg_in,
                                   std::vector<environment_map_msgs::msg::ChunkData>& msgs_out) {
    if (msg_in.from_robot_id == robot_id_) return;

    // Check msg time to avoid overwhelming
    if (msg_in.time - last_chunk_stamp_time_[msg_in.from_robot_id] < 0.3) return;
    
    last_chunk_stamp_time_[msg_in.from_robot_id] = msg_in.time;

    // Check others' stamp info and send chunks unknown by them
    for (int i = 0; i < multi_map_chunks_.size(); ++i) {
        if (i == msg_in.from_robot_id) continue;
        std::vector<int> missed;
        findMissedChunkIds(multi_map_chunks_[i].idx_list_, msg_in.idx_lists[i].ids, missed);
        sendChunks(i, msg_in.from_robot_id, missed, msgs_out);
    }
}

void MultiMapManager::findMissedChunkIds(const std::vector<int>& self_idx_list,
                                        const std::vector<int>& other_idx_list,
                                        std::vector<int>& miss_ids) {
    // Compute the complement set of other idx
    if (other_idx_list.empty()) {
        miss_ids = self_idx_list;
        return;
    }

    std::vector<int> not_in_other;
    if (other_idx_list[0] > 1) {
        not_in_other.push_back(1);
        not_in_other.push_back(other_idx_list[0] - 1);
    }
    for (int i = 1; i < other_idx_list.size(); i += 2) {
        not_in_other.push_back(other_idx_list[i] + 1);
        if (i == other_idx_list.size() - 1) {
            int infinite = std::numeric_limits<int>::max();
            not_in_other.push_back(infinite);
        } else {
            not_in_other.push_back(other_idx_list[i + 1] - 1);
        }
    }

    // Compute the intersection of self and not_in_other (brute-force, O(n^2))
    for (int i = 0; i < self_idx_list.size(); i += 2) {
        for (int j = 0; j < not_in_other.size(); j += 2) {
            int minr, maxr;
            if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
                    not_in_other[j + 1], minr, maxr)) {
                miss_ids.push_back(minr);
                miss_ids.push_back(maxr);
            }
        }
    }
}

bool MultiMapManager::findIntersect(const int& min1, const int& max1, const int& min2,
                                   const int max2, int& minr, int& maxr) {
    minr = std::max(min1, min2);
    maxr = std::min(max1, max2);
    if (minr <= maxr) return true;
    return false;
}

void MultiMapManager::sendChunks(const int& chunk_robot_id,
                                 const int& to_robot_id,
                                 const std::vector<int>& idx_list,
                                 std::vector<environment_map_msgs::msg::ChunkData>& msgs) {
    auto& data = multi_map_chunks_[chunk_robot_id];

    for (int i = 0; i < idx_list.size(); i += 2) {
        for (int j = idx_list[i]; j <= idx_list[i + 1]; ++j) {
            environment_map_msgs::msg::ChunkData msg;
            msg.from_robot_id = robot_id_;
            msg.to_robot_id = to_robot_id;
            msg.chunk_robot_id = chunk_robot_id;
            msg.idx = data.chunks_[j - 1].idx_;
            msg.voxel_adrs = data.chunks_[j - 1].voxel_adrs_;
            if (chunk_robot_id == robot_id_ && data.chunks_[j - 1].need_query_) {
                // Should query the occ info in map if they are still empty
                getOccOfChunk(data.chunks_[j - 1].voxel_adrs_, data.chunks_[j - 1].voxel_occ_);
                data.chunks_[j - 1].need_query_ = false;
            }
            msg.voxel_occ = data.chunks_[j - 1].voxel_occ_;

            // // Swarm communication
            // msg.pos_x = drone_pos_[0];
            // msg.pos_y = drone_pos_[1];
            // msg.pos_z = drone_pos_[2];

            msgs.push_back(msg);
        }
    }
}

void MultiMapManager::getOccOfChunk(const std::vector<uint32_t>& adrs, std::vector<uint8_t>& occs) {
    for (auto adr : adrs) {
        uint8_t occ = map_->getOccupancy(adr);
        occs.push_back(occ);
    }
}

void MultiMapManager::receiveChunk(const environment_map_msgs::msg::ChunkData& msg) {
    // Receive chunks from other drones, store them in chunk buffer
    if (msg.from_robot_id == robot_id_) return;
    if (msg.to_robot_id != robot_id_) return;

    // Ignore chunks that are in the insertion buffer
    if (buffer_map_[msg.chunk_robot_id].find(msg.idx) !=
        buffer_map_[msg.chunk_robot_id].end())
        return;

    chunk_buffer_[msg.chunk_robot_id].push_back(msg);
    buffer_map_[msg.chunk_robot_id][msg.idx] = 1;
}

void MultiMapManager::processChunkBuffer(double inflation_radius) {
    // Process chunks in buffers
    for (int i = 0; i < chunk_buffer_.size(); ++i) {
        auto& buffer = chunk_buffer_[i];
        if (buffer.empty()) continue;

        // Compute the idx list of buffered chunks
        sort(buffer.begin(), buffer.end(),
            [](const environment_map_msgs::msg::ChunkData& chunk1,
               const environment_map_msgs::msg::ChunkData& chunk2) {
            return chunk1.idx < chunk2.idx;
            });
        std::vector<int> idx_list = { int(buffer.front().idx) };
        int last_idx = idx_list[0];
        for (int j = 1; j < buffer.size(); ++j) {
            if (buffer[j].idx - last_idx > 1) {
                idx_list.push_back(last_idx);
                idx_list.push_back(buffer[j].idx);
            }
            last_idx = buffer[j].idx;
        }
        idx_list.push_back(last_idx);

        // Update ChunksData's chunks_
        auto& chunks_data = multi_map_chunks_[i];

        // Add placeholder for chunks
        int len_inc = last_idx - chunks_data.chunks_.size();
        for (int j = 0; j < len_inc; ++j) {
            chunks_data.chunks_.push_back(MapChunk());
            auto& back_chunk = chunks_data.chunks_.back();
            back_chunk.idx_ = chunks_data.chunks_.size();
            back_chunk.empty_ = true;
            back_chunk.need_query_ = false;
        }

        // Process data in buffer
        for (auto msg : buffer) {
            auto& chunk = chunks_data.chunks_[msg.idx - 1];
            if (chunk.empty_) {  // Only insert a chunk once
                chunk.voxel_adrs_ = msg.voxel_adrs;
                chunk.voxel_occ_ = msg.voxel_occ;
                insertChunkToMap(chunk, msg.chunk_robot_id, inflation_radius);
                chunk.empty_ = false;
            }
        }

        // Update ChunksData's idx_list_
        std::vector<int> union_list;
        mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
        chunks_data.idx_list_ = union_list;

        buffer.clear();
        buffer_map_[i].clear();
    }
}

void MultiMapManager::mergeChunkIds(const std::vector<int>& list1, const std::vector<int>& list2,
                                    std::vector<int>& output) {
    if (list1.empty()) {
        output = list2;
        return;
    }

    output = list1;
    int tmp1, tmp2;
    for (int i = 0; i < list2.size(); i += 2) {
        // For each interval in list2, merge it into output list
        bool intersect = false;
        for (int j = 0; j < output.size(); j += 2) {
            if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
                output[j] = std::min(output[j], list2[i]);
                output[j + 1] = std::max(output[j + 1], list2[i + 1]);
                intersect = true;
            }
        }

        if (!intersect) {  // Insert the interval in appropriate position
            std::vector<int> tmp = { list2[i], list2[i + 1] };
            if (list2[i + 1] < output.front()) {
                output.insert(output.begin(), tmp.begin(), tmp.end());
            } else if (list2[i] > output.back()) {
                output.insert(output.end(), tmp.begin(), tmp.end());
            } else {
                for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
                    if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
                        output.insert(iter + 1, tmp.begin(), tmp.end());
                        break;
                    }
                }
            }
        }

        // Remove redundant idx
        for (auto iter = output.begin() + 1; iter != output.end() - 1;) {
            if (*iter >= *(iter + 1) - 1) {
                iter = output.erase(iter);
                iter = output.erase(iter);
            } else {
                iter += 2;
            }
        }
    }
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& robot_id,
                                       double inflation_radius) {
    std::vector<int> adrs_occupied;

    for (int i = 0; i < chunk.voxel_adrs_.size(); ++i) {
        // Insert occ info
        auto& adr = chunk.voxel_adrs_[i];
        map_->setOccupancy(adr, chunk.voxel_occ_[i]);
        // map_->md_->occupancy_buffer_[adr_tf] =
        //     chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

        // Update the chunk box
        Eigen::Vector2i idx;
        map_->adrToIndex(adr, idx);
        Eigen::Vector2d pos;
        map_->indexToPos(idx, pos);
        if (chunk_boxes_[robot_id].valid_) {
            for (int k = 0; k < 2; ++k) {
                chunk_boxes_[robot_id].min_[k] = std::min(chunk_boxes_[robot_id].min_[k], pos[k]);
                chunk_boxes_[robot_id].max_[k] = std::max(chunk_boxes_[robot_id].max_[k], pos[k]);
            }
        } else {
            chunk_boxes_[robot_id].min_ = chunk_boxes_[robot_id].max_ = pos;
            chunk_boxes_[robot_id].valid_ = true;
        }

        if (chunk.voxel_occ_[i] == Map::FREE) {
            // Set free cells in inflated occupancy buffer
            map_->setInflatedOccupancy(adr, Map::FREE);
        } else if (chunk.voxel_occ_[i] == Map::OCCUPIED) {
            // Collect all occupied addresses
            adrs_occupied.push_back(adr);
        }

        /*
        // Update the all box
        for (int k = 0; k < 2; ++k) {
            map_->md_->all_min_[k] = std::min(map_->md_->all_min_[k], pos[k]);
            map_->md_->all_max_[k] = std::max(map_->md_->all_max_[k], pos[k]);
        }
        */
    }

    // Inflate occupied cells
    for (auto adr : adrs_occupied) {
        Eigen::Vector2i idx;
        map_->adrToIndex(adr, idx);
        std::vector<Eigen::Vector2i> inflated_idxs;
        map_->inflatePoint(idx, inflation_radius, inflated_idxs);
        for (auto inf_idx : inflated_idxs) {
            map_->setInflatedOccupancy(inf_idx, Map::OCCUPIED);
        }
    }
}

void MultiMapManager::getChunkBoxes(std::vector<Eigen::Vector2d>& mins,
                                    std::vector<Eigen::Vector2d>& maxs,
                                    bool reset) {
    for (auto& box : chunk_boxes_) {
        if (box.valid_) {
            mins.push_back(box.min_);
            maxs.push_back(box.max_);
            if (reset) box.valid_ = false;
        }
    }
}