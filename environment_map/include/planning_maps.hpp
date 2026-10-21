#pragma once

#include "map.hpp"

class PlanningMaps {
    
public:
    PlanningMaps(const Map::InitMapParams& p, size_t max_readers) {
        slots_.resize(max_readers + 2);
        for (auto& s : slots_) {
            s = std::make_shared<Map>();
            s->initMap(p);
        }
    };

    ~PlanningMaps() {
        for (auto& s : slots_) {
            s.reset();
        }
    };

    // Planner: snapshot of the latest map, valid for the whole callback.
    std::shared_ptr<const Map> acquire(bool reset) {
        std::lock_guard lk(m_);
        reset_updated_box_ = reset;
        return slots_[latest_];
    };

    std::shared_ptr<const Map> acquire() const {
        std::lock_guard lk(m_);
        return slots_[latest_];
    };

    // Updater: check if the updated box needs to be reset
    bool getResetUpdatedBox(bool reset) {
        std::lock_guard lk(m_);
        bool current = reset_updated_box_;
        reset_updated_box_ = reset;
        return current;
    }

    // Updater (single writer): copy the real map's data into a free slot, then publish it.
    bool publish(const Map& real) {
        int target = -1;
        {
            std::lock_guard lk(m_);
            for (int i = 0; i < slots_.size(); ++i)
                if (i != latest_ && slots_[i].use_count() == 1) { target = i; break; }
        }
        if (target < 0) return false;

        // no lock needed: nobody can acquire this slot
        slots_[target]->setDataFrom(real);

        std::lock_guard lk(m_);
        latest_ = target;
        return true;
    };

private:
    std::vector<std::shared_ptr<Map>> slots_;
    mutable std::mutex m_;
    int latest_ = 0;
    bool reset_updated_box_ = true;
};