#include <cmath>
#include <limits>

#include "raycast.hpp"

// From "A Fast Voxel Traversal Algorithm for Ray Tracing"
// by John Amanatides and Andrew Woo, 1987
// <http://www.cse.yorku.ca/~amana/research/grid.pdf>
// <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>

// The foundation of this algorithm is a parameterized representation of
// the provided ray,
//                    origin + t * direction,
// except that t is not actually stored; rather, at any given point in the
// traversal, we keep track of the *greater* t values which we would have
// if we took a step sufficient to cross a cube boundary along that axis
// (i.e. change the integer part of the coordinate) in the variables
// tMaxX and tMaxY.

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

void RayCaster::setParams(const double& res, const Eigen::Vector2d& origin) {
    resolution_ = res;
    half_ = Eigen::Vector2d(0.5, 0.5);
    offset_ = half_ - origin / resolution_;
}

void RayCaster::input(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    start_ = start / resolution_;
    end_ = end / resolution_;

    x_ = (int)std::floor(start_.x());
    y_ = (int)std::floor(start_.y());
    endX_ = (int)std::floor(end_.x());
    endY_ = (int)std::floor(end_.y());

    // Cell traversal direction (either -1, 0, or 1)
    stepX_ = signum(endX_ - x_);
    stepY_ = signum(endY_ - y_);

    // Geometric ray direction
    const double directionX = end_.x() - start_.x();
    const double directionY = end_.y() - start_.y();

    if (directionX == 0.0) {
        tMaxX_ = std::numeric_limits<double>::infinity();
        tDeltaX_ = std::numeric_limits<double>::infinity();
    } else {
        tMaxX_ = intbound(start_.x(), directionX);
        tDeltaX_ = 1.0 / std::abs(directionX);
    }

    if (directionY == 0.0) {
        tMaxY_ = std::numeric_limits<double>::infinity();
        tDeltaY_ = std::numeric_limits<double>::infinity();
    } else {
        tMaxY_ = intbound(start_.y(), directionY);
        tDeltaY_ = 1.0 / std::abs(directionY);
    }

    /*
    // See description above. The initial values depend on the fractional
    // part of the origin.
    tMaxX_ = intbound(start_.x(), dx_);
    tMaxY_ = intbound(start_.y(), dy_);

    // The change in t when taking a step (always positive).
    tDeltaX_ = ((double)stepX_) / dx_;
    tDeltaY_ = ((double)stepY_) / dy_;
    
    // Avoids an infinite loop.
    if (stepX_ == 0 && stepY_ == 0)
        return false;
    else
        return true;
    */
}

bool RayCaster::nextId(Eigen::Vector2i& idx) {
    auto tmp = Eigen::Vector2d(x_, y_);
    idx = (tmp + offset_).cast<int>();

    if (x_ == endX_ && y_ == endY_) {
        return false;
    }

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y. Therefore, choosing the least tMax
    // chooses the closest cube boundary.
    if (tMaxX_ < tMaxY_) {
        // Update which cube we are now in.
        x_ += stepX_;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX_ += tDeltaX_;
    } else {
        y_ += stepY_;
        tMaxY_ += tDeltaY_;
    }

    return true;
}