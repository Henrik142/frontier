#pragma once

#include <Eigen/Eigen>

int signum(int x);

double mod(double value, double modulus);

double intbound(double s, double ds);

class RayCaster {
public:
    RayCaster() {
    }
    ~RayCaster() {
    }

    void setParams(const double& res, const Eigen::Vector2d& origin);
    void input(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
    bool nextId(Eigen::Vector2i& idx);

private:
    /* data */
    Eigen::Vector2d start_;
    Eigen::Vector2d end_;
    int x_;
    int y_;
    int endX_;
    int endY_;
    int stepX_;
    int stepY_;
    double tMaxX_;
    double tMaxY_;
    double tDeltaX_;
    double tDeltaY_;

    double resolution_;
    Eigen::Vector2d offset_;
    Eigen::Vector2d half_;
};
