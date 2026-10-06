#include "perception_utils.hpp"

PerceptionUtils::PerceptionUtils() {
}

PerceptionUtils::~PerceptionUtils() {
}

void PerceptionUtils::initPercepUtils(const InitFOVParams& fov_params) {
    offset_x_ = fov_params.offset_x;
    offset_y_ = fov_params.offset_y;
    left_angle_ = fov_params.left_angle;
    right_angle_ = fov_params.right_angle;
    max_dist_ = fov_params.max_dist;
}

void PerceptionUtils::setPose(const Eigen::Vector2d& camera_pos, const double& yaw) {
    camera_pos_ = camera_pos;
    yaw_ = yaw;
}

void PerceptionUtils::getRobotPos(Eigen::Vector2d& robot_pos) {
    robot_pos.x() = camera_pos_.x() - offset_x_*std::cos(yaw_) + offset_y_*std::sin(yaw_);
    robot_pos.y() = camera_pos_.y() - offset_x_*std::sin(yaw_) - offset_y_*std::cos(yaw_);
}

void PerceptionUtils::getCameraPosFromRobotPose(Eigen::Vector2d& camera_pos,
                                                const Eigen::Vector2d& robot_pos,
                                                const double& yaw) {
    camera_pos.x() = robot_pos.x() + offset_x_*std::cos(yaw) - offset_y_*std::sin(yaw);
    camera_pos.y() = robot_pos.y() + offset_x_*std::sin(yaw) + offset_y_*std::cos(yaw);
}

bool PerceptionUtils::isInsideFOV(const Eigen::Vector2d& point) {
    Eigen::Vector2d dir = point - camera_pos_;
    double distance = dir.norm();

    if (distance > max_dist_) return false;
    if (distance < 1e-9) return true;

    dir /= distance;

    double angle = std::atan2(dir.y(), dir.x()) - yaw_;

    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle >= left_angle_ && angle <= right_angle_;
}

void PerceptionUtils::getParams(double& left_angle, double& right_angle, double& max_dist) {
    left_angle = left_angle_;
    right_angle = right_angle_;
    max_dist = max_dist_;
}