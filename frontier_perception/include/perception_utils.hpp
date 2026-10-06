#pragma once

#include <Eigen/Eigen>

class PerceptionUtils {
public:
    PerceptionUtils();
    ~PerceptionUtils();

    struct InitFOVParams {
        double offset_x;
        double offset_y;
        double left_angle;
        double right_angle;
        double max_dist;
    };

    void initPercepUtils(const InitFOVParams& fov_params);
    void setPose(const Eigen::Vector2d& camera_pos, const double& yaw);
    void getRobotPos(Eigen::Vector2d& robot_pos);
    void getCameraPosFromRobotPose(Eigen::Vector2d& camera_pos,
                                   const Eigen::Vector2d& robot_pos,
                                   const double& yaw);
    bool isInsideFOV(const Eigen::Vector2d& point);
    void getParams(double& left_angle, double& right_angle, double& max_dist);

private:
    // Data
    // Current camera position and yaw
    Eigen::Vector2d camera_pos_;
    double yaw_;

    // Params
    // offset of the sensor from the robot center in the x and y directions
    double offset_x_, offset_y_;

    // Sensing range of camera
    double left_angle_, right_angle_, max_dist_;

};