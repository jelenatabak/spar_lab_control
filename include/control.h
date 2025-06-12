#pragma once

#include <ros/package.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <stdexcept>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spar_lab_control/jointGoal.h>
#include <spar_lab_control/poseGoal.h>
#include <spar_lab_control/positionGoal.h>


class Control {
  public:
    Control(ros::NodeHandle& nodeHandle);
    bool goToJointGoalCallback(spar_lab_control::jointGoal::Request &req, spar_lab_control::jointGoal::Response &res);
    bool goToPoseGoalCallback(spar_lab_control::poseGoal::Request &req, spar_lab_control::poseGoal::Response &res);
    bool goToPositionCallback(spar_lab_control::positionGoal::Request &req, spar_lab_control::positionGoal::Response &res);

    moveit_msgs::CollisionObject addBox(const char* name, float box_x, float box_y, float box_z, float x, float y, float z);

    void control_loop();
    void goToJointGoal();
    void goToPoseGoal();
    void goToPosition();

  private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer goToJointGoalSrv_;
    ros::ServiceServer goToPoseGoalSrv_;
    ros::ServiceServer goToPositionSrv_;

    ros::ServiceClient missionDoneClient_;
    std_srvs::Trigger trigger;


    bool joint_, pose_, position_;

    std::vector<double> jointPositionReference_;
    geometry_msgs::Pose poseReference_;
    geometry_msgs::Point positionReference_;

    std::string planning_group_;
    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    double velocity_scale_, acceleration_scale_;
};