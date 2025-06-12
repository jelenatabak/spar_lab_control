#include "control.h"

Control::Control (ros::NodeHandle& nodeHandle) {

    joint_ = false;
    pose_ = false;
    position_ = false;

    velocity_scale_ = 0.5;
    acceleration_scale_ = 0.5;


    goToJointGoalSrv_ = nodeHandle.advertiseService("/go_to_joint_goal",
        &Control::goToJointGoalCallback, this);
    goToPoseGoalSrv_ = nodeHandle.advertiseService("/go_to_pose_goal",
        &Control::goToPoseGoalCallback, this);
    goToPositionSrv_ = nodeHandle.advertiseService("/go_to_position_goal",
        &Control::goToPositionCallback, this);

    missionDoneClient_ = nodeHandle.serviceClient<std_srvs::Trigger>("/mission_done");

    nodeHandle.getParam("/spar_lab_control/planning_group", planning_group_);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (planning_group_);

    move_group_->setStartStateToCurrentState();
    // move_group_->setNumPlanningAttempts(10);
    // move_group_->setPlanningTime(20);
    move_group_->clearPathConstraints();

    std::cout << "Reference frame: " << move_group_->getPlanningFrame() << std::endl;
    std::cout << "End effector link: " << move_group_->getEndEffectorLink() << std::endl;
    std::cout << "Pose: " << move_group_->getCurrentPose() << std::endl;
    std::vector<double> states = move_group_->getCurrentJointValues();


    // Add collisions
    // ros::Duration(2).sleep();
    // float z_board;
    // nodeHandle.getParam("/spar_lab_control/z", z_board);
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(addBox("board", 2,2,0.01,-1,0,-0.03));      
    // // collision_objects.push_back(addBox("board2", 2,0.7,0.01,-1,-0.82,0.095));
    // // collision_objects.push_back(addBox("wall", 2,0.2,2,-1,0.75,0));
    // planning_scene_interface_.addCollisionObjects(collision_objects);
    // std::cout << "Added collision" << std::endl;
    // std::cout << "Robot is ready!" << std::endl;
}

void Control::control_loop() {
    while (ros::ok()) {
        if (joint_) {
            goToJointGoal();
            missionDoneClient_.call(trigger);
        }
        else if (pose_){
            goToPoseGoal();
            missionDoneClient_.call(trigger);
        }
        else if (position_){
            goToPosition();
            missionDoneClient_.call(trigger);
        }
    }
}

void Control::goToJointGoal() {
    std::cout << "Moving to joint state" << std::endl;
    try {
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit_msgs::MoveItErrorCodes errorCode;

        move_group_->setJointValueTarget(jointPositionReference_);
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        errorCode = move_group_->move();
        std::cout << "Done" << std::endl;

        if (errorCode.val < 0) {
            throw std::runtime_error("Did not move!");
        }

        joint_ = false;


    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void Control::goToPoseGoal() {
    std::cout << "Moving to pose goal" << std::endl;
    try{
        tf2::Quaternion quat;
        tf2::fromMsg(poseReference_.orientation, quat);
        quat.normalize();
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(quat);
        poseReference_.orientation = quat_msg;
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(poseReference_);
        move_group_->move();
        std::cout << "Done" << std::endl;

        pose_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void Control::goToPosition() {
    std::cout << "Moving to position goal" << std::endl;
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    try{
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();

        geometry_msgs::Pose poseRef;
        poseRef.position = positionReference_;

        if(false) 
        {
            poseRef.orientation.x = 0.0;
            poseRef.orientation.y = 0.7071068;
            poseRef.orientation.z = 0.7071068;
            poseRef.orientation.w = 0.0;

            // poseRef.orientation.x = -0.7071068;
            // poseRef.orientation.y = 0.0;
            // poseRef.orientation.z = 0.0;
            // poseRef.orientation.w = 0.7071068;

            poseRef.orientation.x = 0.0;
            poseRef.orientation.y = 1;
            poseRef.orientation.z = 0.0;
            poseRef.orientation.w = 0.0;
        }        

        tf2::Quaternion quat;
        tf2::fromMsg(poseRef.orientation, quat);
        quat.normalize();
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(quat);
        poseRef.orientation = quat_msg;

        std::cout << poseRef << std::endl;

        move_group_->setPoseTarget(poseRef);
        // move_group_->setApproximateJointValueTarget(poseRef);
        move_group_->move();

        std::cout << "Done" << std::endl;
        move_group_->clearPathConstraints();

        position_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

bool Control::goToJointGoalCallback(spar_lab_control::jointGoal::Request &req, spar_lab_control::jointGoal::Response &res){
    joint_ = true;
    jointPositionReference_ = req.joint_states;
    return true;
}

bool Control::goToPoseGoalCallback(spar_lab_control::poseGoal::Request &req, spar_lab_control::poseGoal::Response &res){
    pose_ = true;
    poseReference_ = req.pose;
    return true;
}

bool Control::goToPositionCallback(spar_lab_control::positionGoal::Request &req, spar_lab_control::positionGoal::Response &res){
    position_ = true;
    positionReference_ = req.position;
    return true;
}

// args: collision object id, box size, position in world frame (min x, y, min z)
moveit_msgs::CollisionObject Control::addBox(const char* name, float box_x, float box_y, float box_z, float x, float y, float z) {
  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = "base_link";
  collision_object.id = name;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = box_x;
  primitive.dimensions[1] = box_y;
  primitive.dimensions[2] = box_z;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1;
  box_pose.position.x = x + box_x/2;
  box_pose.position.y = y;
  box_pose.position.z = z + box_z/2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR5e");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    Control robot = Control(nodeHandle);
    robot.control_loop();
}