/*
 * move_to_saved_pose.cpp
 *
 *  Created on: September 30, 2013
 *      Author: Tobias Fromm
 */

#include <ros/ros.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_saved_pose/MoveToSavedPose.h>
#include <map>

ros::Publisher goalPub;
ros::Subscriber feedbackSub;
bool busy = false;
bool succeeded = false;
std::map<std::string, robot_state::RobotModelPtr> model;

bool moveToSavedPose(
        moveit_saved_pose::MoveToSavedPose::Request &req,
        moveit_saved_pose::MoveToSavedPose::Response &res) {
     
  busy = true;
  std::map<std::string, robot_state::RobotModelPtr>::iterator it = model.find(req.move_group_name);
  moveit_msgs::MoveGroupActionGoal moveGroupGoal;
  if(it == model.end()) {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    model.insert(std::pair<std::string, robot_state::RobotModelPtr>(req.move_group_name, kinematic_model));
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues(kinematic_state->getJointModelGroup(req.move_group_name), req.pose_name);
    moveGroupGoal.goal.request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(*kinematic_state, kinematic_state->getJointModelGroup(req.move_group_name)));
  } else {
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(it->second));
    kinematic_state->setToDefaultValues(kinematic_state->getJointModelGroup(req.move_group_name), req.pose_name);
    moveGroupGoal.goal.request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(*kinematic_state, kinematic_state->getJointModelGroup(req.move_group_name)));
  }
  moveGroupGoal.goal.request.group_name = req.move_group_name;
  if(req.planner_name != ""){
    moveGroupGoal.goal.request.planner_id = req.planner_name;
  } else {
      ROS_ERROR("No planner name given in the MoveToSavedPose request!");
      return false;
  }
  goalPub.publish(moveGroupGoal);

  while(busy){
    ros::spinOnce();
  }

  return succeeded;
}

// this gets called as soon as any planning request is fulfilled
void moveCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr& result) {

  succeeded = (result->status.status == actionlib_msgs::GoalStatus::SUCCEEDED);
  busy = false;

}

int main(int argc, char **argv)
{

  ros::init (argc, argv, "moveit_saved_pose");
  ros::NodeHandle nh;

  ros::ServiceServer goalService = nh.advertiseService((std::string)"/moveit_saved_pose/move", &moveToSavedPose);

  goalPub = nh.advertise<moveit_msgs::MoveGroupActionGoal>("/move_group/goal", 1);
  
  feedbackSub = nh.subscribe("/move_group/result", 1, moveCallback);

  ros::spin();
  
  return 0;
}
