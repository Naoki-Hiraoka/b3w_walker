#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/GetPositionIK.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "whole_body_tips";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  robot_state::RobotState start_state(*move_group.getCurrentState());
  //start_state.setToDefaultValues();
  start_state.setVariablePosition("l_footjoint_roll",0.0);
  start_state.setVariablePosition("l_b_footjoint_pitch",0.1);
  start_state.setVariablePosition("l_footjoint_pitch",-0.1);
  start_state.setVariablePosition("l_uf_knee",0.1);
  start_state.setVariablePosition("l_db_knee",0.1);
  start_state.setVariablePosition("l_df_knee",0.1);
  start_state.setVariablePosition("l_ankle_pitch",-0.1);
  start_state.setVariablePosition("l_ankle_roll",0.0);
  start_state.setVariablePosition("l_shoulder_pitch",0.0);
  start_state.setVariablePosition("l_shoulder_roll",0.0);
  start_state.setVariablePosition("l_elbow_roll",0.0);
  start_state.setVariablePosition("r_footjoint_roll",0.0);
  start_state.setVariablePosition("r_b_footjoint_pitch",-0.1);
  start_state.setVariablePosition("r_footjoint_pitch",0.1);
  start_state.setVariablePosition("r_uf_knee",-0.1);
  start_state.setVariablePosition("r_db_knee",-0.1);
  start_state.setVariablePosition("r_df_knee",-0.1);
  start_state.setVariablePosition("r_ankle_pitch",0.1);
  start_state.setVariablePosition("r_ankle_roll",0.0);
  start_state.setVariablePosition("r_shoulder_pitch",0.0);
  start_state.setVariablePosition("r_shoulder_roll",0.0);
  start_state.setVariablePosition("r_elbow_roll",0.0);
  move_group.setStartState(start_state);
  
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "l_foot_endcoords";
  pose_stamped.pose.position.x = 0.0;
  pose_stamped.pose.position.y = 0.0;
  pose_stamped.pose.position.z = 0.0;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  move_group.setPoseTarget(pose_stamped,"l_foot_endcoords");


  pose_stamped.header.frame_id = "l_foot_endcoords";
  pose_stamped.pose.position.x = 0.07;
  pose_stamped.pose.position.y = -0.10;
  pose_stamped.pose.position.z = 0.0;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  move_group.setPoseTarget(pose_stamped,"r_foot_endcoords");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  visual_tools.prompt("next step");
  move_group.setPlanningTime(60);//60秒で中止
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  move_group.execute(my_plan);

  return 0;
}  
  
