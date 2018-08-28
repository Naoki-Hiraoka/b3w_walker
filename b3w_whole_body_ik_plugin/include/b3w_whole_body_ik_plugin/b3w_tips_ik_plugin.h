/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#ifndef MOVEIT_ROS_PLANNING_B3W_WHOLE_BODY_IK_PLUGIN_
#define MOVEIT_ROS_PLANNING_B3W_WHOLE_BODY_IK_PLUGIN_

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit/kdl_kinematics_plugin/chainiksolver_pos_nr_jl_mimic.hpp>
#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_pinv_mimic.hpp>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace b3w_whole_body_ik_plugin
{
  using namespace kdl_kinematics_plugin;
/**
 * @brief Specific implementation of kinematics using KDL. This version can be used with any robot.
 */
class B3wTipsIkPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  B3wTipsIkPlugin();

  virtual bool getPositionIK(const geometry_msgs::Pose& ik_pose,
			     const std::vector<double>& ik_seed_state,
			     std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
			     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool getPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
			     const std::vector<double> &ik_seed_state,
			     std::vector< std::vector<double> >& solutions,
			     kinematics::KinematicsResult& result,
			     const kinematics::KinematicsQueryOptions &options) const override;
  
  virtual bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
				const std::vector<double>& ik_seed_state, double timeout,
				std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
				const std::vector<double>& ik_seed_state, double timeout,
				const std::vector<double>& consistency_limits, std::vector<double>& solution,
				moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
				const std::vector<double>& ik_seed_state, double timeout,
				std::vector<double>& solution,
				const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
				const std::vector<double>& ik_seed_state, double timeout,
				const std::vector<double>& consistency_limits,
				std::vector<double>& solution,
				const IKCallbackFn& solution_callback,
				moveit_msgs::MoveItErrorCodes& error_code,
				const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
				const std::vector<double> &ik_seed_state,
				double timeout,
				const std::vector<double> &consistency_limits,
				std::vector<double> &solution,
				const IKCallbackFn &solution_callback,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
				const moveit::core::RobotState* context_state = NULL) const override;
  
  virtual bool getPositionFK(const std::vector<std::string>& link_names,
			     const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const override;
  

   virtual bool initialize(const std::string &robot_description,
			   const std::string &group_name,
			   const std::string &base_name,
			   const std::string &tip_frame,
			   double search_discretization) override{
     std::vector<std::string> tip_frames;
     tip_frames.push_back(tip_frame);
     initialize(robot_description, group_name, base_name, tip_frames, search_discretization);
   }

  virtual bool initialize(const std::string &robot_description,
			  const std::string &group_name,
			  const std::string &base_name,
			  const std::vector<std::string>& tip_frames,
			  double search_discretization) override;
  
  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const override;

  bool supportsGroup(const moveit::core::JointModelGroup *jmg, std::string* error_text_out = NULL) const;
protected:

  virtual bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices);

private:
  bool timedOut(const ros::WallTime& start_time,
		double duration) const;

  /** @brief Check whether the solution lies within the consistency limit of the seed state
   *  @param seed_state Seed state
   *  @param redundancy Index of the redundant joint within the chain
   *  @param consistency_limit The returned state for redundant joint should be in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param solution solution configuration
   *  @return true if check succeeds
   */
  bool checkConsistency(const KDL::JntArray& seed_state,
			const std::vector<double>& consistency_limit,
                        const KDL::JntArray& solution) const;

  int getJointIndex(const unsigned int idx,const std::string& name) const;

  int getKDLSegmentIndex(const unsigned int idx,const std::string& name) const;

  void getRandomConfiguration(const unsigned int idx,
			      KDL::JntArray& jnt_array,
			      bool lock_redundancy) const;

  /** @brief Get a random configuration within joint limits close to the seed state
   *  @param seed_state Seed state
   *  @param redundancy Index of the redundant joint within the chain
   *  @param consistency_limit The returned state will contain a value for the redundant joint in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param jnt_array Returned random configuration
   */
  void getRandomConfiguration(const unsigned int idx,
			      const KDL::JntArray& seed_state,
			      const std::vector<double>& consistency_limits,
                              KDL::JntArray& jnt_array,
			      bool lock_redundancy) const;

  bool isRedundantJoint(unsigned int index) const;

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  mutable random_numbers::RandomNumberGenerator random_number_generator_;

  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr state_;
  robot_model::JointModelGroup* joint_model_group_;
  std::map<std::string,unsigned int> joint_model_indexs_;//active,mimicjointのindex(redundant含む)
  
  std::vector<std::string> jointnames_;
  std::vector<std::string> linknames_;

  KDL::Chain root2base_chain;
  unsigned int root_link_index_;
  std::vector<unsigned int> tip_link_indexs_;//与えられたposesが、どのsolverに対応するか
  
  //各tipごとに
  // Storage required for when the set of redundant joints is reset
  std::vector<bool> position_ik_;  // whether this solver is only being used for position ik
  std::vector<double> max_solver_iterations_;
  std::vector<double> epsilon_;
  
  std::vector<moveit_msgs::KinematicSolverInfo> ik_chain_infos_; /** Stores information for the inverse kinematics solver */
  std::vector<moveit_msgs::KinematicSolverInfo> fk_chain_infos_; /** Store information for the forward kinematics solver */
  std::vector<KDL::Chain> kdl_chains_;//base-frameから各tipへのchain
  std::vector<unsigned int> dimensions_; /** Dimension of the group */
  std::vector<KDL::JntArray> joint_mins_, joint_maxs_; /** Joint limits */
  std::vector<unsigned int> num_mimic_joints_;
  std::vector<std::vector<JointMimic> > mimic_jointss_;
  std::vector<std::vector<unsigned int> > mimic_joint_indexss_;//mimic_jointss配列上での親の位置を表す。JointMimic.map_indexとは異なる
  std::vector<int> num_possible_redundant_jointss_;
  std::vector<std::vector<unsigned int> > redundant_joints_map_indexs_;//まだ

};
}

#endif
