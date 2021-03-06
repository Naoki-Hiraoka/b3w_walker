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

#include <b3w_whole_body_ik_plugin/b3w_whole_body_ik_plugin.h>
#include <class_loader/class_loader.h>

//#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(b3w_whole_body_ik_plugin::B3wWholeBodyIkPlugin, kinematics::KinematicsBase)

namespace b3w_whole_body_ik_plugin
{
  using namespace kdl_kinematics_plugin;
  
  B3wWholeBodyIkPlugin::B3wWholeBodyIkPlugin() : active_(false){
  }

  bool B3wWholeBodyIkPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
					   const std::vector<double>& ik_seed_state,
					   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
					   const kinematics::KinematicsQueryOptions& options) const{
    const std::vector<geometry_msgs::Pose> ik_poses{ik_pose};
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;
    
    return searchPositionIK(ik_poses, ik_seed_state, default_timeout_,consistency_limits, solution, solution_callback, error_code, options);
  }

  bool B3wWholeBodyIkPlugin::getPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
					   const std::vector<double> &ik_seed_state,
					   std::vector< std::vector<double> >& solutions,
					   kinematics::KinematicsResult& result,
					   const kinematics::KinematicsQueryOptions &options) const{
    ROS_ERROR_NAMED("ik", "getPositionIK!!! attention!!!");
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;
    
    return false;
  }
  
bool B3wWholeBodyIkPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_ERROR_NAMED("ik", "searchPositionIK1");
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,solution, solution_callback, error_code, options);
}

bool B3wWholeBodyIkPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_ERROR_NAMED("ik", "searchPositionIK2");
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout,consistency_limits, solution, solution_callback, error_code, options);
}

bool B3wWholeBodyIkPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_ERROR_NAMED("ik", "searchPositionIK3");
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout,consistency_limits, solution, solution_callback, error_code, options);
}

bool B3wWholeBodyIkPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                           double timeout, const std::vector<double>& consistency_limits,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_ERROR_NAMED("ik", "searchPositionIK4");
  return searchPositionIK(ik_pose, ik_seed_state, timeout,consistency_limits, solution, solution_callback, error_code, options);
}
  
  void B3wWholeBodyIkPlugin::getRandomConfiguration(const unsigned int idx,KDL::JntArray& jnt_array, bool lock_redundancy) const{//lockredundancyされていない、chain内各active,mimicjointに作用
    std::vector<double> jnt_array_vector(joint_model_group_->getActiveJointModels().size()+joint_model_group_->getMimicJointModels().size(), 0.0);//group内各active,mimicjoint
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < jnt_array.rows(); ++i)
  {
    if (lock_redundancy)
      if (isRedundantJoint(joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[i])))
        continue;
    jnt_array(i) = jnt_array_vector[joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[i])];
  }
}

bool B3wWholeBodyIkPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

  void B3wWholeBodyIkPlugin::getRandomConfiguration(const unsigned int idx,
						    const KDL::JntArray& seed_state,
						    const std::vector<double>& consistency_limits,
						    KDL::JntArray& jnt_array, bool lock_redundancy) const//chain内、active,mimicに作用
{
  std::vector<double> values(ik_chain_infos_[idx].joint_names.size(), 0.0);//group内active,mimic
  std::vector<double> near(ik_chain_infos_[idx].joint_names.size(), 0.0);//group内active,mimic
  for (std::size_t i = 0; i < seed_state.rows(); ++i)
    near[i] = seed_state(i);

  for(int i=0;i<dimensions_[idx];i++){//ランダムな値に
    joint_model_group_->getJointModel(ik_chain_infos_[idx].joint_names[i])->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(),values.data()+i,near.data()+i,consistency_limits[i]);
  }
  for(int i=0;i<mimic_jointss_[idx].size();i++){//mimicを処理
    if(!mimic_jointss_[idx][i].active){
      values[i]=values[mimic_joint_indexss_[idx][i]]*mimic_jointss_[idx][i].multiplier+mimic_jointss_[idx][i].offset;
    }
  }

  for (std::size_t i = 0; i < dimensions_[idx]; ++i)
  {
    bool skip = false;
    if (lock_redundancy)
      if (isRedundantJoint(joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[i])))
	continue;
    jnt_array(i) = values[i];
  }
}

bool B3wWholeBodyIkPlugin::checkConsistency(const KDL::JntArray& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < seed_state.rows(); ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool B3wWholeBodyIkPlugin::initialize(const std::string& robot_description, const std::string& group_name,
				      const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                     double search_discretization)
{
  //ROS_ERROR_NAMED("temp", "CONSTRUCT");
  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);
  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  lookupParam("max_solver_iterations", max_solver_iterations, 500);
  lookupParam("epsilon", epsilon, 1e-5);
  lookupParam("position_only_ik", position_ik, false);
  ROS_DEBUG_NAMED("kdl", "Looking for param name: position_only_ik");
  if (position_ik)
    ROS_INFO_NAMED("kdl", "Using position only ik");
  
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("kdl", "URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }
  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group){
    ROS_ERROR_STREAM_NAMED("kdl","no jointmodel group named "<<group_name);
    return false;
  }
  if(joint_model_group->isChain() && tip_frames.size() > 1)
  {
    ROS_ERROR_STREAM_NAMED("temp","The joint model group specified is a chain but " << tip_frames.size() << " tip frames and poses were passed in");
    return false;
  }
  if (!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl", "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }
  int counter=0;
  for(const auto& joint: joint_model_group->getJointModels()){
    if(joint->getType() == moveit::core::JointModel::REVOLUTE || joint->getType() == moveit::core::JointModel::PRISMATIC){
      joint_model_indexs_[joint->getName()]=counter;
      jointnames_.push_back(joint->getName());
      counter++;
    }
  }
  
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_NAMED("kdl", "Could not initialize tree object");
    return false;
  }
  for(const auto& tipframe: getTipFrames()){
    KDL::Chain kdl_chain;
    if (!kdl_tree.getChain(base_frame_, tipframe, kdl_chain))//kdlchainは、rootのリンクを含んでない。[joint+link]
      {
	ROS_ERROR_NAMED("kdl", "Could not initialize chain object");
	return false;
      }
    kdl_chains_.push_back(std::move(kdl_chain));
  }

  for(const auto& kdl_chain: kdl_chains_){
    unsigned int dimension=0;
    moveit_msgs::KinematicSolverInfo ik_chain_info;
    moveit_msgs::KinematicSolverInfo fk_chain_info;
    std::vector<unsigned int> redundant_joints_map_index;
    std::vector<JointMimic> mimic_joints;
    unsigned int joint_counter = 0;
    unsigned int num_mimic_joint = 0;
    for(int i=0; i<kdl_chain.getNrOfSegments(); i++){
      if(kdl_chain.getSegment(i).getJoint().getName().empty()){
	continue;//kdlのsegmentはjointを持たないこともあるというから
      }
      const moveit::core::JointModel* joint=joint_model_group->getJointModel(kdl_chain.getSegment(i).getJoint().getName());
      if(joint->getType() == moveit::core::JointModel::REVOLUTE || joint->getType() == moveit::core::JointModel::PRISMATIC){//active or mimic
	dimension++;
	ik_chain_info.joint_names.push_back(joint->getName());
	const std::vector<moveit_msgs::JointLimits>& jvec =joint->getVariableBoundsMsg();
	ik_chain_info.limits.insert(ik_chain_info.limits.end(), jvec.begin(), jvec.end());

	// Check for mimic joints
	// first check whether it belongs to the set of active joints in the group
	if (joint->getMimic() == NULL && joint->getVariableCount() > 0){//mimic jointでない
	  JointMimic mimic_joint;
	  mimic_joint.reset(joint_counter);
	  mimic_joint.joint_name = joint->getName();
	  mimic_joint.active = true;
	  mimic_joints.push_back(mimic_joint);
	  ++joint_counter;
	}else if (joint->getMimic()/*&& joint_model_group->hasJointModel(joint->getMimic()->getName())*/){//mimic joint チェーン外にmimicの親がいるとやばそう
	  JointMimic mimic_joint;
	  mimic_joint.reset(joint_counter);
	  mimic_joint.joint_name = joint->getName();
	  mimic_joint.offset = joint->getMimicOffset();
	  mimic_joint.multiplier = joint->getMimicFactor();
	  mimic_joints.push_back(mimic_joint);
	  //++joint_counter;//勝手に付けた。大丈夫かな
	  num_mimic_joint++;
	  
	}
      }
    }
    std::vector<unsigned int> mimic_joint_indexs;
    for (std::size_t i = 0; i < mimic_joints.size(); ++i){//map_indexがmimicの親を示すようにする
      unsigned int mimic_joint_index=i;
      if (!mimic_joints[i].active){
	const robot_model::JointModel* joint_model = joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();//mimicの親
	for (std::size_t j = 0; j < mimic_joints.size(); ++j){
	  if (mimic_joints[j].joint_name == joint_model->getName()){
	    mimic_joints[i].map_index = mimic_joints[j].map_index;
	    mimic_joint_index=j;
	  }
	}
      }
      mimic_joint_indexs.push_back(mimic_joint_index);
    }
    mimic_joint_indexss_.push_back(std::move(mimic_joint_indexs));

    num_possible_redundant_jointss_.push_back(kdl_chain.getNrOfJoints() - mimic_joints.size() - (position_ik ? 3 : 6));
    redundant_joints_map_indexs_.push_back(redundant_joints_map_index);
    
    num_mimic_joints_.push_back(num_mimic_joint);
    mimic_jointss_.push_back(std::move(mimic_joints));
    
    dimensions_.push_back(dimension);
    KDL::JntArray joint_min(ik_chain_info.limits.size());
    KDL::JntArray joint_max(ik_chain_info.limits.size());
    for (unsigned int i = 0; i < ik_chain_info.limits.size(); i++) {
      joint_min(i) = ik_chain_info.limits[i].min_position;
      joint_max(i) = ik_chain_info.limits[i].max_position;
    }
    joint_mins_.push_back(std::move(joint_min));
    joint_maxs_.push_back(std::move(joint_max));

    fk_chain_info.joint_names = ik_chain_info.joint_names;
    fk_chain_info.limits = ik_chain_info.limits;
    ik_chain_info.link_names.push_back(kdl_chain.segments.back().getName());//tip link
    for(const auto& segment:kdl_chain.segments)
      fk_chain_info.link_names.push_back(segment.getName());//root以外の全リンク
    ik_chain_infos_.push_back(ik_chain_info);
    fk_chain_infos_.push_back(fk_chain_info);
  }

  // Setup the joint state groups that we need
  state_.reset(new robot_state::RobotState(robot_model_));
  state_2_.reset(new robot_state::RobotState(robot_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  for(const auto& ik_chain_info:ik_chain_infos_)
      for(const auto& name:ik_chain_info.link_names)
	linknames_.push_back(name);
  
  active_ = true;
  ROS_DEBUG_NAMED("kdl", "KDL solver initialized");
  // ROS_ERROR_STREAM_NAMED("kdl", "actives "<<joint_model_group->getActiveJointModels().size()<<" mimics"<<joint_model_group->getMimicJointModels().size());
  // for(const auto& name:getJointNames())
  //   ROS_ERROR_STREAM(name);
  // for(const auto& name:getLinkNames())
  //   ROS_ERROR_STREAM(name);
  ROS_DEBUG_NAMED("kdl", "/KDL solver initialized");
  return true;
}

bool B3wWholeBodyIkPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  ROS_ERROR_NAMED("kdl", "This group cannot have redundant joints");
  return false;
   // if (num_possible_redundant_joints_ < 0)
  // {
  //   ROS_ERROR_NAMED("kdl", "This group cannot have redundant joints");
  //   return false;
  // }
  // if (static_cast<int>(redundant_joints.size()) > num_possible_redundant_joints_)
  // {
  //   ROS_ERROR_NAMED("kdl", "This group can only have %d redundant joints", num_possible_redundant_joints_);
  //   return false;
  // }
  // std::vector<unsigned int> redundant_joints_map_index;
  // unsigned int counter = 0;
  // for (std::size_t i = 0; i < dimension_; ++i)
  // {
  //   bool is_redundant_joint = false;
  //   for (std::size_t j = 0; j < redundant_joints.size(); ++j)
  //   {
  //     if (i == redundant_joints[j])
  //     {
  //       is_redundant_joint = true;
  //       counter++;
  //       break;
  //     }
  //   }
  //   if (!is_redundant_joint)
  //   {
  //     // check for mimic
  //     if (mimic_joints_[i].active)
  //     {
  //       redundant_joints_map_index.push_back(counter);
  //       counter++;
  //     }
  //   }
  // }
  // for (std::size_t i = 0; i < redundant_joints_map_index.size(); ++i)
  //   ROS_DEBUG_NAMED("kdl", "Redundant joint map index: %d %d", (int)i, (int)redundant_joints_map_index[i]);

  // redundant_joints_map_index_ = redundant_joints_map_index;
  // redundant_joint_indices_ = redundant_joints;
  // return true;
}

  int B3wWholeBodyIkPlugin::getJointIndex(const unsigned int idx,const std::string& name) const
{
  for (unsigned int i = 0; i < ik_chain_infos_[idx].joint_names.size(); i++)
  {
    if (ik_chain_infos_[idx].joint_names[i] == name)
      return i;
  }
  return -1;
}

  int B3wWholeBodyIkPlugin::getKDLSegmentIndex(const unsigned int idx,const std::string& name) const
{
  int i = 0;
  while (i < (int)kdl_chains_[idx].getNrOfSegments())
  {
    if (kdl_chains_[idx].getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

bool B3wWholeBodyIkPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}


  bool B3wWholeBodyIkPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
					      const std::vector<double>& ik_seed_state,
					      double timeout,
					      const std::vector<double>& consistency_limits,
					      std::vector<double>& solution,
					      const IKCallbackFn& solution_callback,
					      moveit_msgs::MoveItErrorCodes& error_code,
					      const kinematics::KinematicsQueryOptions& options,
					      const moveit::core::RobotState* context_state) const
{
  ROS_ERROR_STREAM_NAMED("temp", "searchPositionIK5");
  for(const auto& pose:ik_poses)
    ROS_ERROR_STREAM_NAMED("temp", "x: "<<pose.position.x<<" y: "<<pose.position.y<<" z: "<<pose.position.z);
  ros::WallTime n1 = ros::WallTime::now();
  if (!active_)
  {
    ROS_ERROR_NAMED("kdl", "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_poses.size()!=ik_chain_infos_.size())
  {
    ROS_ERROR_STREAM_NAMED("kdl", "ik_poses must have size" << ik_chain_infos_.size() <<"instead of size " << ik_poses.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  
  if (ik_seed_state.size() != joint_model_indexs_.size())
  {
    ROS_ERROR_STREAM_NAMED("kdl", "Seed state must have size " << joint_model_indexs_.size() << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != joint_model_indexs_.size())
  {
    ROS_ERROR_STREAM_NAMED("kdl", "Consistency limits be empty or must have size " << joint_model_indexs_.size() << " instead of size "
                                                                                   << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  solution.resize(joint_model_indexs_.size());

  unsigned int counter(0);
  while(1){
    for(unsigned int idx=0;idx<ik_chain_infos_.size();idx++){
      KDL::JntArray jnt_seed_state(ik_chain_infos_[idx].joint_names.size());
      KDL::JntArray jnt_pos_in(ik_chain_infos_[idx].joint_names.size());
      KDL::JntArray jnt_pos_out(ik_chain_infos_[idx].joint_names.size());
      std::vector<double> this_consistency_limits;
      KDL::ChainFkSolverPos_recursive fk_solver(kdl_chains_[idx]);
      KDL::ChainIkSolverVel_pinv_mimic ik_solver_vel(kdl_chains_[idx],
						     num_mimic_joints_[idx],
						     /*num redundant*/0,
						     position_ik_);
      KDL::ChainIkSolverPos_NR_JL_Mimic ik_solver_pos(kdl_chains_[idx],
						      joint_mins_[idx],
						      joint_maxs_[idx],
						      fk_solver,
						      ik_solver_vel,
						      max_solver_iterations_,
						      epsilon_,
						      position_ik_);
      ik_solver_vel.setMimicJoints(mimic_jointss_[idx]);
      ik_solver_pos.setMimicJoints(mimic_jointss_[idx]);

      //redundant TODO
      if ((redundant_joint_indices_.size() > 0) && !ik_solver_vel.setRedundantJointsMapIndex(redundant_joints_map_indexs_[idx]))
	{
	  ROS_ERROR_NAMED("kdl", "Could not set redundant joints");
	  return false;
	}

      if (options.lock_redundant_joints)
	{
	  ik_solver_vel.lockRedundantJoints();
	}

      KDL::Frame pose_desired;
      tf::poseMsgToKDL(ik_poses[idx], pose_desired);

      ROS_DEBUG_STREAM_NAMED("kdl", "searchPositionIK2: Position request pose is "
			     << ik_poses[idx].position.x << " " << ik_poses[idx].position.y << " " << ik_poses[idx].position.z
			     << " " << ik_poses[idx].orientation.x << " " << ik_poses[idx].orientation.y << " "
			     << ik_poses[idx].orientation.z << " " << ik_poses[idx].orientation.w);
      // Do the IK
      for (unsigned int i = 0; i < ik_chain_infos_[idx].joint_names.size(); i++)
	jnt_seed_state(i) = ik_seed_state[joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[i])];
      if(!consistency_limits.empty()){
	for (unsigned int i = 0; i < ik_chain_infos_[idx].joint_names.size(); i++)
	  this_consistency_limits.push_back(consistency_limits[joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[i])]);
      }
      jnt_pos_in = jnt_seed_state;

      while (1)
	{
	  //    ROS_DEBUG_NAMED("kdl","Iteration: %d, time: %f, Timeout:
	  //    %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
	  counter++;
	  if (timedOut(n1, timeout))
	    {
	      ROS_DEBUG_NAMED("kdl", "IK timed out");
	      ROS_ERROR_NAMED("temp", "IK timed out");
	      error_code.val = error_code.TIMED_OUT;
	      ik_solver_vel.unlockRedundantJoints();
	      return false;
	    }
	  ROS_ERROR_NAMED("temp", "before");
	  int ik_valid = ik_solver_pos.CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);
	  ROS_ERROR_NAMED("temp", "IK valid: %d", ik_valid);
	  for(const auto& limit:this_consistency_limits)
	    ROS_ERROR_STREAM_NAMED("temp", limit);
	  ROS_DEBUG_NAMED("kdl", "IK valid: %d", ik_valid);
	  if (!this_consistency_limits.empty())
	    {
	      getRandomConfiguration(idx,jnt_seed_state, this_consistency_limits, jnt_pos_in, options.lock_redundant_joints);
	      if ((ik_valid < 0 && !options.return_approximate_solution) ||
		  !checkConsistency(jnt_seed_state, this_consistency_limits, jnt_pos_out))
		{
		  ROS_ERROR_NAMED("temp", "Could not find IK solution: does not match consistency limits");
		  ROS_DEBUG_NAMED("kdl", "Could not find IK solution: does not match consistency limits");
		  continue;
		}
	    }
	  else
	    {
	      getRandomConfiguration(idx,jnt_pos_in, options.lock_redundant_joints);
	      ROS_DEBUG_NAMED("kdl", "New random configuration");
	      for (unsigned int j = 0; j < jnt_pos_in.rows(); j++)
		ROS_DEBUG_NAMED("kdl", "%d %f", j, jnt_pos_in(j));

	      if (ik_valid < 0 && !options.return_approximate_solution)
		{
		  ROS_DEBUG_NAMED("kdl", "Could not find IK solution");
		  ROS_ERROR_NAMED("temp", "Could not find IK solution");
		  continue;
		}
	    }
	  ROS_ERROR_NAMED("temp", "Found IK solution");
	  ROS_DEBUG_NAMED("kdl", "Found IK solution");
	  for (unsigned int j = 0; j < ik_chain_infos_[idx].joint_names.size(); j++){
	    solution[joint_model_indexs_.at(ik_chain_infos_[idx].joint_names[j])] = jnt_pos_out(j);
	  }
	  ik_solver_vel.unlockRedundantJoints();
	  break;
	}
    }

    if (!solution_callback.empty())
      solution_callback(ik_poses[0], solution, error_code);//pose1つしか取れないぞ
    else
      error_code.val = error_code.SUCCESS;
  
    if (error_code.val == error_code.SUCCESS)
      {
	ROS_DEBUG_STREAM_NAMED("kdl", "Solved after " << counter << " iterations");
	ROS_ERROR_STREAM_NAMED("temp", "Solved");
	//ik_solver_vel.unlockRedundantJoints();
	return true;
      }
  }
  ROS_DEBUG_NAMED("kdl", "An IK that satisifes the constraints and is collision free could not be found");
  error_code.val = error_code.NO_IK_SOLUTION;
  //ik_solver_vel.unlockRedundantJoints();
  ROS_ERROR_NAMED("temp", "IK timed out");
  return false;
}

  bool B3wWholeBodyIkPlugin::getPositionFK(const std::vector<std::string>& link_names,
					   const std::vector<double>& joint_angles,
					   std::vector<geometry_msgs::Pose>& poses) const
  {
    ROS_ERROR_NAMED("ik", "searchPositionFK");
    ros::WallTime n1 = ros::WallTime::now();
    if (!active_)
      {
	ROS_ERROR_NAMED("kdl", "kinematics not active");
	return false;
      }
    poses.resize(link_names.size());
    if (joint_angles.size() != joint_model_indexs_.size())
      {
	ROS_ERROR_STREAM_NAMED("kdl", "Joint angles vector must have size: "<<joint_model_indexs_.size());
	return false;
      }

    bool valid = true;
    for(int j=0;j<link_names.size();j++){
      unsigned int idx=0;
      for(idx=0;idx<fk_chain_infos_.size();idx++){
	bool found=false;
	for(const auto& linkname:fk_chain_infos_[idx].link_names)
	  if(linkname==link_names[j])
	    found=true;
	if(found)
	  break;
      }
    
      KDL::Frame p_out;
      geometry_msgs::PoseStamped pose;
      tf::Stamped<tf::Pose> tf_pose;
    
      KDL::JntArray jnt_pos_in(fk_chain_infos_[idx].joint_names.size());
      for (unsigned int i = 0; i < jnt_pos_in.rows(); i++)
	{
	  jnt_pos_in(i) = joint_angles[joint_model_indexs_.at(fk_chain_infos_[idx].joint_names[i])];
	}
    
      KDL::ChainFkSolverPos_recursive fk_solver(kdl_chains_[idx]);    

      ROS_DEBUG_NAMED("kdl", "End effector index: %d", getKDLSegmentIndex(idx,link_names[j]));
      if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(idx,link_names[j])) >= 0)
	{
	  tf::poseKDLToMsg(p_out, poses[j]);
	}
      else
	{
	  ROS_ERROR_NAMED("kdl", "Could not compute FK for %s", link_names[j].c_str());
	  valid = false;
	}
    }
    return valid;
  }

  const std::vector<std::string>& B3wWholeBodyIkPlugin::getJointNames() const
  {
    return jointnames_;
  }
  
  const std::vector<std::string>& B3wWholeBodyIkPlugin::getLinkNames() const
  {
    return linknames_;
  }
  
  bool B3wWholeBodyIkPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg, std::string* error_text_out) const{
    return true;
  }
  
}  // namespace
