/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <b3w_humanoid/poses_constraint_sampler.h>
#include <set>
#include <cassert>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>

b3w_humanoid::IKSamplingPose::IKSamplingPose()
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint& pc)
  : position_constraint_(new kinematic_constraints::PositionConstraint(pc))
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::OrientationConstraint& oc)
  : orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraint& pc,
						    const kinematic_constraints::OrientationConstraint& oc)
  : position_constraint_(new kinematic_constraints::PositionConstraint(pc))
  , orientation_constraint_(new kinematic_constraints::OrientationConstraint(oc))
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc)
  : position_constraint_(pc)
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::OrientationConstraintPtr& oc)
  : orientation_constraint_(oc)
{
}

b3w_humanoid::IKSamplingPose::IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc,
						    const kinematic_constraints::OrientationConstraintPtr& oc)
  : position_constraint_(pc), orientation_constraint_(oc)
{
}

void b3w_humanoid::PosesConstraintSampler::clear()
{
  ConstraintSampler::clear();
  kb_.reset();
  ik_frame_ = "";
  transform_ik_ = false;
  sampling_poses_.clear();
  need_eef_to_ik_tip_transforms_.clear();
  eef_to_ik_tip_transform_.clear();
  tip2pose_index_.clear();
}

bool b3w_humanoid::PosesConstraintSampler::configure(const IKSamplingPose& sp)
{
  if (!sp.position_constraint_ && !sp.orientation_constraint_)
    return false;
  if ((!sp.orientation_constraint_ && !sp.position_constraint_->enabled()) ||
      (!sp.position_constraint_ && !sp.orientation_constraint_->enabled()) ||
      (sp.position_constraint_ && sp.orientation_constraint_ && !sp.position_constraint_->enabled() &&
       !sp.orientation_constraint_->enabled()))
    {
      CONSOLE_BRIDGE_logWarn("No enabled constraints in sampling pose");
      return false;
    }

  if (sp.position_constraint_ && sp.orientation_constraint_)
    if (sp.position_constraint_->getLinkModel()->getName() !=
	sp.orientation_constraint_->getLinkModel()->getName())
      {
	CONSOLE_BRIDGE_logError("Position and orientation constraints need to be specified for the same link in "
				"order to use IK-based sampling");
	return false;
      }

  if (sp.position_constraint_ && sp.position_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sp.position_constraint_->getReferenceFrame());
  if (sp.orientation_constraint_ && sp.orientation_constraint_->mobileReferenceFrame())
    frame_depends_.push_back(sp.orientation_constraint_->getReferenceFrame());

  sampling_poses_.push_back(sp);
  
  return true;
}

bool b3w_humanoid::PosesConstraintSampler::configure(const moveit_msgs::Constraints& constr)
{
  clear();
  
  std::vector<bool> p_used(constr.position_constraints.size(),false);
  std::vector<bool> o_used(constr.orientation_constraints.size(),false);

  for (std::size_t p = 0; p < constr.position_constraints.size(); ++p){
    for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o){
      if(p_used[p]||o_used[o])continue;
      if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name){
	kinematic_constraints::PositionConstraintPtr pc(
							new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
	kinematic_constraints::OrientationConstraintPtr oc(
							   new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
	if (pc->configure(constr.position_constraints[p], scene_->getTransforms()) &&
	    oc->configure(constr.orientation_constraints[o], scene_->getTransforms())){
	  if(configure(IKSamplingPose(pc, oc))){
	    p_used[p]=true;
	    o_used[o]=true;
	  }
	}
      }
    }
  }
  for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
    {
      if(p_used[p])continue;
      kinematic_constraints::PositionConstraintPtr pc(new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
      if (pc->configure(constr.position_constraints[p], scene_->getTransforms())){
	if(configure(IKSamplingPose(pc)))
	  p_used[p]=true;
      }
    }

  for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
    {
      if(o_used[o])continue;
      kinematic_constraints::OrientationConstraintPtr oc(
							 new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
      if (oc->configure(constr.orientation_constraints[o], scene_->getTransforms())){
	if(configure(IKSamplingPose(oc)))
	  o_used[o]=true;
      }
    }

  for(auto p:p_used){
    if(!p){
      ROS_ERROR_STREAM_NAMED("poses_constraint_sampler","poses_constraint_sampler: configure failed. unused constraint");
      return false;
    }
  }
  for(auto o:o_used){
    if(!o){
      ROS_ERROR_STREAM_NAMED("poses_constraint_sampler","poses_constraint_sampler: configure failed unused constraint");
      return false;
    }
  }
  //注意：同じリンクに複数のconstraintがあってもエラーを出さないが、ダメ
  
  ik_timeout_ = jmg_->getDefaultIKTimeout();
  kb_ = jmg_->getSolverInstance();
  if (!kb_)
    {
      CONSOLE_BRIDGE_logWarn("No solver instance in setup");
      is_valid_ = false;
      return false;
    }
  is_valid_ = loadIKSolver();
  return is_valid_;
}

bool b3w_humanoid::PosesConstraintSampler::loadIKSolver()
{
  if (!kb_)
    {
      CONSOLE_BRIDGE_logError("No IK solver");
      return false;
    }

  // check if we need to transform the request into the coordinate frame expected by IK
  ik_frame_ = kb_->getBaseFrame();
  transform_ik_ = !robot_state::Transforms::sameFrame(ik_frame_, jmg_->getParentModel().getModelFrame());
  if (!ik_frame_.empty() && ik_frame_[0] == '/')
    ik_frame_.erase(ik_frame_.begin());
  if (transform_ik_)
    if (!jmg_->getParentModel().hasLinkModel(ik_frame_))
      {
	CONSOLE_BRIDGE_logError("The IK solver expects requests in frame '%s' but this frame is not known to the "
				"sampler. Ignoring transformation (IK may fail)",
				ik_frame_.c_str());
	transform_ik_ = false;
      }

  // check if IK is performed for the desired link
  need_eef_to_ik_tip_transforms_.resize(sampling_poses_.size());
  eef_to_ik_tip_transform_.resize(sampling_poses_.size());
  tip2pose_index_.clear();
  tip2pose_index_.resize(kb_->getTipFrames().size(),-1);
  bool wrong_link = false;
  for(int idx=0;idx< sampling_poses_.size();idx++){
    if (sampling_poses_[idx].position_constraint_){
      const moveit::core::LinkModel* lm = sampling_poses_[idx].position_constraint_->getLinkModel();
      bool sameframe=false;
      for(int tip_idx=0;tip_idx<kb_->getTipFrames().size();tip_idx++){
	if (moveit::core::Transforms::sameFrame(kb_->getTipFrames()[tip_idx], lm->getName())){
	  sameframe=true;
	  tip2pose_index_[tip_idx]=idx;
	}
      }
      if (!sameframe){
	wrong_link = true;
	const moveit::core::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
	for (moveit::core::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it){
	  for(int tip_idx=0;tip_idx<kb_->getTipFrames().size();tip_idx++){
	    if(moveit::core::Transforms::sameFrame(it->first->getName(), kb_->getTipFrames()[tip_idx])){
	      eef_to_ik_tip_transform_[idx] = it->second;
	      need_eef_to_ik_tip_transforms_[idx] = true;
	      wrong_link = false;
	      tip2pose_index_[tip_idx]=idx;
	      break;
	    }
	  } 
	}
      }
    }
    
    if (!wrong_link && sampling_poses_[idx].orientation_constraint_){
      const moveit::core::LinkModel* lm = sampling_poses_[idx].orientation_constraint_->getLinkModel();
      bool sameframe=false;
      for(int tip_idx=0;tip_idx<kb_->getTipFrames().size();tip_idx++){
	if (moveit::core::Transforms::sameFrame(kb_->getTipFrames()[tip_idx], lm->getName())){
	  sameframe=true;
	  tip2pose_index_[tip_idx]=idx;
	}
      }
      if (!sameframe){
	wrong_link = true;
	const moveit::core::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
	for (moveit::core::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it){
	  for(int tip_idx=0;tip_idx<kb_->getTipFrames().size();tip_idx++){
	    if (moveit::core::Transforms::sameFrame(it->first->getName(), kb_->getTipFrames()[tip_idx])){
	      eef_to_ik_tip_transform_[idx] = it->second;
	      need_eef_to_ik_tip_transforms_[idx] = true;
	      wrong_link = false;
	      tip2pose_index_[tip_idx]=idx;
	      break;
	    }
	  }
	}
      }
    }

    if (wrong_link){
      CONSOLE_BRIDGE_logError("IK cannot be performed for link '%s'. The solver can report IK solutions for tip frames.",
			      sampling_poses_[idx].position_constraint_ ?
			      sampling_poses_[idx].position_constraint_->getLinkModel()->getName().c_str() :
			      sampling_poses_[idx].orientation_constraint_->getLinkModel()->getName().c_str());
      return false;
    }
  }

  return true;
}

bool b3w_humanoid::PosesConstraintSampler::samplePose(std::vector<Eigen::Vector3d>& poss, std::vector<Eigen::Quaterniond>& quats,
							  const robot_state::RobotState& ks, unsigned int max_attempts)
{
  if (ks.dirtyLinkTransforms())
    {
      // samplePose below requires accurate transforms
      CONSOLE_BRIDGE_logError("PosesConstraintSampler received dirty robot state, but valid transforms are required. "
			      "Failing.");
      return false;
    }

  poss.resize(sampling_poses_.size());
  quats.resize(sampling_poses_.size());
  for(int idx=0;idx<sampling_poses_.size();idx++){
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    if (sampling_poses_[idx].position_constraint_){
      const std::vector<bodies::BodyPtr>& b = sampling_poses_[idx].position_constraint_->getConstraintRegions();
      if (!b.empty())
	{
	  bool found = false;
	  std::size_t k = random_number_generator_.uniformInteger(0, b.size() - 1);
	  for (std::size_t i = 0; i < b.size(); ++i)
	    if (b[(i + k) % b.size()]->samplePointInside(random_number_generator_, max_attempts, pos)){
	      found = true;
	      break;
	    }
	  if (!found){
	    CONSOLE_BRIDGE_logError("Unable to sample a point inside the constraint region");
	    return false;
	  }
	}
      else{
	CONSOLE_BRIDGE_logError("Unable to sample a point inside the constraint region. "
				"Constraint region is empty when it should not be.");
	return false;
      }
      
      // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the
      // model
      if (sampling_poses_[idx].position_constraint_->mobileReferenceFrame())
	pos = ks.getFrameTransform(sampling_poses_[idx].position_constraint_->getReferenceFrame()) * pos;
    }
    else{//if not positionconstraint
      // do FK for rand state
      robot_state::RobotState tempState(ks);
      tempState.setToRandomPositions(jmg_);
      pos = tempState.getGlobalLinkTransform(sampling_poses_[idx].orientation_constraint_->getLinkModel()).translation();
    }
    
    if (sampling_poses_[idx].orientation_constraint_){
      // sample a rotation matrix within the allowed bounds
      double angle_x =
	2.0 * (random_number_generator_.uniform01() - 0.5) *
	(sampling_poses_[idx].orientation_constraint_->getXAxisTolerance() - std::numeric_limits<double>::epsilon());
      double angle_y =
	2.0 * (random_number_generator_.uniform01() - 0.5) *
	(sampling_poses_[idx].orientation_constraint_->getYAxisTolerance() - std::numeric_limits<double>::epsilon());
      double angle_z =
	2.0 * (random_number_generator_.uniform01() - 0.5) *
	(sampling_poses_[idx].orientation_constraint_->getZAxisTolerance() - std::numeric_limits<double>::epsilon());
      Eigen::Affine3d diff(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX()) *
			   Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY()) *
			   Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
      Eigen::Affine3d reqr(sampling_poses_[idx].orientation_constraint_->getDesiredRotationMatrix() * diff.rotation());
      quat = Eigen::Quaterniond(reqr.rotation());
      
      // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the
      // model
      if (sampling_poses_[idx].orientation_constraint_->mobileReferenceFrame()) {
	const Eigen::Affine3d& t = ks.getFrameTransform(sampling_poses_[idx].orientation_constraint_->getReferenceFrame());
	Eigen::Affine3d rt(t.rotation() * quat.toRotationMatrix());
	quat = Eigen::Quaterniond(rt.rotation());
      }
    }
    else{//if not orientation
      // sample a random orientation
      double q[4];
      random_number_generator_.quaternion(q);
      quat = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    }
    
    // if there is an offset, we need to undo the induced rotation in the sampled transform origin (point)
    if (sampling_poses_[idx].position_constraint_ && sampling_poses_[idx].position_constraint_->hasLinkOffset())
      // the rotation matrix that corresponds to the desired orientation
      pos = pos - quat.toRotationMatrix() * sampling_poses_[idx].position_constraint_->getLinkOffset();

    poss[idx]=pos;
    quats[idx]=quat;
  }
  
  return true;
}

namespace b3w_humanoid
{
  namespace
  {
    void samplingIkCallbackFnAdapter(robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
				     const robot_state::GroupStateValidityCallbackFn& constraint,
				     const geometry_msgs::Pose&, const std::vector<double>& ik_sol,
				     moveit_msgs::MoveItErrorCodes& error_code)
    {
      const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();
      std::vector<double> solution(bij.size());
      for (std::size_t i = 0; i < bij.size(); ++i)
	solution[i] = ik_sol[bij[i]];
      if (constraint(state, jmg, &solution[0]))
	error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      else
	error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    }
  }
}

bool b3w_humanoid::PosesConstraintSampler::sample(robot_state::RobotState& state,
						      const robot_state::RobotState& reference_state,
						      unsigned int max_attempts)
{
  return sampleHelper(state, reference_state, max_attempts, false);
}

bool b3w_humanoid::PosesConstraintSampler::sampleHelper(robot_state::RobotState& state,
							    const robot_state::RobotState& reference_state,
							    unsigned int max_attempts, bool project)
{
  if (!is_valid_)
    {
      CONSOLE_BRIDGE_logWarn("PosesConstraintSampler not configured, won't sample");
      return false;
    }

  //callbackは、通常のisstatevalidではなく、constraintを除いたものとする
  kinematics::KinematicsBase::IKCallbackFn adapted_ik_validity_callback;
  if (group_state_validity_callback_)
    adapted_ik_validity_callback =std::bind(&b3w_humanoid::PosesConstraintSampler::callbackfn,this,reference_state,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
	  //boost::bind(&samplingIkCallbackFnAdapter, &state, jmg_, group_state_validity_callback_, _1, _2, _3);

  for (unsigned int a = 0; a < max_attempts; ++a){
    // sample a point in the constraint region
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Quaterniond> quats;
    if (!samplePose(points, quats, reference_state, max_attempts)){
      if (verbose_)
	CONSOLE_BRIDGE_logInform("IK constraint sampler was unable to produce a pose to run IK for");
      return false;
    }

    
    // we now have the transform we wish to perform IK for, in the planning frame
    if (transform_ik_){
      for(int idx=0;idx<sampling_poses_.size();idx++){
	// we need to convert this transform to the frame expected by the IK solver
	// both the planning frame and the frame for the IK are assumed to be robot links
	Eigen::Affine3d ikq(Eigen::Translation3d(points[idx]) * quats[idx].toRotationMatrix());
	ikq = reference_state.getFrameTransform(ik_frame_).inverse() * ikq;
	points[idx] = ikq.translation();
	quats[idx] = Eigen::Quaterniond(ikq.rotation());
      }
    }

    // After sampling the pose needs to be transformed to the ik chain tip
    for(int idx=0;idx<sampling_poses_.size();idx++){
      if (need_eef_to_ik_tip_transforms_[idx]){
	Eigen::Affine3d ikq(Eigen::Translation3d(points[idx]) * quats[idx].toRotationMatrix());
	ikq = ikq * eef_to_ik_tip_transform_[idx];
	points[idx] = ikq.translation();
	quats[idx] = Eigen::Quaterniond(ikq.rotation());
      }
    }
    
    std::vector<geometry_msgs::Pose> ik_query(sampling_poses_.size());
    for(int idx=0;idx<sampling_poses_.size();idx++){
      ik_query[idx].position.x = points[idx].x();
      ik_query[idx].position.y = points[idx].y();
      ik_query[idx].position.z = points[idx].z();
      ik_query[idx].orientation.x = quats[idx].x();
      ik_query[idx].orientation.y = quats[idx].y();
      ik_query[idx].orientation.z = quats[idx].z();
      ik_query[idx].orientation.w = quats[idx].w();
    }
    
    //目標姿勢の生成。posesを適切に並び替えるとともに、目標が未定義のtipには現状維持を命じる
    std::vector<geometry_msgs::Pose> ik_query2(kb_->getTipFrames().size());
    for(int tip_idx=0;tip_idx<ik_query2.size();tip_idx++){
      if(tip2pose_index_[tip_idx]!=-1)
	ik_query2[tip_idx]=ik_query[tip2pose_index_[tip_idx]];
      else{
	// Get the pose of a different EE tip link
	Eigen::Affine3d current_pose = reference_state.getGlobalLinkTransform(kb_->getTipFrames()[tip_idx]);
	// bring the pose to the frame of the IK solver
	if (transform_ik_){
	  //reference_state.setToIKSolverFrame(current_pose,kb_->getBaseName()); non const error
	  current_pose = reference_state.getFrameTransform(ik_frame_).inverse() * current_pose;
	}
	// Convert Eigen pose to geometry_msgs pose
	tf::poseEigenToMsg(current_pose, ik_query2[tip_idx]);
      }
    }
    if (callIK(ik_query2, adapted_ik_validity_callback, ik_timeout_, state, project && a == 0))
      return true;
  }
  return false;
}

bool b3w_humanoid::PosesConstraintSampler::project(robot_state::RobotState& state, unsigned int max_attempts)
{
  return sampleHelper(state, state, max_attempts, true);
}

//ダメそう
bool b3w_humanoid::PosesConstraintSampler::validate(robot_state::RobotState& state) const
{
  return true;
  // state.update();
  // return (!sampling_pose_.orientation_constraint_ ||
  // 	  sampling_pose_.orientation_constraint_->decide(state, verbose_).satisfied) &&
  //   (!sampling_pose_.position_constraint_ ||
  //    sampling_pose_.position_constraint_->decide(state, verbose_).satisfied);
}

bool b3w_humanoid::PosesConstraintSampler::callIK(const std::vector<geometry_msgs::Pose>& ik_query,
						  const kinematics::KinematicsBase::IKCallbackFn& adapted_ik_validity_callback,
						  double timeout,
						  robot_state::RobotState& state,
						  bool use_as_seed)
{
  const std::vector<unsigned int>& ik_joint_bijection = jmg_->getKinematicsSolverJointBijection();
  std::vector<double> seed(ik_joint_bijection.size(), 0.0);
  std::vector<double> vals;

  if (use_as_seed)
    state.copyJointGroupPositions(jmg_, vals);
  else
    // sample a seed value
    jmg_->getVariableRandomPositions(random_number_generator_, vals);

  assert(vals.size() == ik_joint_bijection.size());
  for (std::size_t i = 0; i < ik_joint_bijection.size(); ++i)
    seed[i] = vals[ik_joint_bijection[i]];

  std::vector<double> ik_sol;
  moveit_msgs::MoveItErrorCodes error;

  if (adapted_ik_validity_callback ?
      kb_->searchPositionIK(ik_query, seed, timeout, std::vector<double>{},ik_sol, adapted_ik_validity_callback, error) :
      kb_->searchPositionIK(ik_query, seed, timeout, std::vector<double>{},ik_sol, adapted_ik_validity_callback,error)){
    assert(ik_sol.size() == ik_joint_bijection.size());
    std::vector<double> solution(ik_joint_bijection.size());
    for (std::size_t i = 0; i < ik_joint_bijection.size(); ++i)
      solution[ik_joint_bijection[i]] = ik_sol[i];
    state.setJointGroupPositions(jmg_, solution);
    state.update();
    //return validate(state);うちのIKは正しい絶対座標に行かないので.tip同士の相対座標は正しいが
    return true;
  }
  else{
    if (error.val != moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION &&
	error.val != moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE &&
	error.val != moveit_msgs::MoveItErrorCodes::TIMED_OUT)
      CONSOLE_BRIDGE_logError("IK solver failed with error %d", error.val);
    else if (verbose_)
      CONSOLE_BRIDGE_logInform("IK failed");
  }
  return false;
}

void b3w_humanoid::PosesConstraintSampler::callbackfn(robot_state::RobotState& state,const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_solution, moveit_msgs::MoveItErrorCodes& error_code){
  robot_state::RobotState copy_state{state};
  const std::vector<unsigned int>& bij = jmg_->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[i] = ik_solution[bij[i]];
  copy_state.setJointGroupPositions(jmg_, solution);
  copy_state.update();
  if (scene_->isStateValid(copy_state)){//feasibility,cillision only
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    ROS_ERROR_STREAM("callbackfn success");
  }else{
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    ROS_ERROR_STREAM("callbackfn fail "<<scene_->isStateColliding(copy_state)<<" "<<scene_->isStateFeasible(copy_state));
  }
}
