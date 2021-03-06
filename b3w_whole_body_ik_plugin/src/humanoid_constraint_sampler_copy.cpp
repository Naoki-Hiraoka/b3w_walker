/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman
*/

#include <set>
#include <cassert>
#include <eigen_conversions/eigen_msg.h>
#include <boost/format.hpp>
#include <b3w_whole_body_ik_plugin/humanoid_constraint_sampler.h>

namespace b3w_whole_body_ik_plugin
{
bool HumanoidConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
  clear();
  ROS_ERROR_STREAM_NAMED("temp","Configur");

  // Humanoid custom constraints: define here --------------------
  moveit_msgs::JointConstraint jc1;

  // construct the joint constraints
  std::vector<kinematic_constraints::JointConstraint> jc;
  for (std::size_t i = 0 ; i < constr.joint_constraints.size() ; ++i)
  {
    kinematic_constraints::JointConstraint j(scene_->getRobotModel());
    if (j.configure(constr.joint_constraints[i]))
      jc.push_back(j);
  }

  // If joint constriaints are provided, configure them. otherwise we will use regular random joint sampling
  if (jc.empty())
  {
    sampler_name_ = "No_Joint_Constraints";
    ROS_ERROR_STREAM("No joint constraints passed to humanoid constraint sampler");
    is_valid_ = true; // set as configured
  }
  else
  {
    sampler_name_ = "Has_Joint_Constraints";

    if (!configureJoint(jc))
      return false;

    is_valid_ = true; // set as configured
  }

  
  return true;
}

void HumanoidConstraintSampler::loadVisualTools()
{
  return;
}


  bool HumanoidConstraintSampler::configureJoint(const std::vector<kinematic_constraints::JointConstraint> &jc)//必要
{
  clear();
  if (!jmg_)
  {
    logError("NULL planning group specified for constraint sampler");
    return false;
  }
  // find and keep the constraints that operate on the group we sample
  // also keep bounds for joints for convenience
  std::map<std::string, JointInfo> bound_data;
  for (std::size_t i = 0 ; i < jc.size() ; ++i)
  {
    // Check that joint constraint is enabled
    if (!jc[i].enabled())
      continue;
    
    // Check that joint constraint has valid joint model
    const robot_model::JointModel *jm = jc[i].getJointModel();
    if (!jmg_->hasJointModel(jm->getName()))
      continue;

    // Get the bounds of this variable
    const robot_model::VariableBounds& joint_bounds = jm->getVariableBounds(jc[i].getJointVariableName());

    // Populate the joint info object
    JointInfo ji;
    // Check if this variable already has bounds set (for some reason_
    std::map<std::string, JointInfo>::iterator it = bound_data.find(jc[i].getJointVariableName());
    if (it != bound_data.end())
      ji = it->second;
    else
      ji.index_ = jmg_->getVariableGroupIndex(jc[i].getJointVariableName()); // copy the index of the variable with respect to the joint model group

    // Attempt to tighten the variables bounds if applicable from the constraint
    ji.potentiallyAdjustMinMaxBounds(std::max(joint_bounds.min_position_, jc[i].getDesiredJointPosition() - jc[i].getJointToleranceBelow()),
                                     std::min(joint_bounds.max_position_, jc[i].getDesiredJointPosition() + jc[i].getJointToleranceAbove()));
    // Error check
    if (ji.min_bound_ > ji.max_bound_ + std::numeric_limits<double>::epsilon())
    {
      std::stringstream cs; jc[i].print(cs);
      logError("The constraints for joint '%s' have no possible values for the joint: min_bound: %g, max_bound: %g. Failing.\n",
               jm->getName().c_str(), ji.min_bound_, ji.max_bound_);
      clear();
      return false;
    }
    ROS_ERROR_STREAM("min: "<<ji.min_bound_<<" max: "<<ji.max_bound_);
    // Save this new joint info
    bound_data[jc[i].getJointVariableName()] = ji;
  }

  for (std::map<std::string, JointInfo>::iterator it = bound_data.begin(); it != bound_data.end(); ++it)
    bounds_.push_back(it->second);
  // get a separate list of joints that are not bounded; we will sample these randomly
  const std::vector<const robot_model::JointModel*> &joints = jmg_->getJointModels();
  for (std::size_t i = 0 ; i < joints.size() ; ++i)
    if (bound_data.find(joints[i]->getName()) == bound_data.end() &&
        joints[i]->getVariableCount() > 0 &&
        joints[i]->getMimic() == NULL)
    {
      // check if all the vars of the joint are found in bound_data instead
      const std::vector<std::string> &vars = joints[i]->getVariableNames();
      if (vars.size() > 1)
      {
        bool all_found = true;
        for (std::size_t j = 0 ; j < vars.size() ; ++j)
          if (bound_data.find(vars[j]) == bound_data.end())
          {
            all_found = false;
            break;
          }
        if (all_found)
          continue;
      }
      unbounded_.push_back(joints[i]);
      // Get the first variable name of this joint and find its index position in the planning group
      uindex_.push_back(jmg_->getVariableGroupIndex(vars[0]));
      //logInform("Adding variable index %d for joint index %d",jmg_->getVariableGroupIndex(vars[0]), i);
    }

  values_.resize(jmg_->getVariableCount());
  
  return true;
}

bool HumanoidConstraintSampler::sample(robot_state::RobotState &robot_state, const robot_state::RobotState & /* reference_state */,
                                        unsigned int max_attempts)
{
  ROS_ERROR_STREAM_NAMED("temp","Sample");

  if (!jmg_)
    logError("no joint model group loaded");
  
  if (!is_valid_){
    logWarn("HumanoidConstraintSampler not configured, won't sample");
    return false;
  }

  max_attempts = 100000; // TODO this might be a bad hack
  
  for (std::size_t attempt = 0; attempt < max_attempts; ++attempt){
    if (verbose_)
      logInform("Sampling attempt number %d for group %s", attempt, jmg_->getName().c_str() );
    
    if (!ros::ok())
      return false;
    // Decide how to sample the joints
    bool use_constraint_sampling = false;
    
    if (bounds_.size() > 0){
      if (verbose_)
        ROS_INFO_STREAM_NAMED("sampler","Sampling joints using joint constraints");
      use_constraint_sampling = true;
      // Calculate random position of robot
      // \todo: don't sample virtual joint orientation and the legs to save time
      if (!sampleJoints(robot_state))
	{
	  logError("Unable to sample joints");
	  return false;
	}
      robot_state.update();
    }
    else{
      //ROS_INFO_STREAM_NAMED("temp","Sampling joints using robot state random variables");
      // Generate random state for leg only
      robot_state.setToRandomPositions();
      // Update only the virtual joint and the leg we just updated
      robot_state.update();
    }

    bool check_verbose = false;
    if (!scene_->isStateValid(robot_state, "", check_verbose)){ // second argument is what planning group to collision check, "" is everything
      if (verbose_)
	{
	  ROS_ERROR_STREAM_NAMED("sampler","Pose not valid (self or enviornment collision)");
	}
      continue;
    }
    ROS_ERROR_STREAM_NAMED("temp","passed");
    return true;
  }
  ROS_ERROR_STREAM_NAMED("temp","max iteration");
  return false;
}

bool HumanoidConstraintSampler::sampleJoints(robot_state::RobotState &robot_state)
{
  // sample the unbounded joints first (in case some joint varipables are bounded)
  std::vector<double> v;
  for (std::size_t i = 0 ; i < unbounded_.size() ; ++i)
  {
    v.resize(unbounded_[i]->getVariableCount());
    unbounded_[i]->getVariableRandomPositions(random_number_generator_, &v[0]);

    for (std::size_t j = 0 ; j < v.size() ; ++j)
    {
      values_[uindex_[i] + j] = v[j];
    }
  }

  // enforce the constraints for the constrained components (could be all of them)
  for (std::size_t i = 0 ; i < bounds_.size() ; ++i){
    values_[bounds_[i].index_] = random_number_generator_.uniformReal(bounds_[i].min_bound_, bounds_[i].max_bound_);
  }
  robot_state.setJointGroupPositions(jmg_, values_);
  return true;
}

bool HumanoidConstraintSampler::project(robot_state::RobotState &robot_state,
                                         unsigned int max_attempts)
{
  return sample(robot_state, robot_state, max_attempts);
}

void HumanoidConstraintSampler::clear()
{
  ConstraintSampler::clear();
  bounds_.clear();
  unbounded_.clear();
  uindex_.clear();
  values_.clear();
}

void HumanoidConstraintSampler::setVerbose(bool verbose)
{
  // Load visual tools
  if (verbose)
  {
    loadVisualTools();
  }
}


} //namespace
