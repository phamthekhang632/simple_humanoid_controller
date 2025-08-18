#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <chrono>

SimpleHumanoidController::SimpleHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);

  // CoM
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  postureTask->stiffness(1);

  // End Effector
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), "extensions/simple_humanoid_controller/task.yaml");
  solver().addTask(leftHandTask_);

  // End effectors
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/left_task.yaml");
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/right_task.yaml");
  solver().addTask(rightHandTask_);

  // Save initial poses
  leftHandInitPose_ = leftHandTask_->get_ef_pose();
  rightHandInitPose_ = rightHandTask_->get_ef_pose();

  stateStartTime_ = std::chrono::steady_clock::now();

  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  // // Get the current objective
  // auto pt = leftHandTask_->get_ef_pose();
  // // Update the rotation and position objective
  // leftHandTask_->set_ef_pose(sva::PTransformd{sva::RotY(-M_PI / 2), Eigen::Vector3d{0.5, 0.25, 1.1}});
  return mc_control::MCController::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  // leftHandTask_->reset();
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("SimpleHumanoidController", SimpleHumanoidController)