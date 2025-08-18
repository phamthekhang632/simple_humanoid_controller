#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/RobotLoader.h>

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
  postureTask->stiffness(5);

  // End effectors
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/left_task.yaml");
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/right_task.yaml");
  solver().addTask(rightHandTask_);

  // Save target forward and backward poses
  leftForwardPose_ = leftHandTask_->get_ef_pose();
  rightForwardPose_ = rightHandTask_->get_ef_pose();
  leftHandInitPose_ = robot().bodyPosW("l_wrist");
  rightHandInitPose_ = robot().bodyPosW("r_wrist");

  // Looking at hand

  // const auto &frames = robot().mb().joints(); // Get all frames
  // std::cout << "Robot joints:" << std::endl;
  // for (const auto &frame : frames)
  // {
  //   std::cout << "- " << frame << std::endl;
  // }

  // const auto &neckYLimit = robot().limits()["NECK_Y"];
  // std::cout << "NECK_Y limits: ["
  //           << neckYLimit.min << ", "
  //           << neckYLimit.max << "]" << std::endl;

  // lookAtTask_ = std::make_shared<mc_tasks::LookAtFrameTask>(
  //     robot().frame("NECK_R_S"), // gaze frame
  //     gazeVector,
  //     robot().frame("l_wrist")); // target frame

  lookAtTask_ = std::make_shared<mc_tasks::LookAtTask>(
      "NECK_R_S",
      gazeVector,
      robot().bodyPosW(lookingTarget).translation(),
      robots(),
      0);
  lookAtTask_->selectActiveJoints({"NECK_P", "NECK_Y", "NECK_R"});
  lookAtTask_->stiffness(10.0);
  solver().addTask(lookAtTask_);

  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  auto leftError = leftHandTask_->eval();
  auto rightError = rightHandTask_->eval();

  bool leftReached = leftError.norm() < 0.02;
  bool rightReached = rightError.norm() < 0.02;

  if (leftReached && rightReached)
  {
    switchState();
  }

  if (currentState_ == HandState::BOTH_FORWARD || currentState_ == HandState::BOTH_BACK)
  {
    lookAtTask_->target(Eigen::Vector3d(1000, 0, 0)); // Look at the center
  }
  else
  {
    lookAtTask_->target(robot().bodyPosW(lookingTarget).translation());
  }

  return mc_control::MCController::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  leftHandTask_->reset();
  rightHandTask_->reset();
  currentState_ = HandState::LEFT_FORWARD;

  lookAtTask_->reset();

  mc_control::MCController::reset(reset_data);
}

void SimpleHumanoidController::switchState()
{
  switch (currentState_)
  {
  case HandState::LEFT_FORWARD:
    leftHandTask_->set_ef_pose(leftForwardPose_);
    lookingTarget = "l_wrist";
    currentState_ = HandState::LEFT_BACK;
    break;

  case HandState::LEFT_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    currentState_ = HandState::RIGHT_FORWARD;
    break;

  case HandState::RIGHT_FORWARD:
    rightHandTask_->set_ef_pose(rightForwardPose_);
    lookingTarget = "r_wrist";
    currentState_ = HandState::RIGHT_BACK;
    break;

  case HandState::RIGHT_BACK:
    rightHandTask_->set_ef_pose(rightHandInitPose_);
    currentState_ = HandState::BOTH_FORWARD;
    break;

  case HandState::BOTH_FORWARD:
    leftHandTask_->set_ef_pose(leftForwardPose_);
    rightHandTask_->set_ef_pose(rightForwardPose_);
    currentState_ = HandState::BOTH_BACK;
    break;

  case HandState::BOTH_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    rightHandTask_->set_ef_pose(rightHandInitPose_);
    currentState_ = HandState::LEFT_FORWARD;
    break;
  }
}

CONTROLLER_CONSTRUCTOR("SimpleHumanoidController", SimpleHumanoidController)