#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

SimpleHumanoidController::SimpleHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::fsm::Controller(rm, dt, config)
{
  mc_rtc::log::success("SimpleHumanoidControllerFSM init done");
}

bool SimpleHumanoidController::run()
{
  return mc_control::fsm::Controller::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
