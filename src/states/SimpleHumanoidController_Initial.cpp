#include "SimpleHumanoidController_Initial.h"

#include "../SimpleHumanoidController.h"

void SimpleHumanoidController_Initial::configure(const mc_rtc::Configuration &config)
{
}

void SimpleHumanoidController_Initial::start(mc_control::fsm::Controller &ctl_)
{
  auto &ctl = static_cast<SimpleHumanoidController &>(ctl_);
}

bool SimpleHumanoidController_Initial::run(mc_control::fsm::Controller &ctl_)
{
  auto &ctl = static_cast<SimpleHumanoidController &>(ctl_);
  output("OK");
  return true;
}

void SimpleHumanoidController_Initial::teardown(mc_control::fsm::Controller &ctl_)
{
  auto &ctl = static_cast<SimpleHumanoidController &>(ctl_);
}

EXPORT_SINGLE_STATE("SimpleHumanoidController_Initial", SimpleHumanoidController_Initial);
