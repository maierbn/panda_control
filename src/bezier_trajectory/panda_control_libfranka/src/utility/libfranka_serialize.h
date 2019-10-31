#pragma once

#include <franka/robot_state.h>
#include <franka/gripper_state.h>
#include <franka/model.h>

#include <map>

void writeStateToFile(const std::string &fName, const franka::RobotState &rs,
                const franka::GripperState &gs, const franka::Model &m);

extern const std::map<franka::Frame, std::string> frameStrings;

#include "utility/libfranka_serialize.tpp"