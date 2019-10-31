#include "utility/libfranka_serialize.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/filewritestream.h"

#include <cstdio>

void writeStateToFile(const std::string &fName, const franka::RobotState &rs,
                const franka::GripperState &gs, const franka::Model &m) {
  FILE *fp = fopen(fName.c_str(), "w");

  char writeBuffer[65536];
  rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
  rapidjson::Writer<rapidjson::FileWriteStream> writer(os);

  serializeRobot(writer, rs, gs, m);

  fclose(fp);
}

const std::map<franka::Frame, std::string> frameStrings = {
    {franka::Frame::kJoint1, "kJoint1"},           {franka::Frame::kJoint2, "kJoint2"},
    {franka::Frame::kJoint3, "kJoint3"},           {franka::Frame::kJoint4, "kJoint4"},
    {franka::Frame::kJoint5, "kJoint5"},           {franka::Frame::kJoint6, "kJoint6"},
    {franka::Frame::kJoint7, "kJoint7"},           {franka::Frame::kFlange, "kFlange"},
    {franka::Frame::kEndEffector, "kEndEffector"}, {franka::Frame::kStiffness, "kStiffness"}};