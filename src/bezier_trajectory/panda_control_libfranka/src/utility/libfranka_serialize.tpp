#include <franka/robot_state.h>
#include <franka/gripper_state.h>
#include <franka/model.h>

#include <array>


template <typename Writer, std::size_t SZ>
void serialize(Writer &w, const std::array<double, SZ> &arr) {
  w.StartArray();
  for (double e : arr) {
    w.Double(e);
  }
  w.EndArray();
}

template <typename Writer> void serialize(Writer &w, const franka::RobotState &s) {
  w.StartObject();
  w.Key("O_T_EE");
  serialize<Writer, 16>(w, s.O_T_EE);

  w.Key("O_T_EE_d");
  serialize<Writer, 16>(w, s.O_T_EE_d);

  w.Key("F_T_EE");
  serialize<Writer, 16>(w, s.F_T_EE);

  w.Key("EE_T_K");
  serialize<Writer, 16>(w, s.EE_T_K);

  w.Key("m_ee");
  w.Double(s.m_ee);

  w.Key("I_ee");
  serialize<Writer, 9>(w, s.I_ee);

  w.Key("F_x_Cee");
  serialize<Writer, 3>(w, s.F_x_Cee);

  w.Key("m_load");
  w.Double(s.m_load);

  w.Key("I_load");
  serialize<Writer, 9>(w, s.I_load);

  w.Key("F_x_Cload");
  serialize<Writer, 3>(w, s.F_x_Cload);

  w.Key("m_total");
  w.Double(s.m_total);

  w.Key("I_total");
  serialize<Writer, 9>(w, s.I_total);

  w.Key("F_x_Ctotal");
  serialize<Writer, 3>(w, s.F_x_Ctotal);

  w.Key("elbow");
  serialize<Writer, 2>(w, s.elbow);

  w.Key("elbow_d");
  serialize<Writer, 2>(w, s.elbow_d);

  w.Key("elbow_c");
  serialize<Writer, 2>(w, s.elbow_c);

  w.Key("delbow_c");
  serialize<Writer, 2>(w, s.delbow_c);

  w.Key("ddelbow_c");
  serialize<Writer, 2>(w, s.ddelbow_c);

  w.Key("tau_J");
  serialize<Writer, 7>(w, s.tau_J);

  w.Key("tau_J_d");
  serialize<Writer, 7>(w, s.tau_J_d);

  w.Key("dtau_J");
  serialize<Writer, 7>(w, s.dtau_J);

  w.Key("q");
  serialize<Writer, 7>(w, s.q);

  w.Key("q_d");
  serialize<Writer, 7>(w, s.q_d);

  w.Key("dq");
  serialize<Writer, 7>(w, s.dq);

  w.Key("dq_d");
  serialize<Writer, 7>(w, s.dq_d);

  w.Key("ddq_d");
  serialize<Writer, 7>(w, s.ddq_d);

  w.Key("joint_contact ");
  serialize<Writer, 7>(w, s.joint_contact);

  w.Key("cartesian_contact");
  serialize<Writer, 6>(w, s.cartesian_contact);

  w.Key("joint_collision");
  serialize<Writer, 7>(w, s.joint_collision);

  w.Key("cartesian_collision");
  serialize<Writer, 6>(w, s.cartesian_collision);

  w.Key("tau_ext_hat_filtered");
  serialize<Writer, 7>(w, s.tau_ext_hat_filtered);

  w.Key("O_F_ext_hat_K");
  serialize<Writer, 6>(w, s.O_F_ext_hat_K);

  w.Key("K_F_ext_hat_K");
  serialize<Writer, 6>(w, s.K_F_ext_hat_K);

  w.Key("O_dP_EE_d");
  serialize<Writer, 6>(w, s.O_dP_EE_d);

  w.Key("O_T_EE_c");
  serialize<Writer, 16>(w, s.O_T_EE_c);

  w.Key("O_dP_EE_c");
  serialize<Writer, 6>(w, s.O_dP_EE_c);

  w.Key("O_ddP_EE_c");
  serialize<Writer, 6>(w, s.O_ddP_EE_c);

  w.Key("theta");
  serialize<Writer, 7>(w, s.theta);

  w.Key("dtheta");
  serialize<Writer, 7>(w, s.dtheta);

  // TODO: add errors, success rate, robot_mode and time

  w.EndObject();
}

template <typename Writer> void serialize(Writer &w, const franka::GripperState &s) {
  w.StartObject();

  w.Key("width");
  w.Double(s.width);

  w.Key("max_width");
  w.Double(s.max_width);

  w.Key("is_grasped");
  w.Bool(s.is_grasped);

  w.Key("temperature");
  w.Uint(s.temperature);

  w.Key("time");
  w.Double(s.time.toSec());

  w.EndObject();
}

template <typename Writer>
void serialize(Writer &w, const franka::Model &m, const franka::RobotState &s) {
  w.StartObject();

  for (const auto &e : frameStrings) {
    w.Key(e.second.c_str());
    w.StartObject();

    w.Key("pose");
    serialize<Writer, 16>(w, m.pose(e.first, s));

    w.Key("bodyJacobian");
    serialize<Writer, 42>(w, m.bodyJacobian(e.first, s));

    w.Key("zeroJacobian");
    serialize<Writer, 42>(w, m.zeroJacobian(e.first, s));

    w.EndObject();
  }

  w.Key("mass");
  serialize<Writer, 49>(w, m.mass(s));

  w.Key("coriolis");
  serialize<Writer, 7>(w, m.coriolis(s));

  w.Key("gravity");
  serialize<Writer, 7>(w, m.gravity(s));

  w.EndObject();
}

template <typename Writer>
void serializeRobot(Writer &w, const franka::RobotState &rs, const franka::GripperState &gs,
                    const franka::Model &m) {
  w.StartObject();

  w.Key("RobotState");
  serialize<Writer>(w, rs);

  w.Key("GripperState");
  serialize<Writer>(w, gs);

  w.Key("Model");
  serialize<Writer>(w, m, rs);

  w.EndObject();
}