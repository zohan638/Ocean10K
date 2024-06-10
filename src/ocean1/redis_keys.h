// Add Redis keys here
const std::string JOINT_ANGLES_KEY = "sai2::sim::ocean1::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::ocean1::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::ocean1::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::ocean1::controller";
const std::string SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_LEFT = "sai2::sim::ocean1::simulated_forces_left";
const std::string SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_RIGHT = "sai2::sim::ocean1::simulated_forces_right";
const std::string CONTROLLER_SWITCH = "sai2::sim::ocean1::switchController";
