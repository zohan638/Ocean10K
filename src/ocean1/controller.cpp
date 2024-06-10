/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <iostream>
#include <mutex>
#include <random>
#include <Sai2Model.h>
#include <signal.h>
#include <string>
#include <fstream>
#include <thread>
#include <vector>

#include "Sai2Graphics.h"
#include "Sai2Primitives.h"
#include "Sai2Simulation.h"
#include "redis_keys.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;
using namespace Sai2Common::ChaiHapticDriverKeys;

const double MAX_HAPTIC_FORCE = 3.0;
const double THRESHOLD = 1.0;
const double VELOCITY_SCALING = 0.000005;
bool runloop = false;
void sighandler(int){runloop = false;}

// States 
enum State {
	POSTURE = 0, 
	MOTION
};

enum Robo_State {
	FORWARD = 0, 
	ARMS = 1
};

Eigen::VectorXd generateRandomVector(double lowerBound, double upperBound, int size) {
    // Initialize a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(lowerBound, upperBound);

    // Generate random vector
    Eigen::VectorXd randomVec(size);
    for (int i = 0; i < size; ++i) {
        randomVec(i) = dis(gen);
    }

    return randomVec;
}

// Function to calculate the angle between two 2D vectors using atan2
double calculate_angle_atan2(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    double angle1 = atan2(v1.y(), v1.x());
    double angle2 = atan2(v2.y(), v2.x());
    double angle = angle1 - angle2;

    // Normalize the angle to the range [-pi, pi]
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
}

// Function to calculate the rotations about x, y, and z axes
Eigen::Vector3d calculate_rotations(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    // Projections onto the yz-plane (perpendicular to the x-axis)
    Eigen::Vector2d a_yz(a.y(), a.z());
    Eigen::Vector2d b_yz(b.y(), b.z());
    double angle_x = calculate_angle_atan2(a_yz, b_yz);

    // Projections onto the xz-plane (perpendicular to the y-axis)
    Eigen::Vector2d a_xz(a.x(), a.z());
    Eigen::Vector2d b_xz(b.x(), b.z());
    double angle_y = calculate_angle_atan2(a_xz, b_xz);

    // Projections onto the xy-plane (perpendicular to the z-axis)
    Eigen::Vector2d a_xy(a.x(), a.y());
    Eigen::Vector2d b_xy(b.x(), b.y());
    double angle_z = calculate_angle_atan2(a_xy, b_xy);

    // Return the angles as a Vector3d (in radians)
    return Eigen::Vector3d(angle_x, angle_y, angle_z);
}

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/ocean1/ocean1.urdf";
	vector<string> link_names = {"endEffector_left", "endEffector_right"};

	// initial state 
	int state = POSTURE;
	int robo_state = ARMS;
	int prev_robo_state = ARMS;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof); 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // create haptic controllers
	Affine3d device_home_pose = Affine3d(Translation3d(0, 0, 0));
	Vector3i directions_of_proxy_feedback = Vector3i::Zero();
	vector<bool> haptic_button_was_pressed {false, false};
	vector<int> haptic_button_is_pressed {0, 0};

	Sai2Primitives::HapticDeviceController::DeviceLimits device_limits_left(
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
    Matrix3d device_rotation_in_world_left = AngleAxisd(M_PI, Vector3d::UnitZ()).toRotationMatrix();
	// device_rotation_in_world_left.col(1) *= -1;
    auto haptic_controller_left =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits_left, robot->transformInWorld(link_names[0]), device_home_pose, device_rotation_in_world_left);
	haptic_controller_left->setScalingFactors(3.5);
	haptic_controller_left->setReductionFactorForce(0.1);
	haptic_controller_left->setHapticControlType(Sai2Primitives::HapticControlType::HOMING);
	haptic_controller_left->disableOrientationTeleop();
	haptic_controller_left->setVariableDampingGainsPos(vector<double>{0.05, 0.15}, vector<double>{10, 40});
	Sai2Primitives::HapticControllerInput haptic_input_left;
	Sai2Primitives::HapticControllerOtuput haptic_output_left;
	
	Sai2Primitives::HapticDeviceController::DeviceLimits device_limits_right(
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 1)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_DAMPING_KEY_SUFFIX, 1)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_FORCE_KEY_SUFFIX, 1)));	
	Matrix3d device_rotation_in_world_right = AngleAxisd(M_PI, Vector3d::UnitZ()).toRotationMatrix();
	// device_rotation_in_world_right.col(1) *= -1;
	auto haptic_controller_right =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits_right, robot->transformInWorld(link_names[1]), device_home_pose, device_rotation_in_world_right);
	haptic_controller_right->setScalingFactors(3.5);
	haptic_controller_right->setReductionFactorForce(0.1);
	haptic_controller_right->setHapticControlType(Sai2Primitives::HapticControlType::HOMING);
	haptic_controller_right->disableOrientationTeleop();
	haptic_controller_right->setVariableDampingGainsPos(vector<double>{0.05, 0.15}, vector<double>{10, 40});
	Sai2Primitives::HapticControllerInput haptic_input_right;
	Sai2Primitives::HapticControllerOtuput haptic_output_right;
    
	for (int i=0; i<2; i++) {
		redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, i),
						haptic_button_is_pressed[i]);
		redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, i), 1);
	}
	
    // setup redis communication
	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output_left.device_command_force);
	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output_left.device_command_moment);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input_left.device_position);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input_left.device_orientation);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input_left.device_linear_velocity);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input_left.device_angular_velocity);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   haptic_button_is_pressed[0]);

	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 1),
								haptic_output_right.device_command_force);
	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 1),
								haptic_output_right.device_command_moment);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(POSITION_KEY_SUFFIX, 1),
								   haptic_input_right.device_position);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(ROTATION_KEY_SUFFIX, 1),
								   haptic_input_right.device_orientation);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 1),
		haptic_input_right.device_linear_velocity);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 1),
		haptic_input_right.device_angular_velocity);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 1),
								   haptic_button_is_pressed[1]);

	// create map for arm pose tasks
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> pose_tasks;

    const std::vector<std::string> control_links = {"endEffector_left", "endEffector_right"};
    const std::vector<Vector3d> control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

	Vector3d previous_position = (haptic_input_left.robot_position + haptic_input_right.robot_position) / 2;
	Vector3d handReference;
	Vector3d leftHandPos;
	Vector3d rightHandPos;
	Vector3d leftHandRef = robot->position("endEffector_left", Vector3d(0, 0, 0));
	Vector3d rightHandRef = robot->position("endEffector_right", Vector3d(0, 0, 0));
	handReference = leftHandRef - rightHandRef; //Initialize hand reference vector
	Vector3d handDifference;
	handDifference = leftHandRef - rightHandRef; //Initialize hand difference vector
	Vector3d goalBodyOrientation;
	Vector3d endEffectorPosSum;
	endEffectorPosSum = Vector3d(0, 0, 0);
	int k = 0;
    for (auto name : control_links) {
        endEffectorPosSum += robot->position(control_links[k], control_points[k]);
        ++k;
    }

	Vector3d endEffectorPosAverage;
	endEffectorPosAverage = endEffectorPosSum/2;

	const std::string body_control_link = "Body";
    const std::vector<Vector3d> body_control_point = {Vector3d(0, 0, 0)};

	Vector3d initialBodyPosition;
	initialBodyPosition = Vector3d(robot->position(body_control_link, body_control_point[0]));

	Vector3d leftHandToBodyDistance = leftHandRef - initialBodyPosition;
	Vector3d rightHandToBodyDistance = rightHandRef - initialBodyPosition;

	Vector3d endEffectorToBodyDistance;
	endEffectorToBodyDistance = endEffectorPosAverage - initialBodyPosition; //Distance between end effectors and body position

	Vector3d goalBodyPosition;
	goalBodyPosition = endEffectorPosAverage - endEffectorToBodyDistance; //Initialize goal body position

    for (int i = 0; i < control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = control_points[i];
        pose_tasks[control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_links[i], compliant_frame);
        pose_tasks[control_links[i]]->disableInternalOtg();
        pose_tasks[control_links[i]]->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
        pose_tasks[control_links[i]]->setPosControlGains(400, 40, 0);
        pose_tasks[control_links[i]]->setOriControlGains(400, 40, 0);
    }

    // base partial joint task 
    int num_base_joints = 6;
	MatrixXd base_selection_matrix = MatrixXd::Zero(num_base_joints, robot->dof());
	base_selection_matrix.block(0, 0, num_base_joints, num_base_joints).setIdentity();
    cout << base_selection_matrix << endl;
	auto base_task = std::make_shared<Sai2Primitives::JointTask>(robot, base_selection_matrix);
	base_task->setGains(400, 40, 0);

	VectorXd q_desired = robot->q();

	// dual arm partial joint task 
    int num_arm_joints = 14;
	MatrixXd arms_selection_matrix = MatrixXd::Zero(num_arm_joints, robot->dof());
	arms_selection_matrix.block(0, 6, num_arm_joints, num_arm_joints).setIdentity();
    cout << arms_selection_matrix << endl;
	auto arms_posture_task = std::make_shared<Sai2Primitives::JointTask>(robot, arms_selection_matrix);
	arms_posture_task->setGains(400, 40, 0); 

	// get starting poses
    std::vector<Affine3d> starting_pose;
    for (int i = 0; i < control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(control_links[i], control_points[i]);
        current_pose.linear() = robot->rotation(control_links[i]);
        starting_pose.push_back(current_pose);
    }
    
	ofstream file_test;
    file_test.open("test.txt");

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	Vector3d movedPos = Vector3d(0, 0, 0);

	Vector3d centerBodyPos = robot->position(body_control_link, body_control_point[0]);
	Matrix3d centerBodyRot = robot->rotation(body_control_link, Matrix3d::Identity());
	Vector3d centerBodyRotRPY = centerBodyRot.eulerAngles(0,1,2);
	Vector3d initHandPosLeft = robot->position("endEffector_left", Vector3d(0, 0, 0));
	Vector3d initHandPosRight = robot->position("endEffector_right", Vector3d(0, 0, 0));
	double yaw_desired = centerBodyRotRPY(2);

	double prev_time = timer.elapsedSimTime();

	while (runloop) {
		timer.waitForNextLoop();

		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

        // read haptic device state from Redis
		redis_client.receiveAllFromGroup();

        // compute haptic control
		haptic_input_left.robot_position = robot->positionInWorld(link_names[0]);
		haptic_input_left.robot_orientation = robot->rotationInWorld(link_names[0]);
		haptic_input_left.robot_linear_velocity =
			robot->linearVelocityInWorld(link_names[0]);
		haptic_input_left.robot_angular_velocity =
			robot->angularVelocityInWorld(link_names[0]);
		// send left end-effector simulated force sensor forces to haptic devices
		haptic_input_left.robot_sensed_force = redis_client.getEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_LEFT);
		haptic_output_left = haptic_controller_left->computeHapticControl(haptic_input_left);

		haptic_input_right.robot_position = robot->positionInWorld(link_names[1]);
		haptic_input_right.robot_orientation = robot->rotationInWorld(link_names[1]);
		haptic_input_right.robot_linear_velocity =
			robot->linearVelocityInWorld(link_names[1]);
		haptic_input_right.robot_angular_velocity =
			robot->angularVelocityInWorld(link_names[1]);
		// send right end-effector simulated force sensor forces to haptic devices
		haptic_input_right.robot_sensed_force = redis_client.getEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_RIGHT);
		haptic_output_right = haptic_controller_right->computeHapticControl(haptic_input_right);

		redis_client.sendAllFromGroup();
		// get controller switch key
		int switch_control_type_flag = std::stoi(redis_client.get(CONTROLLER_SWITCH));
		if (switch_control_type_flag) {
			if (robo_state == FORWARD) {
				robo_state = ARMS;
				prev_robo_state = FORWARD;
			} else {
				robo_state = FORWARD;
				prev_robo_state = ARMS;
			}
			
		}
		redis_client.set(CONTROLLER_SWITCH, "0");
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			arms_posture_task->updateTaskModel(N_prec);

			command_torques = arms_posture_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				for (auto name : control_links) {
					pose_tasks[name]->reInitializeTask();
				}
				arms_posture_task->reInitializeTask();

				state = MOTION;
			}

			//MAYBE WE UPDATE POSTURE TASK i.e. q_desired = robot->q()
		} else if (state == MOTION) {
			// if (haptic_controller_left->getHapticControlType() ==
			// 		Sai2Primitives::HapticControlType::MOTION_MOTION &&
			// 	haptic_button_is_pressed[0] && !haptic_button_was_pressed[0]) {
			// 	haptic_controller_left->setHapticControlType(
			// 		Sai2Primitives::HapticControlType::CLUTCH);
			// } else if (haptic_controller_left->getHapticControlType() ==
			// 			Sai2Primitives::HapticControlType::CLUTCH &&
			// 		!haptic_button_is_pressed[0] && haptic_button_was_pressed[0]) {
			// 	haptic_controller_left->setHapticControlType(
			// 		Sai2Primitives::HapticControlType::MOTION_MOTION);
			// }
			// if (haptic_controller_right->getHapticControlType() ==
			// 		Sai2Primitives::HapticControlType::MOTION_MOTION &&
			// 	haptic_button_is_pressed[1] && !haptic_button_was_pressed[1]) {
			// 	haptic_controller_right->setHapticControlType(
			// 		Sai2Primitives::HapticControlType::CLUTCH);
			// } else if (haptic_controller_right->getHapticControlType() ==
			// 			Sai2Primitives::HapticControlType::CLUTCH &&
			// 		!haptic_button_is_pressed[1] && haptic_button_was_pressed[1]) {
			// 	haptic_controller_right->setHapticControlType(
			// 		Sai2Primitives::HapticControlType::MOTION_MOTION);
			// }
            // update body task model
			// int j = 0;

			// endEffectorPosSum = Vector3d(0, 0, 0);
			// for (auto name : control_links) {
			// 	endEffectorPosSum += robot->position(control_links[j], control_points[j]);
			// 	++j;
			// }	

			// endEffectorPosAverage = endEffectorPosSum/2;
			// goalBodyPosition = endEffectorPosAverage - endEffectorToBodyDistance; // Calculate the body position as the end effector location minus some offset

			// leftHandPos = robot->position("endEffector_left", Vector3d(0, 0, 0));
			// rightHandPos = robot->position("endEffector_right", Vector3d(0, 0, 0));
			// handDifference = leftHandPos - rightHandPos; // Get vector between end effectors
			// goalBodyOrientation = calculate_rotations(handDifference, handReference); // Calculate the angle between the reference vector between end effectors and the current one

            N_prec.setIdentity();
			
            base_task->updateTaskModel(N_prec); // base task is set to identity meaning its highest priority
            N_prec = base_task->getTaskAndPreviousNullspace(); // Everything that uses N_prec is lower priority


			//base_task->setGoalPosition(Vector6d(goalBodyPosition[0], goalBodyPosition[1], goalBodyPosition[2], goalBodyOrientation[2], 0, goalBodyOrientation[0])); 


            // update pose task models
            for (auto it = pose_tasks.begin(); it != pose_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec); // updates task to be in nullspace of previous tasks??
                // N_prec = it->second->getTaskAndPreviousNullspace(); //should this be activated?
            }

            // get pose task Jacobian stack 
            MatrixXd J_pose_tasks(6 * control_links.size(), robot->dof());
            for (int i = 0; i < control_links.size(); ++i) {
                J_pose_tasks.block(6 * i, 0, 6, robot->dof()) = robot->J(control_links[i], control_points[i]);
            }        
            N_prec = robot->nullspaceMatrix(J_pose_tasks); 
                
            // redundancy completion
            arms_posture_task->updateTaskModel(N_prec); // updates task to be in null space of previous task

			Vector3d curr_Robot = robot->position(body_control_link, body_control_point[0]);
			Matrix3d body_rotation_in_world = robot->rotationInWorld(body_control_link, Matrix3d::Identity());
			Vector3d body_rotation_in_world_rpy = body_rotation_in_world.eulerAngles(0,1,2);
			
			// endEffectorPosAverage = endEffectorPosSum/2;
			// goalBodyPosition = endEffectorPosAverage - endEffectorToBodyDistance; // Calculate the body position as the end effector location minus some offset
			// leftHandPos = robot->position("endEffector_left", Vector3d(0, 0, 0));
			// rightHandPos = robot->position("endEffector_right", Vector3d(0, 0, 0));
			// handDifference = leftHandPos - rightHandPos; // Get vector between end effectors
			// goalBodyOrientation = calculate_rotations(handDifference, handReference); // Calculate the angle between the reference vector between end effectors and the current one

			Vector3d hapticPosDiff = haptic_output_left.robot_goal_position - haptic_output_right.robot_goal_position;
			goalBodyOrientation = calculate_rotations(hapticPosDiff, Vector3d(0,1,0));

            // -------- set task goals and compute control torques
            command_torques.setZero(); // set the command torques equal to 0

            //command_torques += base_task->computeTorques(); // set the command torques of the base task


			//In forward mode lock arms and increment the base
			if (robo_state == FORWARD) {
				// if(prev_robo_state != FORWARD) {

				// }
				// Vector3d curr_Robot = robot->position(body_control_link, body_control_point[0]);
				// body_rotation_in_world.col(0) << 1,0,0;
				// body_rotation_in_world.col(1) << 0,1,0;
				pose_tasks["endEffector_left"]->setGoalPosition(curr_Robot + (body_rotation_in_world * leftHandToBodyDistance)); 
				command_torques += pose_tasks["endEffector_left"]->computeTorques();
				pose_tasks["endEffector_right"]->setGoalPosition(curr_Robot + (body_rotation_in_world * rightHandToBodyDistance));
				command_torques += pose_tasks["endEffector_right"]->computeTorques();

				Vector3d current_position = (haptic_input_left.device_position + haptic_input_right.device_position) / 2;
				//cout << haptic_input_left.device_position << endl << "\n";
				//Vector3d current_position = (haptic_output_left.robot_goal_position + haptic_output_right.robot_goal_position) / 2;
				Vector3d v_desired = current_position * VELOCITY_SCALING;
				Vector3d new_position =  (new_position + (v_desired * 500)); //Changed this from previous position to new position
				Vector3d previous_position = current_position;
				//cout << new_position << endl << "\n";

				//new_position = body_rotation_in_world.transpose() * new_position; //Commented this out


				yaw_desired += goalBodyOrientation(2) * 0.001; 

				//base_task->setGoalPosition(Vector6d(-new_position[0], new_position[1], new_position[2], yaw_desired, 0, 0));
				base_task->setGoalPosition(Vector6d(-new_position[0], -new_position[1], new_position[2], 0, 0, 0)); //Got rid of yaw desired
				// base_task->setGoalPosition(Vector6d(current_position[0], current_position[1], current_position[2], yaw_desired, 0, 0));
				command_torques += base_task->computeTorques(); 

				//cout << "forward" << endl << "\n";
					//cout << robot->position(body_control_links[0], body_control_points[0]) << endl << "\n";
			} else if (robo_state == ARMS) {
				if (prev_robo_state != ARMS) {
					base_task->reInitializeTask();
					int j = 0;

					endEffectorPosSum = Vector3d(0, 0, 0);
					for (auto name : control_links) {
						endEffectorPosSum += robot->position(control_links[j], control_points[j]);
						++j;
				}	
					centerBodyPos = robot->position(body_control_link, body_control_point[0]);
					centerBodyRot = robot->rotation(body_control_link, Matrix3d::Identity());
					// centerBodyRot.col(0) << 1,0,0;
					// centerBodyRot.col(1) << 0,1,0;
					centerBodyRotRPY = centerBodyRot.eulerAngles(0,1,2);
					initHandPosLeft = robot->position("endEffector_left", Vector3d(0, 0, 0));
					initHandPosRight = robot->position("endEffector_right", Vector3d(0, 0, 0));
					// base_task->setGoalPosition(Vector6d(centerBodyPos[0], centerBodyPos[1], centerBodyPos[2], yaw_desired, 0, 0));
					base_task->setGoalPosition(Vector6d(centerBodyPos[0], centerBodyPos[1], centerBodyPos[2], 0, 0, 0)); //Took out yaw desired
					// base_task->setGoalPosition(Vector6d(centerBodyPos[0], centerBodyPos[1], centerBodyPos[2], centerBodyRotRPY(2), 0, 0));
					// base_task->setGoalPosition(Vector6d(centerBodyPos[0], centerBodyPos[1], centerBodyPos[2], goalBodyOrientation(2), 0, 0));
					//base_task->setGoalPosition(Vector6d(goalBodyPosition[0], goalBodyPosition[1], goalBodyPosition[2], goalBodyOrientation[2], 0, goalBodyOrientation[0]));
					command_torques += base_task->computeTorques();
					pose_tasks["endEffector_left"]->setGoalPosition(robot->position(control_links[0], control_points[0]));
					command_torques += pose_tasks["endEffector_left"]->computeTorques();
					pose_tasks["endEffector_right"]->setGoalPosition(robot->position(control_links[1], control_points[1]));
					command_torques += pose_tasks["endEffector_right"]->computeTorques();
					//cout << "TRANSITION" << endl << "\n";
					prev_robo_state = ARMS;

				} else {
					// Vector3d curr_Robot = robot->position(body_control_link, body_control_point[0]);
					// base_task->setGoalPosition(Vector6d(centerBodyPos[0], centerBodyPos[1], centerBodyPos[2], centerBodyRotRPY(2), 0, 0));
					command_torques += base_task->computeTorques();
					// centerBodyRot.setIdentity();
					pose_tasks["endEffector_left"]->setGoalPosition(centerBodyPos + (centerBodyRot*haptic_output_left.robot_goal_position));
					// pose_tasks["endEffector_left"]->setGoalPosition(initHandPosLeft + (centerBodyRot*haptic_output_left.robot_goal_position));
					command_torques += pose_tasks["endEffector_left"]->computeTorques();
					pose_tasks["endEffector_right"]->setGoalPosition(centerBodyPos + (centerBodyRot*haptic_output_right.robot_goal_position));
					// pose_tasks["endEffector_right"]->setGoalPosition(initHandPosRight + (centerBodyRot*haptic_output_right.robot_goal_position));
					command_torques += pose_tasks["endEffector_right"]->computeTorques();
					//cout << "euler angles: " << centerBodyRotRPY.transpose() << endl;
					//cout << "arms" << endl << "\n";
					prev_robo_state = ARMS;
				}

				// pose_tasks["endEffector_left"]->setGoalPosition(haptic_output_left.robot_goal_position);
				// command_torques += pose_tasks["endEffector_left"]->computeTorques();
				// pose_tasks["endEffector_right"]->setGoalPosition(haptic_output_right.robot_goal_position);
				// command_torques += pose_tasks["endEffector_right"]->computeTorques();
				// cout << "arms" << endl << "\n";
				//cout << robot->position(body_control_links[0], body_control_points[0]) << endl << "\n";
			}	


				//file_test << goalPos[0] << "\t" << goalPos[1] << "\t" << goalPos[2] << "\t" << "\n";
				// TODO (tashakim): set up state machine for button press
				// state machine for button presses
			if (haptic_controller_left->getHapticControlType() == Sai2Primitives::HapticControlType::HOMING) {
				haptic_controller_left->setHapticControlType(Sai2Primitives::HapticControlType::MOTION_MOTION);
				haptic_controller_left->setDeviceControlGains(350.0, 15.0);
			}
			if (haptic_controller_right->getHapticControlType() == Sai2Primitives::HapticControlType::HOMING) {
				haptic_controller_right->setHapticControlType(Sai2Primitives::HapticControlType::MOTION_MOTION);
				haptic_controller_right->setDeviceControlGains(350.0, 15.0);
			}

				// posture task and coriolis compensation
			command_torques += arms_posture_task->computeTorques() + robot->coriolisForce();
		}
		
		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		prev_time = time;

		for (int i=0; i<2; i++) {
			haptic_button_was_pressed[i] = haptic_button_is_pressed[i];
		}
	
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating
    redis_client.setEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 1),
						  Vector3d::Zero());
    redis_client.setEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);
	
	return 0;
}
