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

#include "Sai2Primitives.h"
#include "redis_keys.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

bool runloop = false;
void sighandler(int){runloop = false;}

// States 
enum State {
	POSTURE = 0, 
	MOTION
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

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/ocean1/ocean1.urdf";
    static const string link_name = "end-effector";

	// initial state 
	int state = POSTURE;
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

    // create haptic controller
    Sai2Primitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
    auto haptic_controller =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits, robot->transformInWorld(link_name));
	haptic_controller->setScalingFactors(3.5);
	haptic_controller->setHapticControlType(
		Sai2Primitives::HapticControlType::HOMING);
	haptic_controller->disableOrientationTeleop();
	Vector3i directions_of_proxy_feedback = Vector3i::Zero();

    Sai2Primitives::HapticControllerInput haptic_input;
	Sai2Primitives::HapticControllerOtuput haptic_output;
	bool haptic_button_was_pressed = false;
	int haptic_button_is_pressed = 0;
	redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
						haptic_button_is_pressed);
	redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 1);

    // setup redis communication
	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output.device_command_force);
	redis_client.addToSendGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input.device_position);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(
		Sai2Common::ChaiHapticDriverKeys::createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_angular_velocity);
	redis_client.addToReceiveGroup(Sai2Common::ChaiHapticDriverKeys::createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   haptic_button_is_pressed);

	// create map for arm pose tasks
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> pose_tasks;

    const std::vector<std::string> control_links = {"endEffector_left", "endEffector_right"};
    const std::vector<Vector3d> control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

    for (int i = 0; i < control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity();
        compliant_frame.translation() = control_points[i];
        pose_tasks[control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, control_links[i], compliant_frame);
        pose_tasks[control_links[i]]->disableInternalOtg();
        pose_tasks[control_links[i]]->setDynamicDecouplingType(Sai2Primitives::MotionForceTask::FULL_DYNAMIC_DECOUPLING);
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

    // posture task
    auto posture_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	posture_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	posture_task->setGains(400, 40, 0);
	posture_task->setGoalPosition(q_desired);

	// get starting poses
    std::vector<Affine3d> starting_pose;
    for (int i = 0; i < control_links.size(); ++i) {
        Affine3d current_pose;
        current_pose.translation() = robot->position(control_links[i], control_points[i]);
        current_pose.linear() = robot->rotation(control_links[i]);
        starting_pose.push_back(current_pose);
    }

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

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
		haptic_input.robot_position = robot->positionInWorld(link_name);
		haptic_input.robot_orientation = robot->rotationInWorld(link_name);
		haptic_input.robot_linear_velocity =
			robot->linearVelocityInWorld(link_name);
		haptic_input.robot_angular_velocity =
			robot->angularVelocityInWorld(link_name);
		haptic_input.robot_sensed_force = Vector3d::Zero();
		haptic_input.robot_sensed_moment = Vector3d::Zero();

		haptic_output = haptic_controller->computeHapticControl(haptic_input);

		redis_client.sendAllFromGroup();
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			posture_task->updateTaskModel(N_prec);

			command_torques = posture_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				for (auto name : control_links) {
					pose_tasks[name]->reInitializeTask();
				}
				posture_task->reInitializeTask();

				state = MOTION;
			}
		} else if (state == MOTION) {
            // update body task model
            N_prec.setIdentity();
            base_task->updateTaskModel(N_prec);
            N_prec = base_task->getTaskAndPreviousNullspace();

            // update pose task models
            for (auto it = pose_tasks.begin(); it != pose_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }

            // get pose task Jacobian stack 
            MatrixXd J_pose_tasks(6 * control_links.size(), robot->dof());
            for (int i = 0; i < control_links.size(); ++i) {
                J_pose_tasks.block(6 * i, 0, 6, robot->dof()) = robot->J(control_links[i], control_points[i]);
            }        
            N_prec = robot->nullspaceMatrix(J_pose_tasks);
                
            // redundancy completion
            posture_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            // base task
            command_torques += base_task->computeTorques();

            // pose tasks
            int i = 0;
            for (auto name : control_links) {
                pose_tasks[name]->setGoalPosition(
                    haptic_output.robot_goal_position);
                pose_tasks[name]->setGoalOrientation(
                    haptic_output.robot_goal_orientation);
                command_torques += pose_tasks[name]->computeTorques();
                ++i;
            }

            // TODO (tashakim): set up haptic feedback

            // TODO (tashakim): set up state machine for button press

            // posture task and coriolis compensation
            command_torques += posture_task->computeTorques() + robot->coriolisForce();
        }

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating
    redis_client.setEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
    redis_client.setEigen(Sai2Common::ChaiHapticDriverKeys::createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(Sai2Common::ChaiHapticDriverKeys::createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	return 0;
}
