// @file simviz.cpp

#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include "redis_keys.h"

using namespace Eigen;
using namespace std;

// States 
enum State {
	THIRD_PERSON = 0, 
	FIRST_PERSON = 1,
	DEBUG_CAM 
};

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

//NEW VARIALBES
Vector3d newCamLookat;
Vector3d newCamVert;
Vector3d newCamPos;
const double GROUND_Z = 0.0; // z-coordinate of the ground plane
const double TOUCH_TOLERANCE = 0.05; // tolerance to check if the object is touching the ground
const double GROUND_X_MIN = -6.0; // x-coordinate min boundary of the ground
const double GROUND_X_MAX = 6.0; // x-coordinate max boundary of the ground
const double GROUND_Y_MIN = -6.0; // y-coordinate min boundary of the ground
const double GROUND_Y_MAX = 6.0; // y-coordinate max boundary of the ground
int score = 0; // score variable
const int MAX_SCORE = 3; // Maximum score to finish the game

// specify urdf and robots 
static const string robot_name = "ocean1";
static const string camera_name = "camera_fixed";

// dynamic objects information
const vector<std::string> object_names = {"plastic_bag", "rope", "whale_fall", "welcome_sign", "deep_sponges", "rubber_duck", "trash_can", "rock", "bin"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main() {
	
	Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/ocean1/ocean1.urdf";
	static const string world_file = string(OCEAN1_FOLDER) + "/world_ocean1.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
	graphics->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	//graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
	// graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes 
	graphics->addUIForceInteraction(robot_name);

	// load robots
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	//robot->setQ();
	//robot->setDq();
	robot->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
	
	sim->addSimulatedForceSensor(robot_name, "endEffector_left", Affine3d::Identity(),
								 10.0);
	sim->addSimulatedForceSensor(robot_name, "endEffector_right", Affine3d::Identity(),
								 10.0);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[1]);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	// fill in object information 
	for (int i = 0; i < n_objects; ++i) {
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	// init redis client values 
	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q()); 
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq()); 
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());
	redis_client.setEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_LEFT, Vector3d(0,0,0));
	redis_client.setEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_RIGHT, Vector3d(0,0,0));


	// start simulation thread
	thread sim_thread(simulation, sim);
		
	VectorXd robot_q = robot->q(); //Makes robot_q the joint angles of the robot (since the body is prismatic, this is fine)

	// initial state 
	int state = DEBUG_CAM;
	// while window is open:
	while (graphics->isWindowOpen()) {
		robot_q = redis_client.getEigen(JOINT_ANGLES_KEY); //Updates the joint angles on each iteration
        graphics->updateRobotGraphics(robot_name, robot_q);
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i) {
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[1]);
		graphics->renderGraphicsWorld();

		if (state == FIRST_PERSON) {
			newCamPos = robot_q.head(3) + Vector3d(0.9, 0, 1.5); //Sets the camera position //Comment out if want default camera
			newCamLookat = robot_q.head(3) + Vector3d(1, 0, -0.5); //Tells the camera what to look at //Comment out if want default camera
		} else if (state == THIRD_PERSON) {
			newCamPos = robot_q.head(3) + Vector3d(0.0, 0, 1.5); //Sets the camera position //Comment out if want default camera
			newCamLookat = robot_q.head(3) + Vector3d(1, 0, 0.7); //Tells the camera what to look at //Comment out if want default camera
		} else {

		}

		newCamVert = Vector3d::UnitZ(); //Sets the reference vertical for the camera
		
		
		graphics->renderGraphicsWorld();
		graphics->setCameraPose(camera_name, newCamPos, newCamVert, newCamLookat); //Updates the camera pose
		graphics->render(camera_name); //Renders the new camera
		
		
		graphics->renderGraphicsWorld();
		// graphics->setCameraPose(camera_name, newCamPos, newCamVert, newCamLookat); //Updates the camera pose
		// graphics->render(camera_name); //Renders the new camera

		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

    // stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	fSimulationRunning = true;

    // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	while (fSimulationRunning) {
		timer.waitForNextLoop();

		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
		// force sensor data
		auto force_data = sim->getAllForceSensorData();
		for (auto force : force_data) {
			if (force.link_name == "endEffector_right") {
				redis_client.setEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_RIGHT, force.force_world_frame);
			} else if (force.link_name == "endEffector_left") {
				redis_client.setEigen(SIMULATED_COMMANDED_FORCE_KEY_SUFFIX_LEFT, force.force_world_frame);
			}
		}
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		// update object information 
        {
            lock_guard<mutex> lock(mutex_update);
            for (int i = 0; i < n_objects; ++i) {
                object_poses[i] = sim->getObjectPose(object_names[i]);
                object_velocities[i] = sim->getObjectVelocity(object_names[i]);
                // Check if object is within the ground boundaries and touching the ground
                Vector3d object_pos = object_poses[i].translation();
                if (object_pos.z() <= (GROUND_Z + TOUCH_TOLERANCE) &&
                    object_pos.z() >= (GROUND_Z - TOUCH_TOLERANCE) &&
                    object_pos.x() >= GROUND_X_MIN &&
                    object_pos.x() <= GROUND_X_MAX &&
                    object_pos.y() >= GROUND_Y_MIN &&
                    object_pos.y() <= GROUND_Y_MAX) {
                    score++;
                    std::cout << "Score: " << score << std::endl;
                    // Move the object slightly above the ground to prevent multiple scoring
                    // sim->setObjectPosition(object_names[i], object_pos + Vector3d(0, 0, TOUCH_TOLERANCE + 0.01));

					// Check if maximum score is reached
                    if (score >= MAX_SCORE) {
                        std::cout << "Maximum score reached! Ending simulation." << std::endl;
                        fSimulationRunning = false;
                        break;
                    }
                }
            }
        }
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
