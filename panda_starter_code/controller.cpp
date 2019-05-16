// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <fstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = JOINT_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

// const bool flag_simulation = false;
const bool flag_simulation = true;

const bool inertia_regularization = true;

int main() {

	// Choose where to get sensor values
	if(flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";

		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";		
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = false;
#else
	posori_task->_use_velocity_saturation_flag = false;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 2000.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = false;
#else
	joint_task->_use_velocity_saturation_flag = false;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
    // Desired initial q
	q_init_desired << 1.47471, 0.0283157, -0.55426, -1.29408, 0.994309, 2.5031, 1.38538;
	//q_init_desired << -40.0, -15.0, -45.0, -105.0, 0.0, 90.0, 45.0;
	//q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	double taskStart_time = 0.0;

    // For printing trajectory to file 
    ofstream trajectory;
    trajectory.open("trajectory.txt");

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
		}

		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_init_desired).norm() < 0.15 )
			{
				posori_task->reInitializeTask();
				//posori_task->_desired_position = Vector3d(0.4328,0.09829, 0.01);
				posori_task->_desired_position += Vector3d(-0.0,0.0,0.0);
				posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                cout << posori_task->_current_position.transpose() << ' ' << time << endl;

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = POSORI_CONTROLLER;
				taskStart_time = timer.elapsedTime();
			}
		}

		else if(state == POSORI_CONTROLLER)
		{
			double tTask = timer.elapsedTime() - taskStart_time;

 			float a0 = 0.3414;
 			float a1 = 0.0;
 			float a2 = 0.0305;
 			float a3 = -0.0068;

 			float b0 = 0.4220;
 			float b1 = 0.0;
 			float b2 = 0.2254;
 			float b3 = -0.0871;

 			float c0 = 0.9655;
 			float c1 = 0.0;
 			float c2 = -0.3185;
 			float c3 = 0.0708;

 			Vector3d xDes = Vector3d(0.0,0.0,0.0);
 			xDes(0) = a0 + a1 * tTask + a2 * pow(tTask,2) + a3 * pow(tTask,3);
 			xDes(1) = b0 + b1 * tTask + b2 * pow(tTask,2) + b3 * pow(tTask,3);
 			xDes(2) = c0 + c1 * tTask + c2 * pow(tTask,2) + c3 * pow(tTask,3);

 			Vector3d vDes = Vector3d(0.0,0.0,0.0);
 			vDes(0) = a1 + 2 * a2 * pow(tTask,1) + 3 * a3 * pow(tTask,2);
 			vDes(1) = b1 + 2 * b2 * pow(tTask,1) + 3 * b3 * pow(tTask,2);
 			vDes(2) = c1 + 2 * c2 * pow(tTask,1) + 3 * c3 * pow(tTask,2);

 			double tf = 3;
 			//Vector3d xDesF = Vector3d(0.0,0.0,0.0);
 			Vector3d xDesF = Vector3d(0.4328,0.09829, 0.01);
 			xDesF(0) = a0 + a1 * tf + a2 * pow(tf,2) + a3 * pow(tf,3);
 			xDesF(1) = b0 + b1 * tf + b2 * pow(tf,2) + b3 * pow(tf,3);
 			xDesF(2) = c0 + c1 * tf + c2 * pow(tf,2) + c3 * pow(tf,3);

			posori_task->_desired_position = xDes;
			posori_task->_desired_velocity = vDes;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;

			if ( (posori_task->_current_position - xDesF).norm() < 0.2 ) {
				state = 3;
			}
		}
		else if (state == 3) {
			Vector3d posEE = Vector3d::Zero();
			robot->position(posEE, control_link, control_point);

			posori_task->_desired_position = posEE;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;

            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    trajectory.close();

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
