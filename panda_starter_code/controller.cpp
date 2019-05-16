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

enum STATE {
    READY_POSITION,
    SWING,
    FOLLOW_THRU,
    HOLD
};

STATE state = READY_POSITION;
//int state = JOINT_CONTROLLER;

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
	posori_task->_kp_pos = 200.0;
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
    
    // For printing desired trajectory to file 
    ofstream des_trajectory;
    des_trajectory.open("des_trajectory.txt");

    VectorXd a(24);
    a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.311859, 0.319496, -2.510322, -0.207906, -0.679664, 1.673548, 0.4328, -0.170849917695474, 0.01, 0, -0.608469135802469, 0, 0, 3.42435802469136, 0, 0, -2.54674897119342, 0;

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

		if(state == READY_POSITION)
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
				posori_task->_desired_position += Vector3d(0.0,0.0,0.0);
				posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                cout << posori_task->_current_position.transpose() << ' ' << time << endl;

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = SWING;
				taskStart_time = timer.elapsedTime();
			}
		}

		else if(state == SWING)
		{
			double tTask = timer.elapsedTime() - taskStart_time;

 			Vector3d xDes = Vector3d(0.0,0.0,0.0);
 			xDes(0) = a[0] + a[3] * tTask + a[6] * pow(tTask,2) + a[9] * pow(tTask,3);
 			xDes(1) = a[1] + a[4] * tTask + a[7] * pow(tTask,2) + a[10] * pow(tTask,3);
 			xDes(2) = a[2] + a[5] * tTask + a[8] * pow(tTask,2) + a[11] * pow(tTask,3);

 			Vector3d xDesF = Vector3d(0.4328,0.09829, 0.01);

 			Vector3d vDes = Vector3d(0.0,0.0,0.0);
 			vDes(0) = a[3] + 2 * a[6] * tTask + 3 * a[9] * pow(tTask,2);
 			vDes(1) = a[4] + 2 * a[7] * tTask + 3 * a[10] * pow(tTask,2);
 			vDes(2) = a[5] + 2 * a[8] * tTask + 3 * a[11] * pow(tTask,2);

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

            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDes.transpose() << ' ' << time << endl;

			if ( (posori_task->_current_position - xDesF).norm() < 0.1 ) {
				state = FOLLOW_THRU;
			}
		}
		else if (state == FOLLOW_THRU) {

			double tTask = timer.elapsedTime() - taskStart_time;

 			Vector3d xDes = Vector3d(0.0,0.0,0.0);
 			xDes(0) = a[12] + a[15] * tTask + a[18] * pow(tTask,2) + a[21] * pow(tTask,3);
 			xDes(1) = a[13] + a[16] * tTask + a[19] * pow(tTask,2) + a[22] * pow(tTask,3);
 			xDes(2) = a[14] + a[17] * tTask + a[20] * pow(tTask,2) + a[23] * pow(tTask,3);

 			Vector3d vDes = Vector3d(0.0,0.0,0.0);
 			vDes(0) = a[15] + 2 * a[18] * tTask + 3 * a[21] * pow(tTask,2);
 			vDes(1) = a[16] + 2 * a[19] * tTask + 3 * a[22] * pow(tTask,2);
 			vDes(2) = a[17] + 2 * a[20] * tTask + 3 * a[23] * pow(tTask,2);


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

            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDes.transpose() << ' ' << time << endl;

            command_torques = posori_task_torques + joint_task_torques;

 			Vector3d xDesF = Vector3d(0.4328,-0.2, 0.01);
			if ( (posori_task->_current_position - xDesF).norm() < 0.1 ) {
                state = HOLD;
            }

		} else if (state == HOLD) {

			double tTask = timer.elapsedTime() - taskStart_time;

 			Vector3d xDesF = Vector3d(0.4328,-0.2, 0.01);
            posori_task->_desired_position = xDesF;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;

            command_torques = posori_task_torques + joint_task_torques;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    trajectory.close();
    des_trajectory.close();

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
