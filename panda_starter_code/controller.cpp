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

VectorXd saturate_torques(VectorXd torques) {
    VectorXd max_torques(7);
    max_torques << 85.0, 85.0, 85.0, 85.0, 10.0, 10.0, 10.0;
    for (int i=0;i<7;i++) {
        if (abs(torques[i]) > max_torques[i]) {
            if (torques[i] > 0.0) torques[i] = max_torques[i];
            else torques[i] = -max_torques[i];
        }
    }
    return torques;
}

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
	posori_task->_kp_pos = 120.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 70.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = false;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 50.0;
	joint_task->_kv = 40.0;

	VectorXd q_init_desired = initial_q;
    // Desired initial configuration
	q_init_desired << 1.47471, 0.0283157, -0.55426, -1.29408, 0.294309, 2.5031, 1.38538;
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
    
    // For printing the joint configuration to a file
    ofstream joints;
    joints.open("joints.txt");
    
    // For printing the velocity to a file
    ofstream velocity;
    velocity.open("velocity.txt");
    
    // For printing the command torques
    ofstream torques;
    torques.open("torques.txt");
    
    // For printing the joint velocities
    ofstream joint_velocity;
    joint_velocity.open("joint_velocity.txt");

    // Coefficients for the polynomial trajectory
    VectorXd a(24);
    //a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.311859, 0.319496, -2.510322, -0.207906, -0.679664, 1.673548, 0.4328, -0.170849917695474, 0.01, 0, -0.608469135802469, 0, 0, 3.42435802469136, 0, 0, -2.54674897119342, 0;
    //a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.271937333333333, -0.146890666666667, -1.06236533333333, -0.120861037037037, -0.00878933333333333, 0.486977185185185, 0.5328, -0.0929938921282798, 0.148432944606414, 0, -0.144118440233236, 0.0322157434402332, 0, 0.780536443148688, -0.174198250728863, 0, -0.399628279883382, 0.0874635568513119;
    a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.271937333333333, -0.146890666666667, -1.06236533333333, -0.120861037037037, -0.00878933333333333, 0.486977185185185, 0.5328, -3.9632, 1.75, 0, 8.55376, -3.2, 0, -5.65636, 2.0, 0, 1.17264, -0.4;

    // Intermediate point of the trajectory
    Vector3d xDesInter = Vector3d(0.5328,0.09829, 0.10);
    // End point of the trajectory
    Vector3d xDesF = Vector3d(0.5328,-0.1, 0.15);

	VectorXd q_desired = initial_q;
	q_desired << 0, 0, 0, -1.6, 0, 1.9, 0; 

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
            /*
             * Moves the robot to the desired initial configuration that marks the 
             * start of its swing.
             */
            
			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            joint_task->_use_velocity_saturation_flag = true;

			// Compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = saturate_torques(joint_task_torques);
            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << posori_task->_current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;

            // Print the torques

            // Once the robot has reached close to its desired initial configuration,
            // change states to start the swing controller and start the task timer
			if( (robot->_q - q_init_desired).norm() < 0.1 )
			{
				posori_task->reInitializeTask();
                posori_task->_use_velocity_saturation_flag = false;
				posori_task->_desired_position += Vector3d(0.0,0.0,0.0);
                posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				joint_task->reInitializeTask();
                joint_task->_use_velocity_saturation_flag = false;
				joint_task->_kp = 0;
				joint_task->_kv = 20;
				state = SWING;
				taskStart_time = timer.elapsedTime();
                cout << taskStart_time << endl;
			}
		}

		else if(state == SWING)
		{
            /*
             * Controller for the main swing of the robot to the ball.
             */

			// Initialize the task timer
            double tTask = timer.elapsedTime() - taskStart_time;

            // Define the desired trajectory of the EE based on the precalculated coefficients
 			Vector3d xDes = Vector3d(0.0,0.0,0.0);
 			xDes(0) = a[0] + a[3] * tTask + a[6] * pow(tTask,2) + a[9] * pow(tTask,3);
 			xDes(1) = a[1] + a[4] * tTask + a[7] * pow(tTask,2) + a[10] * pow(tTask,3);
 			xDes(2) = a[2] + a[5] * tTask + a[8] * pow(tTask,2) + a[11] * pow(tTask,3);


            // Define the desired velocity of the swing based on the derivative of the desired trajectory
 			Vector3d vDes = Vector3d(0.0,0.0,0.0);
 			vDes(0) = a[3] + 2 * a[6] * tTask + 3 * a[9] * pow(tTask,2);
 			vDes(1) = a[4] + 2 * a[7] * tTask + 3 * a[10] * pow(tTask,2);
 			vDes(2) = a[5] + 2 * a[8] * tTask + 3 * a[11] * pow(tTask,2);

			posori_task->_desired_position = xDes;
			posori_task->_desired_velocity = vDes;

            joint_task->_desired_position = q_desired;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// Compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(posori_task_torques + joint_task_torques);

            // Write the following attributes to their respective files
            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << posori_task->_current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;

            // Once the robot has reached close enough to the desired intermediate point, 
            // change to the follow through controller 
			if ( (posori_task->_current_position - xDesInter).norm() < 0.1 ) {
				state = FOLLOW_THRU;
                cout << time << endl;
			}
		} 
        else if (state == FOLLOW_THRU) 
        {
            /*
             * Controller for the swing as it goes from the intermediate point, where
             * it should have already hit the ball, to a follow through positon. This ensures
             * that the EE kicks through the ball and doesn't abruptly stop after hitting the 
             * ball.
             */

			// Initialize the task timer
            double tTask = timer.elapsedTime() - taskStart_time;

            posori_task->_kp_pos = 100.0;
            posori_task->_kv_pos = 100.0;
            posori_task->_kp_ori = 100.0;
            posori_task->_kv_ori = 70.0;

            // Define the desired EE trajectory based on the second half of the precalculated coefficients
 			Vector3d xDes = Vector3d(0.0,0.0,0.0);
 			xDes(0) = a[12] + a[15] * tTask + a[18] * pow(tTask,2) + a[21] * pow(tTask,3);
 			xDes(1) = a[13] + a[16] * tTask + a[19] * pow(tTask,2) + a[22] * pow(tTask,3);
 			xDes(2) = a[14] + a[17] * tTask + a[20] * pow(tTask,2) + a[23] * pow(tTask,3);

            // Define the desired velocity based on the desired trajectory above
 			Vector3d vDes = Vector3d(0.0,0.0,0.0);
 			vDes(0) = a[15] + 2 * a[18] * tTask + 3 * a[21] * pow(tTask,2);
 			vDes(1) = a[16] + 2 * a[19] * tTask + 3 * a[22] * pow(tTask,2);
 			vDes(2) = a[17] + 2 * a[20] * tTask + 3 * a[23] * pow(tTask,2);


            posori_task->_desired_position = xDes;
			posori_task->_desired_velocity = vDes;

            joint_task->_desired_position = q_desired;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(posori_task_torques + joint_task_torques);
            
            // Write the following attributes to their respective files
            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << posori_task->_current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;

            // Once the arm is close enough to the final position, change the controller
            // to hold the arm at that position
			if ( (posori_task->_current_position - xDesF).norm() < 0.1 ) {
                state = HOLD;
                cout << time << endl;
            }
		} 
        else if (state == HOLD) 
        {
            /*
             * Hold the arm at the final position once the full swing motion has been completed.
             */

            posori_task->_kp_pos = 120.0;
            posori_task->_kv_pos = 40.0;
            posori_task->_kp_ori = 100.0;
            posori_task->_kv_ori = 70.0;

            // Set the desired position to be the final position
            posori_task->_desired_position = xDesF;

            joint_task->_desired_position = q_desired;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(posori_task_torques + joint_task_torques);

            // Write the following attributes to their respective files
            trajectory << posori_task->_current_position.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << posori_task->_current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;
		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    trajectory.close();
    torques.close();
    des_trajectory.close();
    joints.close();
    joint_velocity.close();
    velocity.close();

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
