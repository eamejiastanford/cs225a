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

//const bool flag_simulation = false;
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
    Vector3d current_velocity;
    Vector3d current_pos;
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
    /*
     * The default values are above.
	posori_task->_kp_pos = 120.0;
	posori_task->_kv_pos = 40.0;
	posori_task->_kp_ori = 100.0;
	posori_task->_kv_ori = 70.0;
    */

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = false;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0; //50;
	joint_task->_kv = 15.0;


	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	double taskStart_time = 0.0;
	double tTask = 0.0;

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
    
    // For printing the command torques5
    ofstream torques;
    torques.open("torques.txt");
    
    // For printing the joint velocities
    ofstream joint_velocity;
    joint_velocity.open("joint_velocity.txt");
			// cout << joint_task->_integratrueted_position_error << endl;

    // Coefficients for the polycnomial trajectory
    VectorXd a(24);
    //a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.3151859, 0.319496, -2.510322, -0.207906, -0.679664, 1.673548, 0.4328, -0.17085849917695474, 0.01, 0, -0.608469135802469, 0, 0, 3.42435802469136, 0, 0, -2.54674897119342, 0;
    //a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.271937333333333, -0.146890666666667, -1.06236533333333, -0.120861037037037, -0.00878933333333333, 0.486977185185185, 0.5328, -3.9632, 1.75, 0, 8.55376, -3.2, 0, -5.65636, 2.0, 0, 1.17264, -0.4;
    a << 0.328847, 0.458458, 0.846774, 0, 0, 0, 0.271937333333333, -0.146890666666667, -0.929032, -0.120861037037037, -0.00878933333333333, 0.427717925925926, 0.5328, -3.9632, 1.85, 0, 8.55376, -3.2, 0, -5.65636, 2.0, 0, 1.17264, -0.4;

    // Intermediate point of the trajectory
    Vector3d xDesInter = Vector3d(0.5328,0.09829, 0.20);
    // End point of the trajectory
    Vector3d xDesF = Vector3d(0.5328,-0.1, 0.25);

	VectorXd q_desired = initial_q;
	VectorXd q_circle = initial_q;
	q_desired << 0, 0, 0, -1.6, 0, 1.9, 0; 

	VectorXd q_ready_pos = initial_q;
	VectorXd q_inter_pos = initial_q;
	VectorXd q_final_pos = initial_q;
	VectorXd q_hold_pos = initial_q;
	VectorXd q_interpolated = initial_q;

    // Used for joint 1 movement controller only
    //q_ready_pos << 2.60051, 0.456864, -1.35736, -2.24334, 0.826839, 2.54507, -1.21573;
    //q_inter_pos << 1.34851, 0.456864, -1.35736, -2.24334, 0.826839, 2.54507, -1.21573;
    //q_final_pos << 0.690139, 0.456864, -1.35736, -2.24334, 0.826839, 2.54507, -1.21573;
    
    // Used for joint 1 and 2 movement controller only
    //q_ready_pos << 2.24239,-0.700111,-0.677187,-2.15274,-0.613343,1.24621,0.0269322;
    //q_inter_pos << 1.39986,0.314558,-1.1326,-2.24341,-0.613585,1.24734,0.0269352;
    //q_final_pos << 0.196068,1.27001,-1.52246,-2.1445,-0.613519,1.37792,0.0269463;

    // Karate kick swing
    // q_ready_pos << -1.63589,1.2622,-1.6661,-2.54296,-0.141813,1.55528,-0.986073;
    // q_inter_pos << 0.648375,1.2622,-1.6661,-0.592374,-0.141813,2.01618,-0.986073;
    // q_final_pos << 0.833803,1.2622,-1.6661,-0.291798,-0.141813,2.51047,-0.986073;

    // Leg kicking motion (horizontal sweep)
    // q_ready_pos << -1.63589,1.2622,-1.6661,-2.54296,-0.141813,1.85528,-0.986073;
    // q_inter_pos << 0.648375,1.2622,-1.6661,-0.592374,-0.141813,2.01618,-0.986073;
    // q_final_pos << 0.833803,1.2622,-1.6661,-0.291798,-0.141813,2.21047,-0.986073;

	// Leg kicking with an extra flick from joint 6
    q_ready_pos << -1.63589,1.2622,-1.6661,-2.54296,-0.141813,1.85528,-0.986073;
    q_inter_pos << 0.648375,1.2622,-1.6661,-1.292374,-0.141813,2.01618,-0.986073;
    //q_inter_pos << 0.648375,1.2622,-1.6661,-0.592374,-0.141813,2.01618,-0.986073;
    //q_final_pos << 0.833803,1.2622,-1.6661,-0.1011798,-0.141813,2.51047,0.186073; //joint 4 was -0.291798
    q_final_pos << 1.51053,1.2622,-1.6661,-0.479243,-0.102015,2.08507,-0.821269;


	// Leg kicking with an extra flick from joint 7
    // q_ready_pos << -1.36611,1.53385,-1.80898,-2.74587,1.37521,1.63605,0.11715;
    // q_final_pos << 0.166549,1.53385,-1.80898,-0.291798,1.37521,1.63605,-0.785267;
    // TODO: tune this to get less overshoot in hold state
 

    double tStartToInter = 1.7;
    double tInterToFinal = 1.7;
    double tMotionTotal_j4 = 0.7;
    double tMotionStart_j4 = 1.0;
    double tMotionTotal_j6 = 0.7;
    double tMotionStart_j6 = 1.0;

    VectorXd tMotionStart(dof);
    tMotionStart << 0.0, 0.0, 0.0, tMotionStart_j4, 0.0, tMotionStart_j6, 0.0;
    VectorXd tMotionTotal(dof);
    tMotionTotal << tStartToInter, tStartToInter, tStartToInter, tStartToInter, tStartToInter, tStartToInter, tStartToInter;

	while (runloop) {
		// wait for next scheduled loop
			// cout << joint_task->_integrated_position_error << endl;
		timer.waitForNextLoop(); 
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->linearVelocity(current_velocity, control_link, control_point);
        robot->position(current_pos, control_link, control_point);

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
            
		    // Desired initial configuration
            // Below was for the full swing
			//q_init_desired << 1.47471, 0.0283157, -0.55426, -1.29408, 0.294309, 2.5031, 1.38538;
            // For the semi circle swing
            // 2.60051
            // 1.34851
            // 0.690139
			joint_task->_kp = 150.0; //50;
			joint_task->_kv = 20.0;
			joint_task->_desired_position = q_ready_pos;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            joint_task->_use_velocity_saturation_flag = true;

			// Compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = saturate_torques(joint_task_torques);
            // Print the torques
			cout << (robot->_q - q_ready_pos).norm()<< endl;
            trajectory << current_pos.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;


            // Once the robot has reached close to its desired initial configuration,
            // change states to start the swing controller and start the task timer
			if( (robot->_q - q_ready_pos).norm() < 0.1 )
			{
				cout << "SWING STATE\n" <<endl;
                /*
				posori_task->reInitializeTask();
                posori_task->_use_velocity_saturation_flag = false;
				posori_task->_desired_position += Vector3d(0.0,0.0,0.0);
                posori_task->_desired_orientation = AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
				joint_task->reInitializeTask();
                */
				joint_task->_kp = 250.0;
				joint_task->_kv = 15.0;
                joint_task->_use_velocity_saturation_flag = false;
				state = SWING;
				taskStart_time = timer.elapsedTime();
			}
		}

		else if(state == SWING)
		{
            /*
             * Controller for the main swing of the robot to the ball.
             */
            
			// Initialize the task timer
            tTask = timer.elapsedTime() - taskStart_time;

            for (int i=0;i<dof;i++) {
                if (tMotionStart[i] != 0.0) {
                    if ( tTask < tMotionStart[i] ) {
                        q_interpolated[i] = q_ready_pos[i];
                    }
                    else if ( (tTask >= (tMotionStart[i])) and (tTask < tMotionTotal[i])) {
                        tTask = tTask - tMotionStart[i];
                        q_interpolated[i] = (q_ready_pos[i] + (q_inter_pos[i]-q_ready_pos[i])*tTask/tMotionTotal[i]);
                    }
                    else {
                        q_interpolated[i] = q_inter_pos[i];
                    }
                } else {
                    q_interpolated[i] = (q_ready_pos[i] + (q_inter_pos[i]-q_ready_pos[i])*tTask/tMotionTotal[i]);
                }
            }

			joint_task->_desired_position = q_interpolated;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            joint_task->_use_velocity_saturation_flag = false;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

            // Print the torques
            trajectory << current_pos.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;

			//cout << (robot->_q - q_inter_pos).norm() << endl;
            // Once the robot has reached close enough to the desired intermediate point, 
            // change to the follow through controller 
			//if ( (robot->_q - q_inter_pos).norm() < 0.05 ) {
			//if ( (robot->_q - q_final_pos).norm() < 0.3) {
			if ( (robot->_q - q_inter_pos).norm() < 0.3 ) {
				cout << "FOLLOW THROUGH STATE\n" <<endl;
				state = FOLLOW_THRU;
				taskStart_time = timer.elapsedTime();
			}

		}
	else if(state == FOLLOW_THRU)
		{
            /*
             * Controller for the main swing of the robot to the ball.
             */

			// Initialize the task timer
            tTask = timer.elapsedTime() - taskStart_time;
            
		    q_interpolated = (q_inter_pos + (q_final_pos-q_inter_pos)*tTask/tInterToFinal);
			joint_task->_desired_position = q_interpolated;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            joint_task->_use_velocity_saturation_flag = false;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

            // Print the torques
            trajectory << current_pos.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << current_velocity.transpose() << ' ' << time << endl;
            torques << command_torques.transpose() << ' ' << time << endl;

            // Once the arm is close enough to the final position, change the controller
            // to hold the arm at that position
			if ( (robot->_q - q_final_pos).norm() < 0.1 ) {
				joint_task->_kp = 150.0;
				joint_task->_kv = 15.0;
                state = HOLD;
				cout << "HOLD STATE\n" <<endl;
           
		} 
		}
        else if (state == HOLD) 
        {
            /*
             * Hold the arm at the final position once the full swing motion has been completed.
             */

            /*
            posori_task->_kp_pos = 120.0;
            posori_task->_kv_pos = 40.0;
            posori_task->_kp_ori = 100.0;
            posori_task->_kv_ori = 70.0;

            // Set the desired position to be the final position
            posori_task->_desired_position = xDesF;

         //   joint_task->_desired_position = q_desired;

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(posori_task_torques + joint_task_torques);
            */

			joint_task->_desired_velocity.setZero();

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            joint_task->_use_velocity_saturation_flag = true;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

            // Print the torques
            trajectory << current_pos.transpose() << ' ' << time << endl;
            des_trajectory << xDesF.transpose() << ' ' << time << endl;
            joints << robot->_q.transpose() << ' ' << time << endl;
            joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
            velocity << current_velocity.transpose() << ' ' << time << endl;
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
   // std::cout << "\n";
    //std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
   // std::cout << "Controller Loop updates   : " << timer.elapsedCycles() <<30 "\n";
   // std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
