/*
This code is written by Shahrukh Khan as part of a master thesis in Cybernetics and Robotics at the Norwegian University of Science and Technology (NTNU) during the fall of 2018. The author can not guarrantee for the safety of anyone that desides to use this code in their own projects. 

Feel free to contact the author at shahrukh0104@gmail.com if you have any questions.

Thank you to Mads Johan Laastad for sharing his code on "Robotic Rehabilitation of Upper-Limb After Stroke", his master thesis project. It has been of great help in my work, and a lot of my code is based on that work, specially the part of compliance control.  

NETWORK DETAILS:
The IP address can be found in the PolyScope interface (tablet) of the robot.
SETUP Robot -> Setup NETWORK (requires password: "ngr12") -> IP address
UR5_IP = "10.42.0.63"
UR5_HOSTNAME = 'ur-2012208983' #requires dns.
F/T_SENSOR_IP = "10.42.0.8"
*/

#include "TremSup.h"

void getq(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double q[6])
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
	
	std::vector<double> q_vector = ur5->rt_interface_->robot_state_->getQActual();
	std::copy(q_vector.begin(), q_vector.end(), q);
	ur5->rt_interface_->robot_state_->setDataPublished();
}

void RobotWait(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double pose_target[6])
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	double startTime = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	double goal_tol = 0.0075;  
	while(1)
	{
		gettimeofday(&tp, NULL);
		double timeStamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		double elapsTime = (timeStamp-startTime)/1000;
	  
		std::mutex msg_lock;
		std::unique_lock<std::mutex> locker(msg_lock);
		while (!ur5->rt_interface_->robot_state_->getDataPublished())
		{
			rt_msg_cond_->wait(locker);
		}
		ur5->rt_interface_->robot_state_->setDataPublished();
		std::vector<double> q_actual = ur5->rt_interface_->robot_state_->getQActual();
	
		if(((fabs(q_actual[0] - pose_target[0])) < goal_tol) &&
	   	((fabs(q_actual[1] - pose_target[1])) < goal_tol) &&
	   	((fabs(q_actual[2] - pose_target[2])) < goal_tol) &&
	   	((fabs(q_actual[3] - pose_target[3])) < goal_tol) &&
	   	((fabs(q_actual[4] - pose_target[4])) < goal_tol) &&
	   	((fabs(q_actual[5] - pose_target[5])) < goal_tol)) break;
	   
		if(elapsTime > 15)
		{
	  	std::cout << "RobotWait is stuck - breaking out!" << std::endl;
	  	break;
		}
	}
	usleep(1000000);
}

void moveSimpleJoint(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
	
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];  
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);
	qGoal[5] = qGoal[5]-((angle_offset+TCP_OFFSET)*M_PI/180);
	char TargetString[200];
	sprintf(TargetString, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString);  
	
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleJointDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double qGoal[6], double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
	
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	char TargetString[200];
	sprintf(TargetString, "movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString);  
	
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleCart(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], int angle_offset, double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
		rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);
	qGoal[5] = qGoal[5]-((angle_offset+TCP_OFFSET)*M_PI/180);
	char TargetString[200];
	sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString); 
	
	RobotWait(ur5, rt_msg_cond_, qGoal);
}

void moveSimpleCartDirect(UrDriver *ur5, std::condition_variable *rt_msg_cond_, double T06[16], double v, double a)
{
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	while (!ur5->rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_->wait(locker);
	}
	ur5->rt_interface_->robot_state_->setDataPublished();
	
	double q[6];
	getq(ur5, rt_msg_cond_, q);
	double qGoal[6];
	getnearest(T06, q, qGoal);	
	char TargetString[200];
	sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n",
		qGoal[0], qGoal[1], qGoal[2], qGoal[3], qGoal[4], qGoal[5], v, a);
	ur5->rt_interface_->addCommandToQueue(TargetString); 
	RobotWait(ur5, rt_msg_cond_, qGoal);
} 



int main()
{
  
	// ROBOT CONNECTION
	std::condition_variable rt_msg_cond_;
	std::condition_variable msg_cond_;
	UrDriver ur5(rt_msg_cond_, msg_cond_,"10.42.0.63",5000);
	ur5.start();
	std::cout << "Connecting to robot ..." << std::endl;
	std::mutex msg_lock;
	std::unique_lock<std::mutex> locker(msg_lock);
	std::cout << "Waiting for data ..." << std::endl;
	
	while (!ur5.rt_interface_->robot_state_->getDataPublished())
	{
	rt_msg_cond_.wait(locker);
	}	
	std::cout << "Data received!" << std::endl;
	
	//GUI
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "======================== Welcome! ========================" << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	//Displays current pose
	// double q[6];
	// getq(&ur5, &rt_msg_cond_, q);	
	// char TargetString[200];
	// sprintf(TargetString, "movel([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], %.4f, %.4f)\n", q[0], q[1], q[2], q[3], q[4], q[5], 0.2, 0.2);
	// std::cout << "Current pose is: " << TargetString << std::endl;

	//q_start -> Starting position with a desired pose and good range of motion in current laboratory setup
	double q_start[6] = {0.377885, -2.60787, -2.07686, -1.57022, -0.872731, 1.62437};
	double q_start_force_tuner_testing[6] = {0.214268, -2.46378, -1.66631, -3.72433, -1.59585, 1.92308};
	double q_start_torque_tuner_testing[6] = {0.269024, -2.45595, -1.74306, -3.66909, -1.5577, 1.96192};

	char user_ready;
	char readyready;
	while(user_ready != 'y' || user_ready != 'Y')
	{
		std::cout << "======================== WARNING! ========================" << std::endl;
		std::cout << "The robot will now move to a starting position. Make sure it is safe for the robot to operate inside the workspace. The external controller is not active. \n Is the area clear [y/n]? : ";
		std::cin >> user_ready;
		if (user_ready == 'y' || user_ready == 'Y')
		{
			// MOVE TO STARING POINT
			std::cout << "======================== POSITION CONTROL ACTIVE ========================" << std::endl;
			std::cout << "Moving to staring location... ";
			moveSimpleJointDirect(&ur5, &rt_msg_cond_, q_start_torque_tuner_testing, 1, 1);
			std::cout << "Press \"Y\" when box is placed and you are ready";
			std::cin >> readyready;
			if (readyready == 'Y'|| readyready == 'y'){
				break;
			}
		}
	}
	
	// FORCE CONTROL
	double safety_timeout = 5; //[s]
	char user_status;
	
	while(user_status != 'y' || user_status != 'Y')
	{
		std::cout << std::endl;
		std::cout << "========== Control and Development of a Tremor Suppression Device ==========" << std::endl;
		std::cout << std::endl;
		break;
	}

	std::cout << "Initializing force control... \n" << std::endl;
	pthread_t forceID;
	forceControl(&ur5, &rt_msg_cond_, safety_timeout);
	
	//usleep(1000);
	//std::cout << "Shutting down force control. \n";
	//ur5.halt();
	//stopFT(&forceID);
	
	std::cout << "Disconnected!\n";
	return 0;
}
