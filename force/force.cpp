#include "force.h"

#include "ujac_c.c"
#include "ufwdkin_c.c"

int initFT = 1;
int iteration_sleeptime = 8000; 
//Fastest possible hardware update rate for the UR5 is set at 8ms ~ 125 Hz equivalent to usleep(8000)
//usleep(10000) ~ 90 Hz --> run a bit faster than double the frequency of the fastest human reaction time -- currently involuntary muscle contractions at 24ms ~ 41,6 Hz. 
double rawFTdata[6];
double biasTF[3]; // Tool frame (or TCP frame)


void *getFTData(void *arg)
{
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	int i;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
	//char const *AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */
	


	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1)
	{
		exit(1);
	}

	*(uint16*)&request[0] = htons(0x1234); 		/* standard header. */
	*(uint16*)&request[2] = htons(COMMAND); 	/* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

	/* Sending the request. */
	he = gethostbyname("10.42.0.8");


	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);

	err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
	if (err == -1)
	{
		exit(2);
	}
	send(socketHandle, request, 8, 0);

	
	int scale_factor = 1000000; //Scales to Newton [N] and Newtonmeter [Nm]
	while(initFT)
	{
		/* Receiving the response. */
		recv( socketHandle, response, 36, 0 );
		resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
		resp.ft_sequence = ntohl(*(uint32*)&response[4]);
		resp.status = ntohl(*(uint32*)&response[8]);
		for( i = 0; i < 6; i++ )
		{
			resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
		}


		double Fx = (double)resp.FTData[0]/scale_factor;
		double Fy = (double)resp.FTData[1]/scale_factor;
		double Fz = (double)resp.FTData[2]/scale_factor;
		double Tx = (double)resp.FTData[3]/scale_factor;
		double Ty = (double)resp.FTData[4]/scale_factor;
		double Tz = (double)resp.FTData[5]/scale_factor;
	
		rawFTdata[0] = Fx; rawFTdata[1] = Fy; rawFTdata[2] = Fz; rawFTdata[3] = Tx; rawFTdata[4] = Ty; rawFTdata[5] = Tz;
		usleep(3800); // sample raw data faster then 4000 microseconds ~ 250 Hz is current FT broadcast frequency
	}
}

void startFT(pthread_t *forceID)
{
	//pthread_t forceID;
	if(pthread_create(forceID, NULL, &getFTData, NULL))
	{
		std::cout << "Error: force/torque thread not created!" << std::endl;
		//return -1;
	}
	std::cout << "Broadcasting force/torque data!" << std::endl;
	usleep(100000);
}

void stopFT(pthread_t *forceID)
{
	initFT = 0;
	pthread_join(*forceID, NULL);
	std::cout << "Force thread joined - stopping force/torque data acquisition!" << std::endl;
	usleep(100000);
}

void rotate(gsl_vector *res,gsl_matrix *R, gsl_vector *inp,gsl_vector *t1,gsl_vector *t2)
{
	t1->data=inp->data;
	t2->data=res->data;
	gsl_blas_dgemv(CblasNoTrans ,1.0,R, t1,0.0,t2); 
	t1->data=&inp->data[3];
	t2->data=&res->data[3];
	gsl_blas_dgemv(CblasNoTrans ,1.0,R, t1,0.0,t2); 
}

void vector_trans_base_tool(std::vector<double> q, double vector_in[3], double vector_out[3])
{	
	gsl_matrix *R = gsl_matrix_calloc(3,3);
	gsl_vector *O = gsl_vector_alloc(3);
	
	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.08313};

	tfrotype tfkin;
	R->data=tfkin.R;
	O->data=tfkin.O;
	ufwdkin(&tfkin,q.data(),apar,dpar);

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3 ;j++)
		{
			vector_out[i] += vector_in[j]*gsl_matrix_get(R, j, i);
		}
	}
	
}

void vector_trans_tool_base(std::vector<double> q, double vector_in[3], double vector_out[3])
{	
	gsl_matrix *R = gsl_matrix_calloc(3,3);
	gsl_vector *O = gsl_vector_alloc(3);
	
	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.08313};

	tfrotype tfkin;
	R->data=tfkin.R;
	O->data=tfkin.O;
	ufwdkin(&tfkin,q.data(),apar,dpar);

	gsl_matrix_transpose(R);

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3 ;j++)
		{
			vector_out[i] += vector_in[j]*gsl_matrix_get(R, j, i);
		}
	}
	
}

void cross_product(const gsl_vector *u, const gsl_vector *v, gsl_vector *product)
{
        double p1 = gsl_vector_get(u, 1)*gsl_vector_get(v, 2)
                - gsl_vector_get(u, 2)*gsl_vector_get(v, 1);

        double p2 = gsl_vector_get(u, 2)*gsl_vector_get(v, 0)
                - gsl_vector_get(u, 0)*gsl_vector_get(v, 2);

        double p3 = gsl_vector_get(u, 0)*gsl_vector_get(v, 1)
                - gsl_vector_get(u, 1)*gsl_vector_get(v, 0);

        gsl_vector_set(product, 0, p1);
        gsl_vector_set(product, 1, p2);
        gsl_vector_set(product, 2, p3);
}

double triple_scalar_product(const gsl_vector *u, const gsl_vector *v, const gsl_vector *w)
{
        double result;
        double tmp[] = {0.0, 0.0, 0.0};
        gsl_vector_view vxw = gsl_vector_view_array((double *) &tmp, 3);

        cross_product(v, w, &vxw.vector);
        gsl_blas_ddot(u, &vxw.vector, &result);

        return result;
}

vector<string> split(string str, char delimiter) {
  vector<string> internal;
  stringstream ss(str); // Turn the string into a stream.
  string tok;
 
  while(getline(ss, tok, delimiter)) {
    internal.push_back(tok);
  }
 
  return internal;
}
 



void solveInverseJacobian(std::vector<double> q, double vw[6], double qd[6])
{
	gsl_vector *x = gsl_vector_alloc(6);
	gsl_vector *vw_ = gsl_vector_alloc(6);
	gsl_vector *t1 = gsl_vector_alloc(3);
	gsl_vector *t2 = gsl_vector_alloc(3);
	gsl_vector *vw_w = gsl_vector_alloc(6);
	gsl_matrix *R = gsl_matrix_alloc(3,3);
	gsl_matrix *A = gsl_matrix_alloc(6,6);
	gsl_permutation *p = gsl_permutation_alloc(6);
	
	int signum;
	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.08313};
	double fqd;
	double sat = 0;
	
	gsl_vector_set(vw_,0,vw[0]);
	gsl_vector_set(vw_,1,vw[1]);
	gsl_vector_set(vw_,2,vw[2]);
	gsl_vector_set(vw_,3,vw[3]);
	gsl_vector_set(vw_,4,vw[4]);
	gsl_vector_set(vw_,5,vw[5]);
	
	
	tfrotype tfkin;
	R->data=tfkin.R;
	ufwdkin(&tfkin,q.data(),apar,dpar);
	rotate(vw_w,R,vw_,t1,t2);
	
	
	ujac(A->data,q.data(),apar,dpar);
	
	gsl_linalg_LU_decomp(A,p,&signum);

	gsl_linalg_LU_solve(A,p,vw_w,x);
	
	for (int k=0;k<6;k++)
	{
		qd[k] = gsl_vector_get(x,k); 
		fqd = fabs(qd[k]);
		if (qd[k] > 1 || qd[k] < -1)
		{
			sat = fqd > sat ? fqd : sat;   //Note: '?:' is a ternary operator
		}
	}
	if(sat > 0)
	{
		for (int i = 0; i < 6; i++)
		{
			qd[i] = qd[i]/sat*1;
		}
	}
}


void forceControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int run_time)//, double des_x, double des_y, double des_z)
{	
	//START F/T SENSOR
	pthread_t forceID;
	startFT(&forceID);
	std::cout << "Force control initiated - starting data logging ..." << std::endl;

	//START ARDUINO READ
	cArduino arduino(ArduinoBaundRate::B230400bps);
	if(!arduino.isOpen()){
		std::cerr << "Can't open Arduino Uno" << endl;
	}	
	std::cout << "Arduino Open At " << arduino.getDeviceName() << endl;
	
	//OPEN LOGGING FILES
	std::ofstream forcelog;
	std::ofstream accelerolog;
	std::ofstream frequencylog;
	forcelog.open("../data/logs/forcelog", std::ofstream::out);
	accelerolog.open("../data/logs/accelerolog", std::ofstream::out);
	frequencylog.open("../data/logs/frequencylog", std::ofstream::out);
	//DEFINE ITERATION AND TIME VARIABLES
	int i = 0; 
	double iteration_time = 0.008;
	int iter = run_time/iteration_time;
	double time_stamp;
	double elaps_time;
	
	//CONTROL SYSTEM FOR TRANSLATION FORCES
	double integrator_Fx = 0, integrator_Fy = 0, integrator_Fz = 0;
	double derivator_Fx = 0, derivator_Fy = 0, derivator_Fz = 0;
	double Kp = 0, Ki = 0, Kd = 0;
	
	double error_Fx = 0, error_Fy = 0, error_Fz = 0;
	double prior_error_Fx = 0, prior_error_Fy = 0, prior_error_Fz = 0;
	double u_Fx = 0, u_Fy = 0, u_Fz = 0;
	
	//CONTROL SYSTEM FOR ROTATIONAL TORQUES
	double integrator_Tx = 0, integrator_Ty = 0, integrator_Tz = 0;
	double derivator_Tx = 0, derivator_Ty = 0, derivator_Tz = 0;
	double Kp_T = 0, Ki_T = 0, Kd_T = 0;
	
	double error_Tx = 0, error_Ty = 0, error_Tz = 0;
	double prior_error_Tx = 0, prior_error_Ty = 0, prior_error_Tz = 0;
	double u_Tx = 0, u_Ty = 0, u_Tz = 0;
	
	//=======================================================================
	double references[6] = {0,0,0,0,0,0}; 					//ref_Fx, ref_Fy, ref_Fz, ref_Tx, ref_Ty, ref_Tz
	double speed[6] = {0,0,0,0,0,0};
	double vw[6] = {0,0,0,0,0,0};

	//END-EFFECTOR TOOL BIAS
	double bias_tool_WF[3] = {0, 0, -1.472};				//-1.472[N] is the calculated gravitational force on the end-effector tool in world frame ('_WF').
	double bias_tool_TF[3]; 								//Gravitational force of end-effector tool in tool frame ('_TF').

	//DEFINE JOINT POSITION, VELOCITY AND ACCELERATION
	std::vector<double> q = ur5->rt_interface_->robot_state_->getQActual();
	std::vector<double> qd = ur5->rt_interface_->robot_state_->getQdActual();


	double theta;
	double angle_max = M_PI/2;//unit [rad]
	
	
	double start_time = ur5->rt_interface_->robot_state_->getTime();
	std::vector<double> sq = ur5->rt_interface_->robot_state_->getQActual();
	vector_trans_base_tool(sq, bias_tool_WF, bias_tool_TF);


	//STATIC F/T MOUNTING BIAS
	double bias_mounting[3] = {rawFTdata[0]-bias_tool_TF[0]+1.5, rawFTdata[1]-bias_tool_TF[1], rawFTdata[2]-bias_tool_TF[2]};
	double bias_force[3];
	double bias_torque[3] = {rawFTdata[3], rawFTdata[4], rawFTdata[5]};
	
	//FORCE/TORQUE VECTOR ALLOCATION
	double forces[3];
	double torques[3];
	double force_tresh[3];
	double torque_tresh[3];

	//ADMITTANCE CONTROLLER VARIABLES
	double error_pos[3];
	double error_pos_tool[3];

	double x_acc;
	double y_acc;
	double z_acc;

	gsl_matrix *R = gsl_matrix_calloc(3,3);
	gsl_vector *O = gsl_vector_alloc(3);


	double apar[6] = {0,-0.42500,-0.39225,0,0,0};
	double dpar[6] = {0.089159,0,0,0.10915,0.09465,0.08313};


	tfrotype tfkin;
	R->data=tfkin.R;
	O->data=tfkin.O;

	ufwdkin(&tfkin,q.data(),apar,dpar);


	double DESIREDXPOS = tfkin.O[0];
	double DESIREDYPOS = tfkin.O[1];
	double DESIREDZPOS = tfkin.O[2];
	
	// HANDNESS TEST VARIABLES
	// gsl_vector *Rx = gsl_vector_alloc(3);
	// gsl_vector *Ry = gsl_vector_alloc(3);
	// gsl_vector *Rz = gsl_vector_alloc(3);

	// gsl_vector_set(Rx,0,gsl_matrix_get(R,0,0));
	// gsl_vector_set(Rx,1,gsl_matrix_get(R,0,1));
	// gsl_vector_set(Rx,2,gsl_matrix_get(R,0,2));

	// gsl_vector_set(Ry,0,gsl_matrix_get(R,1,0));
	// gsl_vector_set(Ry,1,gsl_matrix_get(R,1,1));
	// gsl_vector_set(Ry,2,gsl_matrix_get(R,1,2));

	// gsl_vector_set(Rz,0,gsl_matrix_get(R,2,0));
	// gsl_vector_set(Rz,1,gsl_matrix_get(R,2,1));
	// gsl_vector_set(Rz,2,gsl_matrix_get(R,2,2));

	// double handed;



  	//ARDUINO DATA VARIABLE/VECTOR ALLOCATION
	std::vector<string> ArduinoSplitString;
	std::string ArduinoString;
	double ArduinoFrequencyData;
	double ArduinoAccelerometerData;
	

	
	//MASS-SPRING-DAMPER COEFFICIENTS
	double desired_frequency;	
	double m = 1;
	double k; 					
	double crictical_damping;	
	double c;				
		
	//PID controller gain parameters
	Kp = 0.005;// Prefered between [0.005-0.006]
	//Ki = 0.00015; // Not prefered due to overshoot behaviour.
	//Kd = 0.08; // Not prefered due to noise amplification
	
	Kp_T = 0.5;// Prefered between [0.4-0.5]
	//Ki_T = 0.00007; // Not prefered due to steady-state error.
	//Kd_T = 0.65; // Not prefered due to noise amplification.

	std::string testString = "This,is,a,test";
	std::vector<string> v = split(testString, ',');
	std::cout << v[1] << endl;


	std::cout << "======================== FORCE CONTROL ACTIVE ========================" << std::endl;
	while(i<iter)
	{	
		if (i<=250){
			references[2] += 0.015;
		}
		else{
			references[2] = 0;
		}
		std::cout << i << endl;
		std::mutex msg_lock;
		std::unique_lock<std::mutex> locker(msg_lock);
		while (!ur5->rt_interface_->robot_state_->getDataPublished())
		{
			rt_msg_cond_->wait(locker);
		}
		
		time_stamp = ur5->rt_interface_->robot_state_->getTime();
		elaps_time = time_stamp-start_time;
				  
		//UPDATE JOINT POSITION, VELOCITY AND FWD KINEMATICS
		q = ur5->rt_interface_->robot_state_->getQActual();
		qd = ur5->rt_interface_->robot_state_->getQdActual();
		ufwdkin(&tfkin,q.data(),apar,dpar);

			
		//READ SERIAL DATA FROM ARDUINO

		ArduinoString = arduino.read();
		//std::cout << ArduinoString << endl;


	 	//SPLIT AND CONVERT DATA TO DOUBLE TYPE 
		if (ArduinoString.length() == 22 || ArduinoString.length() == 23 || ArduinoString.length() == 24 || ArduinoString.length() == 25 ){
			ArduinoSplitString = split(ArduinoString, ',');
			//std::cout << ArduinoSplitString[0] << endl;
			//std::cout << ArduinoSplitString[1] << endl;
			ArduinoFrequencyData = atof(ArduinoSplitString[0].c_str());
	 	 	ArduinoAccelerometerData = atof(ArduinoSplitString[1].c_str());
		}
	 	


	 	desired_frequency = ArduinoFrequencyData;
		k = pow(desired_frequency, 2)*m;
		crictical_damping = 2*sqrt(k*m);
		c = 1*crictical_damping;


		
		
		//Update equiptment gravity component.
		bias_tool_TF[0] = bias_tool_TF[1] = bias_tool_TF[2] = bias_force[0] = bias_force[1] = bias_force[2] = 0;
		vector_trans_base_tool(q, bias_tool_WF, bias_tool_TF);
		
		for(int j=0; j<3; j++)
		{
			bias_force[j] = bias_mounting[j] + bias_tool_TF[j];
		}
		
		
		//FORCES
		forces[0] = rawFTdata[0]-bias_force[0];
		forces[1] = rawFTdata[1]-bias_force[1];
		forces[2] = rawFTdata[2]-bias_force[2];
		


		//FORCE TRESHOLDING FOR REMOVING DRIFT AND SMOOTH START AND STOP
		force_tresh[0] = (1- exp(-pow(forces[0], 2)/pow(0.2, 2)))*forces[0];		 
		force_tresh[1] = (1- exp(-pow(forces[1], 2)/pow(0.2, 2)))*forces[1];
		force_tresh[2] = (1- exp(-pow(forces[2], 2)/pow(0.2, 2)))*forces[2];


		//HANDNESS TEST

		// handed = triple_scalar_product(Rx,Ry,Rz);
		// if (handed < 0){
		// 	std::cout << "RIGHT HANDED!!!" << endl;
		// 	break;
		// }
		// else if (i%100){
		// 	std::cout << "LEFT HANDED!!!" << endl;
		// }

		//FORCE ERROR UPDATES
		error_Fx = error_Fy = error_Fz = 0;

		error_Fx = references[0] + force_tresh[0]; //force_tresh_lp_x[i];
		error_Fy = references[1] + force_tresh[1]; //force_tresh_lp_y[i];
		error_Fz = references[2] + force_tresh[2]; //force_tresh_lp_y[i];

		//TORQUES
		torques[0] = rawFTdata[3]-bias_torque[0];
		torques[1] = rawFTdata[4]-bias_torque[1];
		torques[2] = rawFTdata[5]-bias_torque[2];
		 
		//TORQUE TRESHOLDING FOR REMOVING DRIFT AND SMOOTH START AND STOP
		torque_tresh[0] = (1- exp(-pow(torques[0], 2)/pow(0.3, 2)))*torques[0];		 
		torque_tresh[1] = (1- exp(-pow(torques[1], 2)/pow(0.3, 2)))*torques[1];
		torque_tresh[2] = (1- exp(-pow(torques[2], 2)/pow(0.3, 2)))*torques[2];

		//TORQUE ERROR UPDATES
		error_Tx  = error_Ty = error_Tz = 0;

		error_Tx = references[3] + torque_tresh[0];
		error_Ty = references[4] + torque_tresh[1];
   		error_Tz = references[5] + torque_tresh[2];


		theta = atan2(gsl_vector_get(O,1), gsl_vector_get(O,0));
   		
		
		//SAFETY MECHANISM 
		if(fabs(error_Fx) > 50 || fabs(error_Fy) > 50 || fabs(error_Fz) > 50)
		{
			std::cout << "============================= STOPPING! ============================" << std::endl;
			ur5->halt();
			std::cout << "Force levels too large - force control halted!" << std::endl;
			break;
		}
		
		if(fabs(error_Tx) > 8 || fabs(error_Ty) > 8 || fabs(error_Tz) > 8)
		{
			std::cout << "============================= STOPPING! ============================" << std::endl;
			ur5->halt();
			std::cout << "Torque levels too large - force control halted!" << std::endl;
			break;
		}
		
		if(fabs(q[3]) > fabs(sq[3])+angle_max*2 || fabs(q[4]) > fabs(sq[4])+angle_max*2 || fabs(q[5]) > fabs(sq[5])+angle_max*2)
		{
			std::cout << "============================= STOPPING! ============================" << std::endl;
			ur5->halt();
			std::cout << "Joint angle too large  - force control halted!" << std::endl;
			break;
		}
		
		
		//=============== CONTROLLER =====================
		//Translational forces - 3DOF
		integrator_Fx += error_Fx;//integrator_Fx + error_Fx*iteration_time;//
		integrator_Fy += error_Fy;//integrator_Fy + error_Fy*iteration_time;//
		integrator_Fz += error_Fz;//integrator_Fz + error_Fz*iteration_time;//
		
		derivator_Fx = error_Fx - prior_error_Fx;//(error_Fx - prior_error_Fx)/iteration_time;//
		derivator_Fy = error_Fy - prior_error_Fy;//(error_Fy - prior_error_Fy)/iteration_time;//
		derivator_Fz = error_Fz - prior_error_Fz;//(error_Fz - prior_error_Fz)/iteration_time;//
		
		u_Fx = Kp*error_Fx + Ki*integrator_Fx + Kd*derivator_Fx;
		u_Fy = Kp*error_Fy + Ki*integrator_Fy + Kd*derivator_Fy;
		u_Fz = Kp*error_Fz + Ki*integrator_Fz + Kd*derivator_Fz;

		//Rotational torques - 3DOF
		integrator_Tx += error_Tx;
		integrator_Ty += error_Ty;
		integrator_Tz += error_Tz;
		
		derivator_Tx = error_Tx - prior_error_Tx;
		derivator_Ty = error_Ty - prior_error_Ty;
		derivator_Tz = error_Tz - prior_error_Tz;
		
		u_Tx = Kp_T*error_Tx + Ki_T*integrator_Tx + Kd_T*derivator_Tx;
		u_Ty = Kp_T*error_Ty + Ki_T*integrator_Ty + Kd_T*derivator_Ty;
		u_Tz = Kp_T*error_Tz + Ki_T*integrator_Tz + Kd_T*derivator_Tz;
		
		
		//SAFETY MECHANISM 
		if(fabs(u_Fx) > 5 || fabs(u_Fy) > 5|| fabs(u_Fz) > 5)
		{
			std::cout << "============================= STOPPING! ============================" << std::endl;
			ur5->halt();
			std::cout << "Force control input too large - stopping force control!" << std::endl;
			break;
		}
		
		if(fabs(u_Tx) > 5 || fabs(u_Ty) > 5 || fabs(u_Tz) > 5)
		{
			std::cout << "============================= STOPPING! ============================" << std::endl;
			ur5->halt();
			std::cout << "Torque control input too large - stopping force control!" << std::endl;
			break;
		}
		

		
		//ADMITTANCE CONTROLLER ERROR 
		error_pos[0] = tfkin.O[0] - DESIREDXPOS; 
		error_pos[1] = tfkin.O[1] - DESIREDYPOS; 
		error_pos[2] = tfkin.O[2] - DESIREDZPOS; 
		
		error_pos_tool[0] = error_pos_tool[1] = error_pos_tool[2] = 0;
		vector_trans_base_tool(q, error_pos, error_pos_tool);

		// //ADMITTANCE CONTROLLER DYNAMICS
		x_acc = (1.0/m)*(-c*vw[0] - k*error_pos_tool[0] + force_tresh[0] + references[0]); 
		y_acc = (1.0/m)*(-c*vw[1] - k*error_pos_tool[1] + force_tresh[1] + references[1]); 
		z_acc = (1.0/m)*(-c*vw[2] - k*error_pos_tool[2] + force_tresh[2] + references[2]); 

		//SOLVE AND SEND TO MANIPULATOR
		vw[0] = 0;//vw[0] + x_acc*iteration_time; 
		vw[1] = 0;//vw[1] + y_acc*iteration_time;  
		vw[2] = vw[2] + z_acc*iteration_time;
		vw[3] = 0;//u_Tx;
		vw[4] = 0;//u_Ty;
		vw[5] = 0;//u_Tz;
		solveInverseJacobian(q, vw, speed);
		
		ur5->rt_interface_->robot_state_->setDataPublished();
		ur5->setSpeed(speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], 100);
		
		//SET PRIOR VARIABLES
		prior_error_Fx = error_Fx; 
		prior_error_Fy = error_Fy;
		prior_error_Fz = error_Fz;
		
		prior_error_Tx = error_Tx; 
		prior_error_Ty = error_Ty;
		prior_error_Tz = error_Tz;

		


		//DATA LOGGING
		//Currently 55 variables
		forcelog << elaps_time << " " << speed[0] << " " << speed[1] << " " 
		<< speed[2] << " " << speed[3] << " "
		<< speed[4] << " " << speed[5] << " " 
		<< q[0] << " " << q[1] << " " << q[2] << " " 
		<< q[3] << " " << q[4] << " " << q[5] << " " 
		<< rawFTdata[0] << " " << rawFTdata[1] << " " << rawFTdata[2] << " " 
		<< rawFTdata[3] << " " << rawFTdata[4] << " " << rawFTdata[5] << " " 
		<< forces[0] << " " << forces[1] << " " << forces[2] << " "
		<< torques[0] << " " << torques[1] << " " << torques[2] << " " 
		<< error_pos_tool[0] << " " << error_pos_tool[1] << " " << error_pos_tool[2] << " " 
		<< error_Tx << " " << error_Ty << " " << error_Tz << " " 
		<< references[0] << " " << references[1] << " " << references[2] << " " 
		<< u_Tx << " " << u_Ty << " " << u_Tz << " " 
		<< bias_force[0] << " " << bias_force[1] << " " << bias_force[2] << " "  
		<< bias_tool_TF[0] << " " << bias_tool_TF[1] << " " << bias_tool_TF[2] << " " 
		<< gsl_vector_get(O,0) << " " << gsl_vector_get(O,1) << " " 
		<< gsl_vector_get(O,2) << " " 
		<< gsl_matrix_get(R,0,0) << " " << gsl_matrix_get(R,0,1) << " " 
		<< gsl_matrix_get(R,0,2) << " " << gsl_matrix_get(R,1,0) << " " 
		<< gsl_matrix_get(R,1,1) << " " << gsl_matrix_get(R,1,2) << " " 
		<< gsl_matrix_get(R,2,0) << " " << gsl_matrix_get(R,2,1) << " " 
		<< gsl_matrix_get(R,2,2) << " " << "\n\n\n\n";
		
		
		accelerolog << ArduinoAccelerometerData << endl;
		frequencylog << ArduinoFrequencyData << endl;

		i += 1;
		usleep(iteration_sleeptime);

		// std::cout << "(R,0,0): " << gsl_matrix_get(R,0,0) << endl;
		// std::cout << "(R,0,1): " << gsl_matrix_get(R,0,1) << endl;
		// std::cout << "(R,0,2): " << gsl_matrix_get(R,0,2) << endl << endl;

		// std::cout << "(R,1,0): " << gsl_matrix_get(R,1,0) << endl;
		// std::cout << "(R,1,1): " << gsl_matrix_get(R,1,1) << endl;
		// std::cout << "(R,1,2): " << gsl_matrix_get(R,1,2) << endl << endl;

		// std::cout << "(R,2,0): " << gsl_matrix_get(R,2,0) << endl;
		// std::cout << "(R,2,1): " << gsl_matrix_get(R,2,1) << endl;
		// std::cout << "(R,2,2): " << gsl_matrix_get(R,2,2) << endl << endl;


		// std::cout << "q[0]" << q[0] << endl;
		// std::cout << "q[1]" << q[1] << endl;
		// std::cout << "q[2]" << q[2] << endl;
		// std::cout << "q[3]" << q[3] << endl;
		// std::cout << "q[4]" << q[4] << endl;
		// std::cout << "q[5]" << q[5] << endl;

	}	
	
	//stopFT(&forceID);
	
	forcelog.close();
	accelerolog.close();
	frequencylog.close();
}