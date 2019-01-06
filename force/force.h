#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>
#include <math.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>

#include "../ur_modern_driver-master/include/ur_driver.h"
#include "../kinematics/ur_kin.h"
#include "../cArduino-master/cArduino.h"

#define PORT 49152 
#define COMMAND 2 
#define NUM_SAMPLES 0

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct response_struct 
{
  uint32 rdt_sequence;
  uint32 ft_sequence;
  uint32 status;
  int32 FTData[6];
} RESPONSE;

typedef struct
{
  double R[9];
  double O[3];
} tfrotype;

void *getFTData(void *arg);
void startFT(pthread_t *forceID);
void stopFT(pthread_t *forceID);

void vector_trans_base_tool(std::vector<double> q, double vector_in[3], double vector_out[3]);
void vector_trans_tool_base(std::vector<double> q, double vector_in[3], double vector_out[3]);
void cross_product(const gsl_vector *u, const gsl_vector *v, gsl_vector *product);
double triple_scalar_product(const gsl_vector *u, const gsl_vector *v, const gsl_vector *w);
const vector<string> explode(const string& s, const char& c);
void forceControl(UrDriver *ur5, std::condition_variable *rt_msg_cond_, int run_time);
void rotate(gsl_vector *res,gsl_matrix *R, gsl_vector *inp,gsl_vector *t1,gsl_vector *t2);
void solveInverseJacobian(std::vector<double> q, double vw[6], double qd[6]);
