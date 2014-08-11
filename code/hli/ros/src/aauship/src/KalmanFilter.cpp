// // Jeppe forsoeger at imp kalmanFilter
// #include "ros/ros.h"
// #include "aauship/ADIS16405.h"
// #include <math.h>'
// 
// KalmanFilter::KalmanFilter(void){
// 	PHI = [];
// 
// 
// 
// }
// 
// void KalmanFilter::KalmanFilterUpdate(){
// 
// 
// 
// }

/* example-009.cc
*
* Copyright (C) 2011-2014 Jeremy Fix
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or (at
* your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/
/* In this example, we use EKF for state/parameter estimation in order to estimate the state and parameters of a Lorentz attractor */
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include "ukf.h"
#include <sys/time.h>
using namespace ekf;
//#define SIGMA 10.0
//#define RHO 28.0
//#define BETA 8.0/3.0
//#define DT 0.01
//#define X0 0.1
//#define Y0 1.2
//#define Z0 0.4
#define NOISE_AMPLITUDE 2.0
#define VERBOSE true
/*************************************************************************************/
/* Definition of the evolution and observation functions */
/*************************************************************************************/
// Evolution function
void f(gsl_vector * params, gsl_vector * xk_1, gsl_vector * xk)
{
double x = gsl_vector_get(xk_1,0);
double y = gsl_vector_get(xk_1,1);
double phi = gsl_vector_get(xk_1,2);
double theta = gsl_vector_get(xk_1,3);
double psi = gsl_vector_get(xk_1,4);
double u = gsl_vector_get(xk_1,5);
double v = gsl_vector_get(xk_1,6);
double roll = gsl_vector_get(xk_1,7);
double pitch = gsl_vector_get(xk_1,8);
double yaw = gsl_vector_get(xk_1,9);
double dt = gsl_vector_get(params, 0);
gsl_vector_set(xk, 0, x);
gsl_vector_set(xk, 1, y);
gsl_vector_set(xk, 2, phi);
gsl_vector_set(xk, 3, theta);
gsl_vector_set(xk, 4, psi);
gsl_vector_set(xk, 5, u);
gsl_vector_set(xk, 6, v);
gsl_vector_set(xk, 7, roll);
gsl_vector_set(xk, 8, pitch);
gsl_vector_set(xk, 9, yaw);
}
// Jacobian of the evolution function
void df(gsl_vector * params, gsl_vector * xk_1, gsl_matrix * Fxk)
{
double x = gsl_vector_get(xk_1,0);
double y = gsl_vector_get(xk_1,1);
double phi = gsl_vector_get(xk_1,2);
double theta = gsl_vector_get(xk_1,3);
double psi = gsl_vector_get(xk_1,4);
double u = gsl_vector_get(xk_1,5);
double v = gsl_vector_get(xk_1,6);
double roll = gsl_vector_get(xk_1,7);
double pitch = gsl_vector_get(xk_1,8);
double yaw = gsl_vector_get(xk_1,9);
double dt = gsl_vector_get(params, 0);
// Derivatives for x(t+1)
gsl_matrix_set(Fxk, 0, 0, 0.0);
gsl_matrix_set(Fxk, 0, 1, 0.0);
gsl_matrix_set(Fxk, 0, 2, 0.0);
gsl_matrix_set(Fxk, 0, 3, 0.0);
gsl_matrix_set(Fxk, 0, 4, 0.0);
gsl_matrix_set(Fxk, 0, 5, 1.0);
gsl_matrix_set(Fxk, 0, 6, 1.0);
gsl_matrix_set(Fxk, 0, 7, 1.0);
gsl_matrix_set(Fxk, 0, 8, 1.0);
gsl_matrix_set(Fxk, 0, 9, 1.0);
// Derivatives for y(t+1)
gsl_matrix_set(Fxk, 1, 0, 0.0);
gsl_matrix_set(Fxk, 1, 1, 0.0);
gsl_matrix_set(Fxk, 1, 2, 0.0);
gsl_matrix_set(Fxk, 1, 3, 0.0);
gsl_matrix_set(Fxk, 1, 4, 0.0);
gsl_matrix_set(Fxk, 1, 5, 1.0);
gsl_matrix_set(Fxk, 1, 6, 1.0);
gsl_matrix_set(Fxk, 1, 7, 1.0);
gsl_matrix_set(Fxk, 1, 8, 1.0);
gsl_matrix_set(Fxk, 1, 9, 1.0);
// Derivatives for phi(t+1)
gsl_matrix_set(Fxk, 2, 0, 0.0);
gsl_matrix_set(Fxk, 2, 1, 0.0);
gsl_matrix_set(Fxk, 2, 2, 0.0);
gsl_matrix_set(Fxk, 2, 3, 0.0);
gsl_matrix_set(Fxk, 2, 4, 0.0);
gsl_matrix_set(Fxk, 2, 5, 1.0);
gsl_matrix_set(Fxk, 2, 6, 1.0);
gsl_matrix_set(Fxk, 2, 7, 1.0);
gsl_matrix_set(Fxk, 2, 8, 1.0);
gsl_matrix_set(Fxk, 2, 9, 1.0);
// Derivatives for theta(t+1)
gsl_matrix_set(Fxk, 3, 0, 0.0);
gsl_matrix_set(Fxk, 3, 1, 0.0);
gsl_matrix_set(Fxk, 3, 2, 0.0);
gsl_matrix_set(Fxk, 3, 3, 0.0);
gsl_matrix_set(Fxk, 3, 4, 0.0);
gsl_matrix_set(Fxk, 3, 5, 1.0);
gsl_matrix_set(Fxk, 3, 6, 1.0);
gsl_matrix_set(Fxk, 3, 7, 1.0);
gsl_matrix_set(Fxk, 3, 8, 1.0);
gsl_matrix_set(Fxk, 3, 9, 1.0);
// Derivatives for psi(t+1)
gsl_matrix_set(Fxk, 4, 0, 0.0);
gsl_matrix_set(Fxk, 4, 1, 0.0);
gsl_matrix_set(Fxk, 4, 2, 0.0);
gsl_matrix_set(Fxk, 4, 3, 0.0);
gsl_matrix_set(Fxk, 4, 4, 0.0);
gsl_matrix_set(Fxk, 4, 5, 1.0);
gsl_matrix_set(Fxk, 4, 6, 1.0);
gsl_matrix_set(Fxk, 4, 7, 1.0);
gsl_matrix_set(Fxk, 4, 8, 1.0);
gsl_matrix_set(Fxk, 4, 9, 1.0);
// Derivatives for u(t+1)
gsl_matrix_set(Fxk, 5, 0, 0.0);
gsl_matrix_set(Fxk, 5, 1, 0.0);
gsl_matrix_set(Fxk, 5, 2, 0.0);
gsl_matrix_set(Fxk, 5, 3, 0.0);
gsl_matrix_set(Fxk, 5, 4, 0.0);
gsl_matrix_set(Fxk, 5, 5, 1.0);
gsl_matrix_set(Fxk, 5, 6, 1.0);
gsl_matrix_set(Fxk, 5, 7, 1.0);
gsl_matrix_set(Fxk, 5, 8, 1.0);
gsl_matrix_set(Fxk, 5, 9, 1.0);
// Derivatives for v(t+1)
gsl_matrix_set(Fxk, 6, 0, 0.0);
gsl_matrix_set(Fxk, 6, 1, 0.0);
gsl_matrix_set(Fxk, 6, 2, 0.0);
gsl_matrix_set(Fxk, 6, 3, 0.0);
gsl_matrix_set(Fxk, 6, 4, 0.0);
gsl_matrix_set(Fxk, 6, 5, 1.0);
gsl_matrix_set(Fxk, 6, 6, 1.0);
gsl_matrix_set(Fxk, 6, 7, 1.0);
gsl_matrix_set(Fxk, 6, 8, 1.0);
gsl_matrix_set(Fxk, 6, 9, 1.0);
// Derivatives for roll(t+1)
gsl_matrix_set(Fxk, 7, 0, 0.0);
gsl_matrix_set(Fxk, 7, 1, 0.0);
gsl_matrix_set(Fxk, 7, 2, 0.0);
gsl_matrix_set(Fxk, 7, 3, 0.0);
gsl_matrix_set(Fxk, 7, 4, 0.0);
gsl_matrix_set(Fxk, 7, 5, 1.0);
gsl_matrix_set(Fxk, 7, 6, 1.0);
gsl_matrix_set(Fxk, 7, 7, 1.0);
gsl_matrix_set(Fxk, 7, 8, 1.0);
gsl_matrix_set(Fxk, 7, 9, 1.0);
// Derivatives for pitch(t+1)
gsl_matrix_set(Fxk, 8, 0, 0.0);
gsl_matrix_set(Fxk, 8, 1, 0.0);
gsl_matrix_set(Fxk, 8, 2, 0.0);
gsl_matrix_set(Fxk, 8, 3, 0.0);
gsl_matrix_set(Fxk, 8, 4, 0.0);
gsl_matrix_set(Fxk, 8, 5, 1.0);
gsl_matrix_set(Fxk, 8, 6, 1.0);
gsl_matrix_set(Fxk, 8, 7, 1.0);
gsl_matrix_set(Fxk, 8, 8, 1.0);
gsl_matrix_set(Fxk, 8, 9, 1.0);
// Derivatives for yaw(t+1)
gsl_matrix_set(Fxk, 9, 0, 0.0);
gsl_matrix_set(Fxk, 9, 1, 0.0);
gsl_matrix_set(Fxk, 9, 2, 0.0);
gsl_matrix_set(Fxk, 9, 3, 0.0);
gsl_matrix_set(Fxk, 9, 4, 0.0);
gsl_matrix_set(Fxk, 9, 5, 1.0);
gsl_matrix_set(Fxk, 9, 6, 1.0);
gsl_matrix_set(Fxk, 9, 7, 1.0);
gsl_matrix_set(Fxk, 9, 8, 1.0);
gsl_matrix_set(Fxk, 9, 9, 1.0);
}
// Observation function
void h(gsl_vector * params, gsl_vector * xk , gsl_vector * yk)
{
for(unsigned int i = 0 ; i < yk->size ; ++i)
gsl_vector_set(yk, i, gsl_vector_get(xk,i) + NOISE_AMPLITUDE*rand()/ double(RAND_MAX));
}
// Jacobian of the observation function
void dh(gsl_vector * params, gsl_vector * xk , gsl_matrix * Hyk)
{
gsl_matrix_set_zero(Hyk);
gsl_matrix_set(Hyk, 0, 0, 1.0);
gsl_matrix_set(Hyk, 1, 1, 1.0);
gsl_matrix_set(Hyk, 2, 2, 1.0);
}
/*****************************************************/
/* main */
/*****************************************************/
int main(int argc, char* argv[]) {
struct timeval before, after;
srand(time(NULL));
// Definition of the parameters and state variables
ekf_param p;
ekf_state s;
// The parameters for the evolution equation
s.params = gsl_vector_alloc(1);
s.params->data[0] = DT;
// Initialization of the parameters
p.n = 6;
p.no = 3;
//EvolutionNoise * evolution_noise = new ekf::EvolutionAnneal(1e-2, 0.99, 1e-8);
EvolutionNoise * evolution_noise = new ekf::EvolutionRLS(1e-2, 0.9995);
//EvolutionNoise * evolution_noise = new ekf::EvolutionRobbinsMonro(1e-5, 1e-6);
p.evolution_noise = evolution_noise;
p.observation_noise = 1e-1;
p.prior_pk = 1.0;
p.observation_gradient_is_diagonal = true;
// Initialization of the state and parameters
ekf_init(p,s);
// Initialize the parameter vector to some random values
for(int i = 0 ; i < p.n ; i++)
gsl_vector_set(s.xk,i,5.0*(2.0*rand()/double(RAND_MAX-1)-1.0));
s.xk->data[0] = -15.0;
s.xk->data[1] = -15.0;
s.xk->data[2] = -15.0;
// Allocate the input/output vectors
gsl_vector * xi = gsl_vector_alloc(p.n);
gsl_vector * yi = gsl_vector_alloc(p.no);
gsl_vector_set_zero(yi);
/***********************************************/
/***** Iterate the learning on the samples *****/
/***********************************************/
int epoch = 0;
xi->data[0] = X0;
xi->data[1] = Y0;
xi->data[2] = Z0;
xi->data[3] = SIGMA;
xi->data[4] = RHO;
xi->data[5] = BETA;
std::ofstream outfile("example-009.data");
std::ofstream outfile_rms("example-009-rms.data");
double rms= 0.0;
double errorBound = NOISE_AMPLITUDE/2.0;
int count_time=0;
int nb_steps_below_error=1000;
//int is_counting=0;
gettimeofday(&before, NULL);
outfile << epoch << " ";
for(int i = 0 ; i < 6 ; ++i)
outfile << xi->data[i] << " " ;
for(int i = 0 ; i < 3 ; ++i)
outfile << yi->data[i] << " " ;
for(int i = 0 ; i < 6 ; ++i)
outfile << s.xk->data[i] << " " ;
outfile << std::endl;
while( epoch < 8000)
{
//printf("Epoch %i \n", epoch);
// Evaluate the true dynamical system
f(s.params, xi, xi);
h(s.params, xi, yi);
// Provide the observation and iterate
ekf_iterate(p,s,f,df,h,dh,yi);
epoch++;
rms = 0.0;
for(int i = 0 ; i < p.n ; ++i)
rms += gsl_pow_2(xi->data[i] - s.xk->data[i]);
rms /= double(p.n);
rms = sqrt(rms);
if(rms <= errorBound)
{
count_time++;
if(count_time >= nb_steps_below_error)
break;
}
else
{
count_time = 0;
}
printf("[%i] True state : %e %e %e %e %e %e, estimated : %e %e %e %e %e %e\n", epoch,
xi->data[0],
xi->data[1],
xi->data[2],
xi->data[3],
xi->data[4],
xi->data[5],
s.xk->data[0],
s.xk->data[1],
s.xk->data[2],
s.xk->data[3],
s.xk->data[4],
s.xk->data[5]);
outfile << epoch << " ";
for(int i = 0 ; i < 6 ; ++i)
outfile << xi->data[i] << " " ;
for(int i = 0 ; i < 3 ; ++i)
outfile << yi->data[i] << " " ;
for(int i = 0 ; i < 6 ; ++i)
outfile << s.xk->data[i] << " " ;
outfile << std::endl;
outfile_rms << rms << std::endl;
}
gettimeofday(&after, NULL);
double sbefore = before.tv_sec + before.tv_usec * 1E-6;
double safter = after.tv_sec + after.tv_usec * 1E-6;
double total = safter - sbefore;
outfile.close();
outfile_rms.close();
std::cout << " Run on " << epoch << " epochs ;" << std::scientific << total / double(epoch) << " s./step " << std::endl;
printf("I found the following parameters : %e %e %e ; The true parameters being : %e %e %e \n", s.xk->data[3], s.xk->data[4],s.xk->data[5],SIGMA, RHO, BETA);
std::cout << " Outputs are saved in example-009*.data " << std::endl;
std::cout << " You can plot them using e.g. gnuplot : " << std::endl;
//std::cout << " gnuplot Data/plot-example-009.gplot ; gv Output/example-009-rms.ps ; gv Output/example-009-Lorentz.ps " << std::endl;
/***********************************************/
/**** Free the memory ****/
/***********************************************/
ekf_free(p,s);
}