/*
MIT License

Copyright (c) 2023 Eyal Porat

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "string.h"
#include "nav_kalman.h"

#define sq(x) ((x)*(x))


void inverse2x2(double mat[2][2], double result[2][2])
{
	//calculating the inverse matrix using the adjoint matrix properties
	
	//calculating the determinant of the matrix
	double inv_det = 1.0 / (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1] + 1e-7);
	
	//dividing the adjoint matrix with the determinant
	result[0][0] = mat[1][1] * inv_det;
	result[0][1] = -mat[1][0] * inv_det;
	result[1][0] = -mat[0][1] * inv_det;
	result[1][1] = mat[0][0] * inv_det;
}



navKalman::navKalman(double R_pos, double R_vel, double a_std)
{
	memset(this, 0, sizeof(navKalman));

	//assigning the accelerometer error values from the datasheet
	Q[0][0] = sq(a_std);
	Q[1][1] = sq(a_std);
	Q[0][1] = Q[1][0] = 0;
	
	//assigning the GNSS error values from the datasheet
	R[0][0] = R_pos;
	R[1][1] = R_vel;
	R[1][0] = R[0][1] = 0;
}

void navKalman::accel_update(double Dt, float accel)
{
  
  U_hat[0] = U_hat[0] + U_hat[1] * Dt + 0.5 * accel * sq(Dt);

  U_hat[1] = U_hat[1] + accel * Dt;
	
	
	double P_k_1[2][2]; //P k-1
	
	P_k_1[0][0] = P[0][0];
	P_k_1[0][1] = P[0][1];
	P_k_1[1][0] = P[1][0];
	P_k_1[1][1] = P[1][1];
	
	P[0][0] = P_k_1[0][0] + Dt * P_k_1[1][0] + Dt * P_k_1[0][1] + sq(Dt) * P_k_1[1][1] + Q[0][0];
  //P[0][1] = P_k_1[0][1] + Dt * P_k_1[1][1] + Q[0][1];
  //P[1][0] = P_k_1[1][0] + Dt * P_k_1[1][1] + Q[1][0];
  P[1][1] = P_k_1[1][1] + Q[1][1];
	
	
	
}

void navKalman::GNSS_update(double coord, double vel)
{
	double temp[2][2];
	
	temp[0][0] = P[0][0] + R[0][0];
	temp[0][1] = P[0][1] + R[0][1];
	temp[1][0] = P[1][0] + R[1][0];
	temp[1][1] = P[1][1] + R[1][1];
	
	double invrs[2][2];
	
	//calculating the inverse of the P+R matrix
	inverse2x2(temp, invrs);
	
	//updating the Kalman gains, trearing the diagonal as zero
	K[0][0] = P[0][0] * invrs[0][0] + P[0][1] * invrs[1][0];
	//K[0][1] = P[0][0] * invrs[0][1] + P[0][1] * invrs[1][1];
	//K[1][0] = P[1][0] * invrs[0][0] + P[1][1] * invrs[1][0];
	K[1][1] = P[1][0] * invrs[0][1] + P[1][1] * invrs[1][1];
	
	
	//update the predictions based on the GNSS readings
	U_hat[0] += K[0][0] * (coord - U_hat[0]) + K[0][1] * (coord - U_hat[0]);
	U_hat[1] += K[1][0] * (vel - U_hat[1]) + K[1][1] * (vel - U_hat[1]);
	
	
	double P_k_1[2][2]; //P k-1
	
	P_k_1[0][0] = P[0][0];
	P_k_1[0][1] = P[0][1];
	P_k_1[1][0] = P[1][0];
	P_k_1[1][1] = P[1][1];
	
	//calculating the error covariance estimation, trearing the diagonal as zero
	P[0][0] = (1 - K[0][0]) * P_k_1[0][0] - K[0][1] * P_k_1[1][0];
	//P[0][1] = (1 - K[0][0]) * P_k_1[0][1] - K[0][1] * P_k_1[1][1];
	//P[1][0] = (-K[1][0]) * P_k_1[0][0] + (1 - K[1][1]) * P_k_1[1][0];
	P[1][1] = (-K[1][0]) * P_k_1[0][1] + (1 - K[1][1]) * P_k_1[1][1];
	
}
