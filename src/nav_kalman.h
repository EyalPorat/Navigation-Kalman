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


#pragma once

struct navKalman
{
	navKalman(double P_pos, double P_vel, double a_std);

	//The estimated error covariances
	double P[2][2];
	//The accelerometer covariance matrix (hyperparameter)
	double Q[2][2];
	//The Kalman gains
	double K[2][2];
	//The predicted values
	double U_hat[2];
	//The GNSS covariance matrix (hyperparameter)
	double R[2][2];
	
	void GNSS_update(double coord, double vel);
	void accel_update(double Dt, float accel);

};
