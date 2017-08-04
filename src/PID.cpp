#include "PID.h"
#include <numeric>
#include <iostream>
using namespace std;
#define TOLERANCE 1
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	this->Kd = Kd_;
	this->Ki = Ki_;
	this->Kp = Kp_;

	this->d_error = 0;
	this->i_error = 0;
	this->p_error = 0;

	p.push_back(0);
	p.push_back(0);
	p.push_back(0);

	dp.push_back(1.0);
	dp.push_back(1.0);
	dp.push_back(1.0);
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

double PID::TotalError(std::vector<double> par) {
	return -par[0] * p_error - par[1] * d_error - par[2] * i_error;
}

void PID::Twiddle(const double cte) {


	//UpdateError(cte);
	double best_error = this->TotalError(p);
	//std::cout << " inital best_error = " << best_error << std::endl;
	int i = 0;
 	while ((dp[0] + dp[1] + dp[2]) > TOLERANCE) {
 		//std::cout << "1p[0]= " << p[0] << " p[1]=" << p[1] << " p[2]=" << p[2] << endl;
		//for(int i = 0; i < p.size(); i++)
 		if (i > 2)
 			i = 0;
		{
			p[i] += dp[i];

			//UpdateError(cte);
			double error = this->TotalError(p);
			//std::cout << " 1 error = " << error << std::endl;
			//std::cout << "err1=" << error << endl;
			if (error < best_error) {
				best_error = error;
				//std::cout << " 1 best_error = " << best_error << std::endl;
				dp[i] *= 1.1;
			}
			else {
				p[i] -= 2 * dp[i];
				//UpdateError(cte);
				error = this->TotalError(p);
				//std::cout << " 2 error = " << error << std::endl;
				//std::cout << "err2=" << error << endl;
				if (error < best_error) {
					best_error = error;
					//std::cout << " 1 best_error = " << best_error << std::endl;
					dp[i] *= 1.1;
				}
				else {
					p[i] += dp[i];
					dp[i] *= 0.9;
					//std::cout << "dp[0]= " << dp[0] << " dp[1]=" << dp[1] << " dp[2]=" << dp[2] << endl;
				}
			}
		}
		i++;
		//std::cout << "2p[0]= " << p[0] << " p[1]=" << p[1] << " p[2]=" << p[2] << endl;
	}
 	//std::cout << "here" << endl;
// 	std::cout << "p[0]= " << p[0] << " p[1]=" << p[1] << " p[2]=" << p[2] << endl;
// 	std::cout << "dp[0]= " << dp[0] << " dp[1]=" << dp[1] << " dp[2]=" << dp[2] << endl;
	Kp = p[0];
	Kd = p[1];
	Ki = p[2];

	p[0] = 0;
	p[1] = 0;
	p[2] = 0;

	dp[0] = 1;
	dp[1] = 1;
	dp[2] = 1;
}
void PID::changeP(const int idx, const double dVal) {
	p[idx] += dVal;
	switch(idx) {
		case 0:
			Kp = p[idx];
			break;
		case 1:
			Kd = p[idx];
			break;
		case 2:
			Ki = p[idx];
			break;
		default:
			break;
	}
}

void PID::changeDp(const int idx, const double dVal) {
	dp[idx] *= dVal;
}

bool PID::isTwiddle() {
	int dp_tol = std::accumulate(dp.begin(), dp.end(), 0);
	if (dp[0] + dp[1] + dp[2] > TOLERANCE) {
		std::cout << "dp0=" << dp[0] << " dp[1]=" << dp[1] << "dp[2]=" << dp[2] << std::endl;
		std::cout << "p[0]=" << dp[0] << " p[1]=" << p[1] << " p[2]=" << p[2] << std::endl;
		std::cout << "Still twiddle=" << dp_tol << std::endl;
		return true;
	}
	else {
		std::cout << "Twiddle Finished!" << std::endl;
		Kp = p[0];
		Kd = p[1];
		Ki = p[2];
		std::cout << "Kp=" << Kp << " Kd=" << Kd << " Ki=" << Ki << std::endl;

		p.clear();
		p.push_back(0);
		p.push_back(0);
		p.push_back(0);

		dp.clear();
		dp.push_back(1.0);
		dp.push_back(1.0);
		dp.push_back(1.0);

		return false;
	}
}

double PID::getDpValue(const int idx) {
	return this->dp[idx];
}

