#ifndef PID_H
#define PID_H
#include <vector>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  std::vector<double> p;
  std::vector<double> dp;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  double TotalError(std::vector<double> par);

  /*
   * Twiddle algorithm for parameters optimization
   */
  void Twiddle(const double cte);
  bool isTwiddle();
  void changeP(const int idx, const double dVal);
  void changeDp(const int idx, const double dVal);
  double getDpValue(const int idx);
  bool isErrorHigh();
};

#endif /* PID_H */
