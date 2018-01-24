#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class MPC {
 public:
     //const double Lf = 2.67;
  MPC();

  virtual ~MPC();
size_t getN();
double getDt();
void setRefV(double desiredV);
void init(size_t pN, double pDt);
 // void init(size_t N, double dt);
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
