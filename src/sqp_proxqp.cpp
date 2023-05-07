
/**
 * Author: Vineet Tambe
 * Carnegie Mellon University Robotics Institute
 * Date: 10/18/2018
 */

#include <sqp_proxqp.hpp>

namespace sqp_proxqp
{

template <typename T>
SQP_ProxQP<T>::SQP_ProxQP(long dim, long n_eq, long n_in, long iters)
{
  dim_ = dim;
  n_eq_ = n_eq;
  n_in_ = n_in;
}

template <typename T>
void SQP_ProxQP<T>::setQPParams(Eigen::MatrixXd H, Eigen::MatrixXd g, Eigen::MatrixXd A, Eigen::VectorXd b,
                                Eigen::MatrixXd C, Eigen::VectorXd l, Eigen::VectorXd u, Eigen::VectorXd x)
{
  H_ = H;
  H_spa_ = H.sparseView();

  g__ = g;

  A_ = A;
  A_spa_ = A.sparseView();
  b_ = A_spa_ * x;

  C_ = C;
  C_spa_ = C.sparseView();

  l_ = C_spa_ * x;
  u_ = u;

  return;
}

// TODO
template <typename T>
void SQP_ProxQP<T>::solveQP(Eigen::VectorXd& primal, Eigen::VectorXd& dual)
{
  for (int i = 0; i < iters_; i++)
  {
    std::cout << "iteration: " << i << "\n";
    // simulate forward step -- this has been reduced to only update the variables that change

    // TODO compute Hessian H
    // TODO compute gradient g
    // TODO update equality constraints A, b
    // TODO update inequality constraints C, l, u

    //  TODO setQPParams(H, g, A, b, C, l, u);

    // TODO  runQP(primmal, dual);
  }
  std::cout << "Final result:\n";
  std::cout << "primal = " << primal << std::endl;
  std::cout << "dual = " << dual << std::endl;
}

template <typename T>
std::pair<Eigen::VectorXd, Eigen::VectorXd> SQP_ProxQP<T>::runQP(proxsuite::proxqp::sparse::QP<T, int>& qp_)
{
  std::pair<Eigen::VectorXd, Eigen::VectorXd> result;

  qp_.init(H_spa_, g__, A_spa_, b_, C_spa_, l_, u_);
  qp_.solve();

  if (qp_.results.info.status == QPSolverOutput::PROXQP_MAX_ITER_REACHED)
  {
    std::cout << "ProxQP PROXQP_MAX_ITER_REACHED\n";
  }

  result.first.resize(qp_.results.x.size());
  result.first << qp_.results.x;
  result.second.resize(qp_.results.y.size() + qp_.results.z.size());
  result.second << qp_.results.y, qp_.results.z;

  return result;
}

// template <typename T>
// proxsuite::proxqp::sparse::QP<T, int> SQP_ProxQP<T>::getQPObject()
// {
//   return qp_;
// }

// template <typename T>
// void SQP_ProxQP<T>::setQPObject(proxsuite::proxqp::sparse::QP<T, int>& qp)
// {
//   qp_ = qp;
// }

template class SQP_ProxQP<double>;
// template class SQP_ProxQP<float>;
}  // namespace sqp_proxqp
