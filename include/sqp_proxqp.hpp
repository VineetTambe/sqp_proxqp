/**
 * Author: Vineet Tambe
 * Carnegie Mellon University Robotics Institute
 * Date: 10/18/2018
 */

#include <iostream>
#include <proxsuite/proxqp/sparse/sparse.hpp>  // get the sparse API of ProxQP
#include <Eigen/Core>
#include <proxsuite/helpers/optional.hpp>  // for c++14
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <utility>

using namespace proxsuite;
using namespace proxsuite::proxqp;
using proxsuite::nullopt;  // c++17 simply use std::nullopt

namespace sqp_proxqp
{

// template <typename T>
// class SQP_ProxQP;

template <typename T>
class SQP_ProxQP
{
private:
  // QP setup parameters
  isize dim_, n_eq_, n_in_;
  int iters_;

public:
  SQP_ProxQP();
  ~SQP_ProxQP() = default;
  SQP_ProxQP(long dim, long n_eq, long n_in, long iters);
  // -------------------
  // QP params
  // cost H
  Eigen::MatrixXd H_;
  Eigen::MatrixXd g__;

  // equality constraints
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  // inequality constraints
  Eigen::MatrixXd C_;
  Eigen::VectorXd l_;
  Eigen::VectorXd u_;

  // sparse matrics of H, C, A
  Eigen::SparseMatrix<T> H_spa_;
  Eigen::SparseMatrix<T> C_spa_;
  Eigen::SparseMatrix<T> A_spa_;

  void setQPParams(Eigen::MatrixXd H, Eigen::MatrixXd g, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::MatrixXd C,
                   Eigen::VectorXd l, Eigen::VectorXd u, Eigen::VectorXd x);
  void solveQP(Eigen::VectorXd& primal, Eigen::VectorXd& dual);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> runQP();
  // proxsuite::proxqp::sparse::QP<T, int> getQPObject();
  // void setQPObject(proxsuite::proxqp::sparse::QP<T, int>& qp);
};

extern template class SQP_ProxQP<double>;

}  // namespace sqp_proxqp