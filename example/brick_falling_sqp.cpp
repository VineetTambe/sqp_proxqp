#include <sqp_proxqp.hpp>

using namespace sqp_proxqp;
using T = double;
using proxsuite::nullopt;  // c++17 simply use std::nullopt

int main()
{
  long dim = 2, n_eq = 0, n_in = 1;

  int mass = 1.0;
  double dt = 0.01;
  double Time = 5.0;
  long iters = (long)Time / dt;

  SQP_ProxQP<T> qp_solver(dim, n_eq, n_in, iters);

  Eigen::MatrixXd vk = Eigen::MatrixXd(dim, 1);
  vk << 1.0, 4.0;  // set initial conditions
  Eigen::VectorXd dual = Eigen::VectorXd(1);

  Eigen::MatrixXd qk = Eigen::MatrixXd(dim, 1);
  qk << 0.0, 1.0;  // set initial conditions x = 0 y = 1 final state should be some translation in x and y = 0

  double eps_abs = 1e-9;

  Eigen::MatrixXd gravity = Eigen::MatrixXd(dim, 1);
  gravity << 0.0, 9.81;

  Eigen::MatrixXd J = Eigen::MatrixXd(1, dim);
  J << 0.0, 1.0;

  Eigen::MatrixXd mass_matrix = Eigen::MatrixXd(dim, dim);
  mass_matrix << 1.0, 0.0, 0.0, 1.0;
  mass_matrix = mass * mass_matrix;

  // TODO the following line should replace the for loop
  // qp_solver.solveQP(x, dual);

  Eigen::MatrixXd A = Eigen::MatrixXd(n_eq, dim);
  Eigen::VectorXd b = Eigen::VectorXd(n_eq);
  Eigen::VectorXd l = Eigen::VectorXd(n_in);

  try
  {
    auto C = -J * dt;
    proxsuite::proxqp::sparse::QP<T, int> qp_(dim, n_eq, n_in);
    qp_.settings.eps_abs = 1e-9;
    qp_.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp_.settings.verbose = false;

    for (int i = 0; i < iters; i++)
    {
      std::cout << "iteration: " << i << "\n";
      // This will be handeled by qp_solver.setQPParams in in the future when linearizaiton is implemented
      auto g = mass_matrix * (dt * gravity - vk);
      auto u = J * qk;  // upper bound

      // Update A and b if necessary

      // l = C.spar * vk;

      qp_solver.setQPParams(mass_matrix, g, A, b, C, l, u, vk);

      std::pair<Eigen::VectorXd, Eigen::VectorXd> result = qp_solver.runQP(qp_);

      // auto vk_1 = result.first;
      // dual = result.second;

      auto vk_1 = qp_.results.x;

      qk = qk + vk_1 * dt;
      vk << vk_1;
    }
  }

  catch (...)
  {
    std::cout << "Error: " << std::endl;
  }

  std::cout << "primal = " << vk << std::endl;
  std::cout << "dual = " << dual.transpose() << std::endl;

  std::cout << "Final State vk after " << iters << " iterations: \n" << vk << "\n";
  std::cout << "Final State qk after " << iters << " iterations: \n" << qk << "\n";

  return 0;
}