// Command to generate the executable:
// g++ -O3 -march=native -DNDEBUG -std=gnu++17 brick_sqp.cppa -o brick_sqp $(pkg-config --cflags proxsuite)
//

// Author: Vineet Tambe

#include <iostream>
#include <proxsuite/proxqp/sparse/sparse.hpp>  // get the sparse API of ProxQP
#include <Eigen/Core>
#include <proxsuite/helpers/optional.hpp>  // for c++14
#include <proxsuite/proxqp/sparse/sparse.hpp>

using namespace proxsuite;
using namespace proxsuite::proxqp;
using proxsuite::nullopt;  // c++17 simply use std::nullopt

using T = double;

int main()
{
  isize dim = 2, n_eq = 0, n_in = 1;
  T p = 0.15;             // level of sparsity
  T conditioning = 10.0;  // conditioning level for H

  // ------------------ Initial conditions ------------------
  int mass = 1.0;
  double dt = 0.01;
  double Time = 5.0;
  int iters = (int)Time / dt;

  Eigen::MatrixXd vk = Eigen::MatrixXd(dim, 1);
  vk << 1.0, 4.0;  // set initial conditions

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

  // -------------------
  // QP params
  // cost H
  Eigen::MatrixXd H = mass_matrix;
  Eigen::SparseMatrix<double> H_spa(n_in, dim);

  Eigen::MatrixXd g = Eigen::VectorXd(dim, 1);
  // inequality constraints C
  Eigen::MatrixXd C = Eigen::MatrixXd(n_in, dim);
  Eigen::SparseMatrix<double> C_spa(n_in, dim);
  Eigen::VectorXd l = Eigen::VectorXd(n_in);
  Eigen::VectorXd u = Eigen::VectorXd(n_in);

  H_spa = H.sparseView();
  C = -J * dt;
  C_spa = C.sparseView();
  //   l << -Eigen::Infinity;  // lower bound

  // Setup a new qp to solve the above problem
  proxsuite::proxqp::sparse::QP<T, int> qp(dim, n_eq, n_in);

  qp.settings.eps_abs = eps_abs;
  //   qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
  qp.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  qp.settings.verbose = false;

  for (int i = 0; i < iters; i++)
  {
    std::cout << "iteration: " << i << "\n";
    // simulate forward step -- this has been reduced to only update the variables that change
    g = mass_matrix * (dt * gravity - vk);
    u = J * qk;  // upper bound
    l = C_spa * vk;

    std::cout << "g = " << g << std::endl;
    std::cout << "u = " << u << std::endl;
    std::cout << "l = " << l << std::endl;

    qp.init(H_spa, g, nullopt, nullopt, C_spa, l, u);
    qp.solve();

    auto vk_1 = qp.results.x;

    qk = qk + vk_1 * dt;
    vk = vk_1;
  }

  std::cout << "Final State vk after " << iters << " iterations: \n" << vk << "\n";
  std::cout << "Final State qk after " << iters << " iterations: \n" << qk << "\n";
}
