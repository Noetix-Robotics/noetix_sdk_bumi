#pragma once

#include <Eigen/Dense>

using tensor_element_t = float;
using scalar_t = double;
using Duration = std::chrono::duration<double>;
using Vecjoint = Eigen::VectorXd;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

using Clock = std::chrono::high_resolution_clock;

template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;
