#ifndef eigen_matrix_utils_201806130948
#define eigen_matrix_utils_201806130948

#include <sstream>
#include <exception>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/node_handle.h>
#include <rosparam_utilities/rosparam_utilities.h>

namespace eigen_utils
{

template <typename Derived>
double standard_deviation(const Eigen::MatrixBase<Derived>& vet)
{
  static int  nrows  = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  static Derived mean_vet;
  if( nrows == Eigen::Dynamic)
  {
    nrows = vet.rows();
    mean_vet.resize(vet.rows());
  }
  mean_vet.setConstant(vet.mean());
  return std::sqrt((vet-mean_vet).dot(vet-mean_vet))/nrows;
}


template <typename Derived>
double correlation(const Eigen::MatrixBase<Derived>& vet1, const Eigen::MatrixBase<Derived>& vet2)
{
  static int nrows = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  static Eigen::VectorXd mean_vet1(nrows);
  static Eigen::VectorXd mean_vet2(nrows);
  static Eigen::VectorXd vet1_no_mean(nrows);
  static Eigen::VectorXd vet2_no_mean(nrows);

  if(nrows == Eigen::Dynamic)
  {
    nrows = vet1.rows();
    mean_vet1.resize(nrows);
    mean_vet2.resize(nrows);
    vet1_no_mean.resize(nrows);
    vet1_no_mean.resize(nrows);
  }
  
  mean_vet1.setConstant(vet1.mean());
  mean_vet2.setConstant(vet2.mean());
  vet1_no_mean = vet1 - mean_vet1;
  vet2_no_mean = vet2 - mean_vet2;
  
  return ( vet1_no_mean.dot(vet2_no_mean) ) / std::sqrt( ( vet1_no_mean.dot(vet1_no_mean) ) * ( vet2_no_mean.dot(vet2_no_mean) ) );
  
}


inline std::string to_string(const double& m, bool transpose = true)
{
    std::ostringstream out;
    out.precision(4);
    out << std::fixed << m;
    return out.str();
}

template<typename Derived>
inline std::string to_string(const Eigen::MatrixBase<Derived>& m, bool transpose = true)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  if(transpose)
    ss << m.transpose().format(CleanFmt);
  else
    ss << m.format(CleanFmt);
  return ss.str();
}

inline unsigned int factorial(unsigned int n)
{
    return (n == 0) ? 1 : n * factorial(n - 1);
}



};

#endif
