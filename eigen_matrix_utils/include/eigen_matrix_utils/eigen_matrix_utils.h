#ifndef eigen_matrix_utils_201806130948
#define eigen_matrix_utils_201806130948

#include <ros/ros.h>
#include <XmlRpc.h>
#include <boost/array.hpp>
#include <bitset>
#include <eigen3/Eigen/Core>
#include <rosparam_utilities/rosparam_utilities.h>

namespace eigen_utils
{

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int> = 0> bool resize(Eigen::MatrixBase<Derived> const & m, int rows, int cols)
{
  Eigen::MatrixBase<Derived>& mat = const_cast< Eigen::MatrixBase<Derived>& >(m);
  if((Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic) 
  && (Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic))
  {
    std::cout << "dynamic rows and cols at runtime :" << rows <<"," <<"cols" << std::endl;
    mat.derived().resize(rows,cols);
  }
  else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic) 
  {
    std::cout << "dynamc rows at runtime:" << rows<< std::endl;
    mat.derived().resize(rows,Eigen::NoChange);
  }
  else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic) 
  {
    std::cout << "dynamc cols at runtime: " << cols<< std::endl;
    mat.derived().resize(Eigen::NoChange, cols);
  }
  return true;
}


/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic) 
                        && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic) 
                        , int> = 0> bool resize(Eigen::MatrixBase<Derived> const & m, int rows, int cols)
{
  return true;
}


template <typename Derived>
void standard_deviation(const Eigen::MatrixBase<Derived>& vet)
{
  static int  nrows  = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  static Eigen::MatrixBase<Derived> mean_vet;
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
  static int  nrows  = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  static Eigen::MatrixBase<Derived> mean_vet1;
  static Eigen::MatrixBase<Derived> mean_vet2;
  static Eigen::MatrixBase<Derived> vet1_no_mean;
  static Eigen::MatrixBase<Derived> vet2_no_mean;
  if( nrows == Eigen::Dynamic)
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


inline bool getParam( const ros::NodeHandle& nh,  const std::string& key, Eigen::MatrixXd& matrix)
{
  std::vector<std::vector<double>> mtx;
  if (!rosparam_utilities::getParamMatrix<double>(nh, key, mtx))
    return false;

  int nrows, ncols;
  nrows = mtx.size();
  if (nrows>0)
    ncols = mtx.at(0).size();
  else
    ncols = 0;

  matrix.resize(nrows, ncols);
  for (int iR = 0;iR<nrows;iR++)
    for (int iC = 0;iC<ncols;iC++)
      matrix(iR, iC) = mtx.at(iR).at(iC);

  return true;
}

inline bool getParam( const ros::NodeHandle& nh,  const std::string& key, Eigen::VectorXd& vector)
{
  std::vector<double> vtc;
  if (!nh.getParam(key,vtc))
    return false;

  int nrows;
  nrows = vtc.size();

  vector.resize(nrows);
  for (int iR = 0;iR<nrows;iR++)
    vector(iR) = vtc.at(iR);

  return true;
}


};

#endif
