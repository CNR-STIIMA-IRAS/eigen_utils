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

double standard_deviation(const Eigen::Ref<Eigen::VectorXd>& vet);
double correlation(const Eigen::Ref<Eigen::VectorXd>& vet1, const Eigen::Ref<Eigen::VectorXd>& vet2);

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

inline bool getParam( const ros::NodeHandle& nh,  const std::string& key, std::vector<Eigen::VectorXd>& vector )
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

  vector.clear();
  for (int iC=0; iC<ncols; iC++)
  {
    Eigen::VectorXd col(nrows);
    for (int iR=0; iR<nrows; iR++)
      col(iR) = mtx.at(iR).at(iC);

    vector.push_back(col);
  }

  return true;
}

inline bool setParam( const ros::NodeHandle& nh,  const std::string& key, const std::vector<Eigen::VectorXd>& vector )
{
  XmlRpc::XmlRpcValue data;
  int iCol = 0;
  for ( auto itCol=vector.begin(); itCol!=vector.end(); itCol++ )
  {
    int iEl = 0;
    XmlRpc::XmlRpcValue col;
    for (auto iRow=0; iRow<(*itCol).size(); iRow++)
      col[iEl++] = (*itCol)[iRow];
     
    data[iCol++] = (XmlRpc::XmlRpcValue)col;
  }

  nh.setParam( key, data );
  return true;
}


};

#endif
