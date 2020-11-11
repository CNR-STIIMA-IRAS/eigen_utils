#ifndef eigen_matrix_utils_201806130948
#define eigen_matrix_utils_201806130948

#include <exception>
#include <Eigen/Core>

#include <ros/ros.h>
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
    mat.derived().resize(rows,cols);
  }
  else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic) 
  {
    mat.derived().resize(rows,Eigen::NoChange);
  }
  else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic) 
  {
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
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
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

inline void setZero(double& m)
{
  m = 0;
}

template<typename Derived>
inline void setZero(Eigen::MatrixBase<Derived>& m)
{
  m.setZero();
}

inline void setIdentity(double& m)
{
  m = 1;
}

template<typename Derived>
void setIdentity(Eigen::MatrixBase<Derived>& m)
{
  m.setIdentity();
}


inline double* data(double& v)
{
  return &v;
}

template<typename Derived>
inline typename Derived::Scalar* data(Eigen::MatrixBase<Derived>& m)
{
  Eigen::Ref<Eigen::Matrix<
    typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> 
            > _m(m);
  return _m.data();
}

inline const double* data(const double& v)
{
  return &v;
}

template<typename Derived>
inline const typename Derived::Scalar* data(const Eigen::Ref<Derived const>& m)
{
  return m.data();
}

inline const double& at(const double& v, const int& i, const int& j)
{
  if((i!=0)||(j!=0))
    throw std::invalid_argument("Input is double, while the index is greater than 0");
  return v;
}

template<typename Derived>
inline const double& at(const Eigen::MatrixBase<Derived>& m, const int& i, const int& j)
{
  return m(i,j);
}

inline double& at(double& v, const int& i, const int& j)
{
  if((i!=0)||(j!=0))
    throw std::invalid_argument("Input is double, while the index is greater than 0");
  return v;
}

template<typename Derived>
inline double& at(Eigen::MatrixBase<Derived>& m, const int& i, const int& j)
{
  return m(i,j);
}


inline double norm(const double& m)
{
  return std::fabs(m);
}

template<typename Derived>
inline double norm(const Eigen::MatrixBase<Derived>& m)
{
  return m.norm();
}

inline int size(const double& m)
{
  return 1;
}

template<typename Derived>
inline int size(const Eigen::MatrixBase<Derived>& m)
{
  return m.size();
}


inline int rows(const double& m)
{
  return 1;
}

template<typename Derived>
inline int rows(const Eigen::MatrixBase<Derived>& m)
{
  return m.rows();
}

inline int cols(const double& m)
{
  return 1;
}

template<typename Derived>
inline int cols(const Eigen::MatrixBase<Derived>& m)
{
  return m.cols();
}

inline double& block(double& m, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return m;
}

inline const double& block(const double& m, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return m;
}

template<typename Derived>
inline Eigen::Block<Derived> block(Eigen::MatrixBase<Derived>& m, 
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return Eigen::Block<Derived>(m.derived(),startRow, startCol, blockRows, blockCols);
}
 
template<typename Derived>
inline const Eigen::Block<Derived> block(const Eigen::MatrixBase<Derived>& m, 
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return Eigen::Block<Derived>(m.derived(),startRow, startCol, blockRows, blockCols);
}

inline bool copy_block(double& lhs, const double& rhs, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  lhs = rhs;
  return startRow==0 && startCol==0 && blockRows==1 && blockCols==1;
}

template<typename Derived, typename OtherDerived>
inline bool copy_block(Eigen::MatrixBase<Derived>& lhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols, 
              const Eigen::MatrixBase<OtherDerived>& rhs)
{
  if((blockRows == rhs.rows() && blockCols == rhs.cols() )
  && (startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() ))
  {
    lhs.block(startRow, startCol, blockRows, blockCols) = rhs;
    return true;
  }
  return false;
}

template<typename Derived>
inline bool copy_block(Eigen::MatrixBase<Derived>& lhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols, 
              const double& rhs)
{
  if(startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() )
  {
    lhs.block(startRow, startCol, blockRows, blockCols).setConstant(rhs);
    return true;
  }
  return false;
}

template<typename Derived>
inline bool copy_block(double& lhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols, 
              const Eigen::MatrixBase<Derived>& rhs)
{
  if((blockRows==1 && blockCols==1)
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols()))
  {
    lhs = rhs(startRow,startCol);
    return true;
  }
  return false;
}


};

#endif
