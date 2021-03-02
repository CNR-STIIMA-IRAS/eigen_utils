#pragma once //workaround qtcreator clang-tidy

#ifndef EIGEN_STATE_SPACE_SYSTEMS__OPERATIONS_IMPL_H
#define EIGEN_STATE_SPACE_SYSTEMS__OPERATIONS_IMPL_H

#include <random>
#include<eigen_matrix_utils/overloads.h>

namespace eigen_utils
{



/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int> >
inline bool resize(Eigen::MatrixBase<Derived> const & m, int rows, int cols)
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
  return (mat.derived().rows() == rows) && (mat.derived().cols() == cols);
}


/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic) 
                        && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic) 
                        , int> >
inline bool resize(Eigen::MatrixBase<Derived> const & /*m*/, int rows, int cols)
{
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
}

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
inline bool resize(const double& /*m*/, int rows, int cols)
{
  return rows==1 && cols ==1;
}


/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
        std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        ||(Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int> >
inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  if((m.rows()!= rows)||(m.cols()!= cols))
  {
    error += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
    error +=    id + " dimension mismatch."
        " Object size: " + std::to_string(m.rows()) + "x" + std::to_string(m.cols()) + ","
        " Expected size: " + std::to_string(rows) + "x" + std::to_string(cols) + "";
    return false;
  }
  return true;
}


/**
 * CHECK - SKIP, SINCE THE DIMENSION IS CHECKED AT COMPILE TIME
 */
template<typename Derived,
        std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic) 
                        && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic) 
                        , int> >
inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
}

inline bool checkInputDim(const std::string& id, const double& m, int rows, int cols, std::string& error)
{
  return rows == 1 && cols == 1;
}

template<typename Derived>
inline void checkInputDimAndThrowEx(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols)
{
  std::string error = std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
  if(!eigen_utils::checkInputDim(id,m,rows,cols,error))
  {
    throw std::runtime_error(error.c_str());
  }
}

inline void checkInputDimAndThrowEx(const std::string& id, const double& m, int rows, int cols)
{
  if(rows !=1 && cols !=1)
  {
    throw std::runtime_error((id + ": expected a 1x1 (double) while a matrix has been assigned").c_str());
  }
}

// double, double
inline bool copy(double& lhs, const double& rhs)
{
  lhs = rhs;
  return true;
}


// double, double
inline bool copy(double& lhs, const std::vector<double>& rhs)
{
  if(rhs.size()!=1)
    return false;
  lhs = rhs.front();
  return true;
}

// double, double
inline bool copy(std::vector<double>& lhs, const double& rhs)
{
  if(lhs.size()!=1)
    return false;
  lhs.front() = rhs;
  return true;
}

// double, matrix
template<typename Derived> 
inline bool copy(double& lhs, const Eigen::MatrixBase<Derived>& rhs)
{
  if(rhs.rows()!=1 || rhs.cols()!=1)
  {
    return false;
  }
  lhs = rhs(0,0);
  return true;
}

// matrix, matrix
template<typename Derived, typename OtherDerived>
inline bool copy(Eigen::MatrixBase<Derived> & lhs, 
          const Eigen::MatrixBase<OtherDerived>& rhs)
{
  if(!eigen_utils::resize(lhs, rhs.rows(),  rhs.cols()))
    return false;
  lhs = rhs;
  return true;
}

// matrix, double
template<typename Derived> 
inline bool copy(Eigen::MatrixBase<Derived>& lhs, const double& rhs)
{
  lhs.setConstant(rhs);
  return true;
}

// matrix, double
template<typename Derived>
inline bool copy(Eigen::MatrixBase<Derived>& lhs, const std::vector<double>& rhs)
{
  if(!eigen_utils::resize(lhs, int(rhs.size()),  1))
    return false;
  for(int i=0;i<lhs.rows();i++)
    lhs(i) = rhs.at(size_t(i));
  return true;
}

// matrix, double
template<typename Derived>
inline bool copy(std::vector<double>& lhs, const Eigen::MatrixBase<Derived>& rhs)
{
  if(int(lhs.size())!=rhs.rows())
  {
    lhs.resize(rhs.rows());
  }
  for(int i=0;i<rhs.rows();i++)
    lhs.at(size_t(i)) = rhs(i);
  return true;
}



// A least-squares solution of m*x = rhs
//     x   = S x 1
//     rhs = (OxS) x 1
//     m   = (OxS) x S
template<typename Derived, typename InputDerived, typename OutputDerived>
inline bool svd( const Eigen::MatrixBase<Derived>& m, // OW x S
          const Eigen::MatrixBase<InputDerived>& rhs,
          Eigen::MatrixBase<OutputDerived>& x)
{
  Eigen::JacobiSVD< Eigen::MatrixXd > svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if(svd.rank()<m.cols())
  {
    return false;
  }
  x = svd.solve(rhs); // state at the begin of initialization interval
  return true;
}

inline bool svd(const double& m, 
                const double& rhs,
                double&       x)
{
  x = rhs; // state at the begin of initialization interval
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


inline void setRandom(double& m)
{
 double lower_bound = 0;
 double upper_bound = 10000;
 std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
 std::default_random_engine re;
 m = unif(re);
}

template<typename Derived>
inline void setRandom(Eigen::MatrixBase<Derived>& m)
{
  m.setRandom();
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
inline const typename Derived::Scalar* data(const Eigen::MatrixBase<Derived>& m)
{
  return m.derived().data();
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

inline double normalized(const double& m)
{
  return 1.0;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
  normalized(const Eigen::MatrixBase<Derived>& m)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> ret;
  ret = m.normalized();
  return ret;
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


inline int rank(const double& m)
{
  return 1;
}

template<typename Derived>
inline int rank(const Eigen::MatrixBase<Derived>& m)
{
  Eigen::FullPivLU<Derived> lu_decomp(m);
  return lu_decomp.rank();
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
//================================================================
//
//================================================================
inline bool copy_to_block(double& lhs, const double& rhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  lhs = rhs;
  bool ret = startRow==0 && startCol==0 && blockRows==1 && blockCols==1;
  if(!ret)
  {
    std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments " 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  }
  return ret;
}

template<typename LHS, typename RHS>
inline bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows == rhs.rows() && blockCols == rhs.cols() )
  && (startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() ))
  {
    lhs.block(startRow, startCol, blockRows, blockCols) = rhs;
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_to_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if(startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() )
  {
    lhs.block(startRow, startCol, blockRows, blockCols).setConstant(rhs);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_to_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows==1 && blockCols==1)
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols()))
  {
    lhs = rhs(startRow,startCol);
    return true;
  }
  return false;
}

inline bool copy_to_block(double& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename LHS, typename RHS>
inline bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename Derived>
inline bool copy_to_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename Derived>
inline bool copy_to_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}


//================================================================
//
//================================================================
inline bool copy_from_block(double& lhs, const double& rhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  lhs = rhs;
  bool ret = startRow==0 && startCol==0 && blockRows==1 && blockCols==1;
  if(!ret)
  {
    std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  }
  return ret;
}

template<typename LHS, typename RHS>
inline bool copy_from_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows == lhs.rows() && blockCols == lhs.cols() )
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols() ))
  {
    lhs = rhs.block(startRow, startCol, blockRows, blockCols);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_from_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if(startRow==0 && startCol==0 && blockRows==1 && blockCols==1)
  {
    lhs.setConstant(rhs);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_from_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows==1 && blockCols==1)
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols()))
  {
    lhs = rhs(startRow,startCol);
    return true;
  }
  return false;
}





inline bool solve(double& ret, const double& A, const double& b )
{
  ret = b / A;
  return true;
}

template<typename Derived>
inline bool solve(Eigen::MatrixBase<Derived>& x, const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  Eigen::VectorXd _x;
  if((A.rows()==A.cols()) && (eigen_utils::rank(A) == A.rows()))
  {
    _x = A.fullPivLu().solve(b);
  }
  else if(eigen_utils::rank(A) == std::min(A.rows(),A.cols()))
  {
    _x = A.colPivHouseholderQr().solve(b);
  }
  else
  {
    _x = A.jacobiSvd().solve(b);
  }

  return (eigen_utils::copy_to_block(x, _x, 0, 0, _x.rows(),1) );
}


template<typename Derived>
inline bool solve(Eigen::MatrixBase<Derived>& x, const Eigen::MatrixXd& A, const double& b)
{
  Eigen::VectorXd _b(1); _b << b;
  Eigen::VectorXd _x;
  if((A.rows()==A.cols()) && (eigen_utils::rank(A) == A.rows()))
  {
    _x = A.fullPivLu().solve(_b);
  }
  else if(eigen_utils::rank(A) == std::min(A.rows(),A.cols()))
  {
    _x = A.colPivHouseholderQr().solve(_b);
  }
  else
  {
    _x = A.jacobiSvd().solve(_b);
  }

  return (eigen_utils::copy_to_block(x, _x, 0, 0, _x.rows(),1) );
}


inline void setConstant(double& m, const double& v)
{
  m = v;
}

template<typename Derived>
inline void setConstant(Eigen::MatrixBase<Derived>& m, const double& v)
{
  m.setConstant(v);
}

inline void setDiagonal(double& m, const double& v)
{
  m = v;
}

template<typename Derived>
inline void setDiagonal(Eigen::MatrixBase<Derived>& m, const double& v)
{
  m.diagonal().setConstant(v);
}


inline void saturate(double& v, const double& min, const double& max)
{
  v = std::max(std::min(v, max), min);
}

template<typename Derived>
inline void saturate(Eigen::MatrixBase<Derived>& m, const double& min, const double& max)
{
  for(int i=0;i<m.rows();i++)
  {
    for(int j=0;j<m.rows();j++)
    {
        m(i,j) = std::max(std::min(m(i,j), max), min);
    }
  }
}

template<typename Derived>
inline void saturate(Eigen::MatrixBase<Derived>& m, 
                     const Eigen::MatrixBase<Derived>& min,
                     const Eigen::MatrixBase<Derived>& max)
{
  assert(m.rows() == min.rows());
  assert(m.rows() == max.rows());
  assert(m.cols() == min.cols());
  assert(m.cols() == max.cols());

  for(int i=0;i<m.rows();i++)
  {
    for(int j=0;j<m.cols();j++)
    {
        m(i,j) = std::max(std::min(m(i,j), max(i,j)), min(i,j));
    }
  }
}

inline double dot(const double& m1, const double& m2)
{
  return m1 * m2;
}

template<typename Derived, typename OtherDerived>
inline double dot(const Eigen::MatrixBase<Derived>& m1, const Eigen::MatrixBase<OtherDerived>& m2)
{
  return m1.dot(m2);
}



//============
template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon)
{
	using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
																			 MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
	Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
	Eigen::Index rank = svd.rank();
	Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
								0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
		tmp = svd.matrixU().leftCols(rank).adjoint();
	tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
	return svd.matrixV().leftCols(rank) * tmp;
}

inline double div(const double& a, const double b)
{
  return a/b;
}

template<typename AType, typename BType> //  C = B^-1 x A  (bc x ac ) = (bc x br)  x (ar x ac )
using DivType = Eigen::Matrix<typename AType::Scalar, BType::ColsAtCompileTime, AType::ColsAtCompileTime>;

template<typename AType, typename BType>
inline DivType<AType, BType> div(const AType& a, const BType& b)
{
  DivType<AType, BType> ret;
  PseudoInverseType<BType> binv = pseudoInverse(b);
  ret = binv * a;
  return ret;
}
  
}  // namesapce eigen_utils

#endif