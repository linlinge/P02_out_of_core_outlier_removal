#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>
#include <algorithm>
#include <omp.h>
using namespace std;
Eigen::MatrixXd get_curvature(Eigen::MatrixXd& pts, Eigen::MatrixXd& neighbour_idx)
{
  Eigen::MatrixXd curvature(pts.rows(),1);
  Eigen::MatrixXd pmean(1,3);
  int K=neighbour_idx.cols();
 
  #pragma omp parallel for
  for(int i=0;i<neighbour_idx.rows();i++){
    // row
    Eigen::MatrixXd dat(neighbour_idx.cols(),3);
    for(int j=0;j<neighbour_idx.cols();j++){
      dat(j,0)=pts(neighbour_idx(i,j),0);
      dat(j,1)=pts(neighbour_idx(i,j),1);
      dat(j,2)=pts(neighbour_idx(i,j),2);
      pmean=pmean+dat.row(j);
    }
    pmean=pmean/neighbour_idx.cols();

    for(int j=0;j<dat.rows();j++)
      dat.row(j)=dat.row(j)-pmean;    
    Eigen::MatrixXd C=dat.transpose()*dat/K;
    
    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(C);
    Eigen::VectorXcd tt=eigen_solver.eigenvalues();
    vector<double> lambda;
    lambda.resize(3);
    lambda[0]=tt(0).real();
    lambda[1]=tt(1).real();
    lambda[2]=tt(2).real();
    sort(lambda.begin(),lambda.end());
    curvature(i,0)=lambda[0]/(lambda[0]+lambda[1]+lambda[2]);    
  }
  return curvature;
}

void print()
{
  cout<<"wokaka"<<endl;
}

PYBIND11_MODULE(solve_curvature, m)
{
    // optional module docstring
    m.doc() = "pybind11 example plugin";
    m.def("get_curvature", &get_curvature, "get_curvature");
    m.def("print",&print,"print");
}