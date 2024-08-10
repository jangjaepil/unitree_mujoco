#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <iostream>

class MPC  
{
public:
    double getErrorNorm(const Eigen::VectorXd& x, const Eigen::VectorXd& xRef);
    void updateConstraintVectors(const Eigen::VectorXd& x0,
                                            Eigen::VectorXd& lowerBound,
                                            Eigen::VectorXd& upperBound);
    void castMPCToQPConstraintVectors(const Eigen::VectorXd& xMax,
                                  const Eigen::VectorXd& xMin,
                                  const Eigen::VectorXd& uMax,
                                  const Eigen::VectorXd& uMin,
                                  const Eigen::VectorXd& x0,
                                  int mpcWindow,
                                  int rSize,
                                  Eigen::VectorXd& lowerBound,
                                  Eigen::VectorXd& upperBound);
    void castMPCToQPConstraintMatrix(const Eigen::MatrixXd& dynamicMatrix,
                                 const Eigen::MatrixXd& controlMatrix,
                                 int mpcWindow,
                                 int rSize,
                                 double& Fc,
                                 Eigen::SparseMatrix<double>& constraintMatrix);

    void castMPCToQPGradient(const Eigen::MatrixXd& Qn,const Eigen::MatrixXd& Q,
                         const Eigen::VectorXd& xRef,
                         int mpcWindow,
                         int rSize,
                         Eigen::VectorXd& gradient);

    void castMPCToQPHessian(const Eigen::MatrixXd& Qn, const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R,
                        int mpcWindow,
                        int rSize,
                        Eigen::SparseMatrix<double>& hessianMatrix);
    void setWeightMatrices(Eigen::MatrixXd& Qn, Eigen::MatrixXd& Q, Eigen::MatrixXd& R,int rSize);
    void setInequalityConstraints(Eigen::VectorXd& xMax,
                              Eigen::VectorXd& xMin,
                              Eigen::VectorXd& uMax,
                              Eigen::VectorXd& uMin,
                              double m,
                              int rSize);
    void setDynamicsMatrices(double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, std::vector<Eigen::VectorXd>& r,double m,int rSize);
    bool init(Eigen::VectorXd& x_init,Eigen::VectorXd& x_ref,double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, std::vector<Eigen::VectorXd>& r,double m,double fc, OsqpEigen::Solver& solver);
    bool solveProblem(OsqpEigen::Solver& solver);
    bool updateProblem(double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, Eigen::VectorXd x0, 
                        std::vector<Eigen::VectorXd>& r, double m,double fc,OsqpEigen::Solver& solver);

    Eigen::VectorXd getStates();
    Eigen::VectorXd getCtr();
    void setRef(Eigen::VectorXd dx);

private:
    int mpcWindow = 20;
    int rSize = 0;
    Eigen::VectorXd QPSolution;
    Eigen::VectorXd ctr;
    

    // allocate the dynamics matrices
    
    Eigen::MatrixXd a;
    Eigen::MatrixXd b;

    // allocate the constraints vector
    Eigen::VectorXd xMax;
    Eigen::VectorXd xMin;
    Eigen::VectorXd uMax;
    Eigen::VectorXd uMin;

    // allocate the weight matrices
    Eigen::MatrixXd Qn;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    // allocate the initial and the reference state space
    Eigen::VectorXd x0;
    Eigen::VectorXd xRef;

    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

};
