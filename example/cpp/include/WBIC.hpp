#include <Eigen/Dense>
#include <iostream>
#include "OsqpEigen/OsqpEigen.h"

class WBIC  
{
public:
    bool WBIC_Init(unsigned int& nt, unsigned int& dof, Eigen::MatrixXd& q1,Eigen::MatrixXd& q2, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>&alljacobian,
                                Eigen::MatrixXd& a, Eigen::VectorXd& b, Eigen::VectorXd& g,double& Fc, Eigen::VectorXd Q,Eigen::VectorXd Q_dot
                                ,OsqpEigen::Solver& solver);
    Eigen::VectorXd WBIC_solve_problem(unsigned int& dof, Eigen::VectorXd& Fr, Eigen::MatrixXd& P,Eigen::MatrixXd& D,OsqpEigen::Solver& solver);
    void WBIC_setWeightMatrices(Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi);
    void WBIC_setModel(Eigen::MatrixXd& A,Eigen::VectorXd& b, Eigen::VectorXd& g);
    void WBIC_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian);
    void WBIC_setJpre(unsigned int& nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian);
    void WBIC_setJointStates(Eigen::VectorXd& q,Eigen::VectorXd& q_dot);
    void WBIC_setFrictinConstant(double& Fc);
    Eigen::MatrixXd WBIC_pseudoInverse(Eigen::MatrixXd& A);
    Eigen::MatrixXd WBIC_DCpseudoInverse(Eigen::MatrixXd& J,Eigen::MatrixXd& A);
    void WBIC_getJointCommands(unsigned int& nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian,std::vector<Eigen::MatrixXd>& allJpre,
                                std::vector<Eigen::VectorXd>& alldel_x,
                                std::vector<Eigen::VectorXd>& alldeisred_x_dot,
                                std::vector<Eigen::VectorXd>& alldeisred_x_2dot,
                                Eigen::MatrixXd& a,Eigen::VectorXd Q,Eigen::VectorXd Q_dot);
    void WBIC_setCartesianCommands(unsigned int& nt, std::vector<Eigen::VectorXd>& alldesired_x,std::vector<Eigen::VectorXd>& alldesired_x_dot,
                                std::vector<Eigen::VectorXd>& alldesired_x_2dot,std::vector<Eigen::VectorXd>& allx,
                                std::vector<Eigen::VectorXd>& allx_dot,
                                std::vector<Eigen::MatrixXd>& Kp,
                                std::vector<Eigen::MatrixXd>& Kd);
    void WBIC_setContactForce(Eigen::VectorXd& Fr);
    void WBIC_castWBIC2qpHessian(unsigned int& dof,Eigen::MatrixXd& q1,Eigen::MatrixXd& q2,Eigen::VectorXd& Fr);
    void WBIC_castWBIC2qpGradient(unsigned int& dof, Eigen::VectorXd& Fr);
    void WBIC_castWBIC2qpConstraintMatrix(unsigned int& dof, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>& alljacobian, 
                                Eigen::MatrixXd& a, double& Fc);
    void WBIC_castWBIC2qpConstraintVector(unsigned int dof, Eigen::VectorXd& Fr, Eigen::VectorXd& desired_q_2dot,Eigen::VectorXd& b
                                        ,Eigen::VectorXd& g);
    void WBIC_setTaskGains(std::vector<Eigen::MatrixXd>& kp, std::vector<Eigen::MatrixXd>& kd);
    // bool WBIC_updateAllConstraint(std::vector<Eigen::MatrixXd>&allProjections,std::vector<Eigen::MatrixXd>&update_jacobian,std::vector<Eigen::VectorXd>& update_x_dot_d,Eigen::VectorXd& update_q,Eigen::MatrixXd& update_Qr,Eigen::MatrixXd& update_Qi);
   


private:
    
    unsigned int Nt = 0;
    unsigned int Dof = 0; //floating base joint + actual joint
    double fc = 0; //static friction constant
    
    
    Eigen::VectorXd QPSolution;
    Eigen::VectorXd ctr;
    Eigen::VectorXd Ts;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lbq_dot;
    Eigen::VectorXd ubq_dot;
    Eigen::VectorXd current_q;
    Eigen::VectorXd q_lower_limit;
    Eigen::VectorXd q_upper_limit;
    

    //hessian weights
    Eigen::MatrixXd Q1;
    Eigen::MatrixXd Q2;
    //task jacobians 
    std::vector<Eigen::MatrixXd> jacobians;
    std::vector<Eigen::MatrixXd> Jpres;
    //task gains
    std::vector<Eigen::MatrixXd> Kp;
    std::vector<Eigen::MatrixXd> Kd;
    //deisred poses 
    std::vector<Eigen::VectorXd> desired_x;
    std::vector<Eigen::VectorXd> del_x;
    std::vector<Eigen::VectorXd> desired_x_dot;
    std::vector<Eigen::VectorXd> desired_x_2dot;
    //deisred states
    Eigen::VectorXd del_q;
    Eigen::VectorXd desired_q;
    Eigen::VectorXd desired_q_dot;
    Eigen::VectorXd desired_q_2dot;
    //desired contact force
    Eigen::VectorXd fr;
    //current states
    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;
    std::vector<Eigen::VectorXd> x;
    std::vector<Eigen::VectorXd> x_dot;
    //floatng base model
    Eigen::MatrixXd A;
    Eigen::VectorXd B; //coriollis * q double dot
    Eigen::MatrixXd S;
    Eigen::VectorXd G;
    //Optimal solutions
    Eigen::VectorXd q2dot;
    Eigen::VectorXd desired_fr;
    //Solutions accounting model
    Eigen::VectorXd tau;
    Eigen::VectorXd motor_cmd;
    
};
