#include "WBIC.hpp"

void WBIC::WBIC_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian)
{
    this -> jacobians = alljacobian;
}

void WBIC::WBIC_setModel(Eigen::MatrixXd& a,Eigen::MatrixXd& b, Eigen::VectorXd g)
{
    this -> A = a;
    this -> B = b;
    this -> G = g;

}

void WBIC::WBIC_setJointStates(Eigen::VectorXd& Q,Eigen::VectorXd& Q_dot)
{
    this-> q = Q;
    this-> q_dot = Q_dot;
}
void WBIC::WBIC_setContactForce(Eigen::VectorXd& Fr)
{
    this -> fr = Fr;
}

void WBIC::WBIC_setWeightMatrices(Eigen::MatrixXd& q1, Eigen::MatrixXd& q2)
{
    this -> Q1 = q1;
    this -> Q2 = q2;
}

void WBIC::WBIC_setTaskGains(std::vector<Eigen::MatrixXd>& kp, std::vector<Eigen::MatrixXd>& kd)
{
    this -> Kp = kp;
    this -> Kd = kd;

}

Eigen::MatrixXd WBIC::WBIC_pseudoInverse(Eigen::MatrixXd& m)
{   
     Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Get the singular values, U and V matrices
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd singularValues = svd.singularValues();

    // Form the diagonal matrix Sigma^+
    Eigen::MatrixXd SigmaPlus = Eigen::MatrixXd::Zero(m.cols(), m.rows());
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > 1e-10) { // Consider singular values greater than a small epsilon
            SigmaPlus(i, i) = 1.0 / singularValues(i);
        }
    }

    // Compute the pseudo-inverse: A^+ = V * Sigma^+ * U^T
    Eigen::MatrixXd A_pseudoInverse = V * SigmaPlus * U.transpose();

    return A_pseudoInverse;
}

Eigen::MatrixXd WBIC::WBIC_DCpseudoInverse(Eigen::MatrixXd& J,Eigen::MatrixXd& A)
{
    Eigen::MatrixXd temp;
    Eigen::MatrixXd J_dcinv;
    temp = J*A.inverse()*J.transpose();
    J_dcinv = A.inverse()*J.transpose()*temp.inverse();

    return J_dcinv;
}

void WBIC::WBIC_setJpre(unsigned int& Nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian)
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Dof,Dof);
    std::vector<Eigen::MatrixXd> allJpre;
    Eigen::MatrixXd Jpre;
    Eigen::MatrixXd N0;
    Eigen::MatrixXd Ni;

    allJpre.clear();
    for(int i = 0;i<Nt;i++)
    {
        
        if(i==0)
        {   
            N0 = (I - WBIC_pseudoInverse(alljacobian[0])*alljacobian[0]);        
            Jpre = alljacobian[1]*N0;
            allJpre.push_back(Jpre);
            Ni = N0;
        }
        else
        {  
            Ni = Ni*(I - WBIC_pseudoInverse(Jpre)*Jpre);
            Jpre = alljacobian[i+1]*Ni;
            allJpre.push_back(Jpre);
        }
    }

    this -> Jpres = allJpre;
}

void WBIC::WBIC_getJointCommands(unsigned int& Nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian,std::vector<Eigen::MatrixXd>& allJpre,
                                std::vector<Eigen::VectorXd>& alldel_x,
                                std::vector<Eigen::VectorXd>& alldeisred_x_dot,
                                std::vector<Eigen::VectorXd>& alldeisred_x_2dot,
                                Eigen::MatrixXd& a,Eigen::VectorXd Q,Eigen::VectorXd Q_dot)
{
    
    Eigen::VectorXd Delq = Eigen::VectorXd::Zero(Dof);
    Eigen::VectorXd Desired_q_dot = Eigen::VectorXd::Zero(Dof);
    Eigen::VectorXd Desired_q_2dot = Eigen::VectorXd::Zero(Dof);

    Desired_q_2dot = WBIC_DCpseudoInverse(alljacobian[0],a)*(-alljacobian[0]*Q_dot);
    
    for(int i = 0; i<Nt ; i++)
    {
        Delq = Delq + allJpre[i]*(alldel_x[i] - alljacobian[i+1]*Delq);
        Desired_q_dot = Desired_q_dot + allJpre[i]*(alldeisred_x_dot[i] - alljacobian[i+1]*Desired_q_dot);
        Desired_q_2dot = Desired_q_2dot + WBIC_DCpseudoInverse(allJpre[i],a)*(alldeisred_x_2dot[i] - alljacobian[i+1]*Desired_q_2dot); 
        
    }

    this->desired_q = Q + Delq;
    this->desired_q_dot = Desired_q_dot;
    this->desired_q_2dot = Desired_q_2dot;
}

void WBIC::WBIC_setCartesianCommands(std::vector<Eigen::VectorXd>& alldesired_x,std::vector<Eigen::VectorXd>& alldesired_x_dot,
                                std::vector<Eigen::VectorXd>& alldeisred_x_2dot,std::vector<Eigen::VectorXd>& allx,
                                std::vector<Eigen::VectorXd>& allx_dot,
                                std::vector<Eigen::MatrixXd>& Kp,
                                std::vector<Eigen::MatrixXd>& Kd)
{

    this-> desired_x = alldesired_x;
    this-> desired_x_dot = alldesired_x_dot;
    
    this-> del_x.clear();
    this-> desired_x_2dot.clear();

    for(int i = 0;i<Nt;i++)
    {
        this-> del_x.push_back(alldesired_x[i] - allx[i]);
        this-> desired_x_2dot.push_back(alldeisred_x_2dot[i] + Kp[i]*(del_x[i]) + Kd[i]*(alldesired_x_dot[i] - allx_dot[i]));
    }    
}

void WBIC::WBIC_castWBIC2qpHessian(unsigned int& dof,Eigen::MatrixXd& q1,Eigen::MatrixXd& q2,Eigen::VectorXd& Fr)
{
    hessian.resize(q1.cols()+q2.cols()+ dof + Fr.size(),q1.cols()+q2.cols()+ dof + Fr.size());
    
    for(int i = 0;i<q1.cols()+q2.cols();i++)
    {
        if(i<q1.cols())
        {
            hessian.insert(i,i) = q1(i,i);
        }
        else
        {
            hessian.insert(i,i) = q2(i-q1.cols(),i-q1.cols());
        }
    }
}
void WBIC::WBIC_castWBIC2qpConstraintMatrix(unsigned int& dof, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>& alljacobian, 
                                Eigen::MatrixXd& a,Eigen::MatrixXd& b, float& Fc)
{
    this -> linearMatrix.resize(6 + dof + 6*Fr.size(),2*Fr.size()+ dof + 6);
    Eigen::MatrixXd LinearMatrix = Eigen::MatrixXd::Zero(6 + dof + 6*Fr.size(),2*Fr.size()+ dof + 6);
    
    Eigen::MatrixXd Sf = Eigen::MatrixXd::Zero(6,dof);
    Sf.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
    
    Eigen::MatrixXd Sf_e = Eigen::MatrixXd::Zero(dof,6);
    Sf_e.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);
    
    LinearMatrix.block(0,Fr.size(),dof,6) = Sf_e;
    LinearMatrix.block(0,Fr.size()+6,dof,dof) = Eigen::MatrixXd::Identity(dof,dof);
    LinearMatrix.block(dof,Fr.size()+6,6,dof) = Sf*a;
    LinearMatrix.block(dof,Fr.size()+6+dof,6,Fr.size()) = -Sf*alljacobian[0].transpose();
    LinearMatrix.block(dof+6,0,Fr.size(),Fr.size()) = -Eigen::MatrixXd::Identity(Fr.size(),Fr.size());
    LinearMatrix.block(dof+6,Fr.size()+6+dof,Fr.size(),Fr.size()) = Eigen::MatrixXd::Identity(Fr.size(),Fr.size());
    
    Eigen::MatrixXd FrConstrainMatrix = Eigen::MatrixXd::Zero(5,3);
    FrConstrainMatrix <<1,0,Fc,
                        0,1,Fc,
                        -1,0,Fc,
                        0,-1,Fc,
                        0,0,1;

    for(int i = 0;i<Fr.size()/3;i++)
    {
        LinearMatrix.block(dof+6+Fr.size()+i*5,Fr.size()+6+dof+i*3,5,3) = FrConstrainMatrix;
    }
    
    this -> linearMatrix = LinearMatrix.sparseView();
    
}

void WBIC::WBIC_castWBIC2qpGradient(unsigned int& dof, Eigen::VectorXd& Fr)
{
    gradient.resize(2*Fr.size() + dof + 6);
    this -> gradient = Eigen::VectorXd::Zero(2*Fr.size() + dof + 6);
}

void WBIC::WBIC_setFrictinConstant(float& Fc)
{
    this -> fc = Fc;
}
bool WBIC::WBIC_Init(unsigned int& nt, unsigned int& dof, Eigen::MatrixXd& q1,Eigen::MatrixXd& q2, std::vector<Eigen::MatrixXd>& kp,
                                std::vector<Eigen::MatrixXd>& kd, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>&alljacobian,
                                Eigen::MatrixXd& a, Eigen::MatrixXd& b, float& Fc)
{
    this-> Nt = nt;
    this-> Dof = dof;

    WBIC_setFrictinConstant(Fc);
    WBIC_setWeightMatrices(q1,q2);
    WBIC_setTaskGains(kp,kd);
    WBIC_castWBIC2qpHessian(dof,q1,q2,Fr);
    WBIC_castWBIC2qpGradient(dof,Fr);
    WBIC_castWBIC2qpConstraintMatrix(dof, Fr, alljacobian, a, b, Fc);

    solver.settings()->setWarmStart(true);

    std::cout<<" set the initial data of the QP solver"<<std::endl;
    solver.data()->setNumberOfVariables(6 + dof + 2*Fr.size());
    solver.data()->setNumberOfConstraints(6 + dof + 8*Fr.size()/3);
    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    return 0;
}

bool WBIC::WBIC_solve_problem(unsigned int& dof, Eigen::VectorXd& Fr)
{
    // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    ctr.resize(dof + Fr.size());
    ctr = QPSolution.block(6 + Fr.size(), 0, dof + Fr.size(), 1);
    
    return 0;
}
