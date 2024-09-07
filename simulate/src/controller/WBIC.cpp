#include "WBIC.hpp"

void WBIC::WBIC_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian)
{
    this -> jacobians = alljacobian;
}

void WBIC::WBIC_setModel(Eigen::MatrixXd& a,Eigen::VectorXd& b, Eigen::VectorXd& g)
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

    J_dcinv = A.inverse()*J.transpose()*WBIC_pseudoInverse(temp);
    Eigen::MatrixXd result = J*J_dcinv;
    //std::cout<<"result: \n"<<result<<std::endl;
    
    return J_dcinv;
}

void WBIC::WBIC_setJpre(unsigned int& nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian)
{
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Dof,Dof);
    std::vector<Eigen::MatrixXd> allJpre;
    Eigen::MatrixXd Jpre;
    Eigen::MatrixXd N0 = Eigen::MatrixXd::Identity(Dof,Dof);
    Eigen::MatrixXd Ni;

    allJpre.clear();
    //////std::cout<<"nt: "<<nt<<std::endl;
    for(int i = 0;i<nt-1;i++)
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
int size = allJpre.size();

////std::cout<<"size of jpre: "<< size<<std::endl;
// for(int i = 0;i<size;i++)
// {
//     //std::cout<<"jpre "<< i <<"\n"<<allJpre[i]<<std::endl;
// }

    this -> Jpres = allJpre;
}

void WBIC::WBIC_getJointCommands(unsigned int& nt, unsigned int& Dof,std::vector<Eigen::MatrixXd>& alljacobian,std::vector<Eigen::MatrixXd>& allJpre,
                                std::vector<Eigen::VectorXd>& alldel_x,
                                std::vector<Eigen::VectorXd>& alldeisred_x_dot,
                                std::vector<Eigen::VectorXd>& alldeisred_x_2dot,
                                Eigen::MatrixXd& a,Eigen::VectorXd Q,Eigen::VectorXd Q_dot)
{
    
    Eigen::VectorXd Delq = Eigen::VectorXd::Zero(Dof);
    Eigen::VectorXd Desired_q_dot = Eigen::VectorXd::Zero(Dof);
    Eigen::VectorXd Desired_q_2dot = Eigen::VectorXd::Zero(Dof);

    Desired_q_2dot = WBIC_DCpseudoInverse(alljacobian[0],a)*(-alljacobian[0]*Q_dot);
    ////std::cout<<"q_dot: \n"<<Q_dot<<std::endl;
    ////std::cout<<"dfinverse: \n"<<WBIC_DCpseudoInverse(alljacobian[0],a)<<std::endl;
    
    ////std::cout<<"Initail deisred_q_2dot: \n"<<Desired_q_2dot.transpose();
    ////std::cout<<"joint commands iter start"<<std::endl;
    ////std::cout<<"Jpre 1: \n"<<allJpre[0]<<std::endl;
    ////std::cout<<"Jpre 2: \n"<<allJpre[1]<<std::endl;
    ////std::cout<<"Jaboian 0: \n"<<alljacobian[0]<<std::endl;
    ////std::cout<<"Jaboian 1: \n"<<alljacobian[1]<<std::endl;
    ////std::cout<<"Jaboian 2: \n"<<alljacobian[2]<<std::endl;
    for(int i = 0; i<nt-1; i++)
    {
        Delq = Delq + WBIC_pseudoInverse(allJpre[i])*(alldel_x[i] - alljacobian[i+1]*Delq);
        Desired_q_dot = Desired_q_dot + WBIC_pseudoInverse(allJpre[i])*(alldeisred_x_dot[i] - alljacobian[i+1]*Desired_q_dot);
        Desired_q_2dot = Desired_q_2dot + WBIC_DCpseudoInverse(allJpre[i],a)*(alldeisred_x_2dot[i] - alljacobian[i+1]*Desired_q_2dot); 
        ////std::cout<<"desired_x_2dot: \n"<<alldeisred_x_2dot[i].transpose()<<std::endl;    
    }
    //std::cout<<"1deisred_q_2dot: \n"<<Desired_q_2dot.transpose();
    
    
    
    
    
    
    this->desired_q = Q + Delq;
    this->desired_q_dot = Desired_q_dot;
    this->desired_q_2dot = Desired_q_2dot;
}

void WBIC::WBIC_setCartesianCommands(unsigned int& nt, std::vector<Eigen::VectorXd>& alldesired_x,std::vector<Eigen::VectorXd>& alldesired_x_dot,
                                std::vector<Eigen::VectorXd>& alldesired_x_2dot,std::vector<Eigen::VectorXd>& allx,
                                std::vector<Eigen::VectorXd>& allx_dot,
                                std::vector<Eigen::MatrixXd>& Kp,
                                std::vector<Eigen::MatrixXd>& Kd)
{

    this-> desired_x = alldesired_x;
    this-> desired_x_dot = alldesired_x_dot;
    
    this-> del_x.clear();
    this-> desired_x_2dot.clear();

    
     //////std::cout<<"set cartesian commands iter start"<<std::endl;
    for(int i = 0;i<nt-1;i++)
    {   
        //std::cout<<"deisred_x: "<<i<<"\n"<<alldesired_x[i].transpose()<<std::endl;
        //std::cout<<"current_x: "<<i<<"\n"<<allx[i].transpose()<<std::endl;
        //std::cout<<"current_x_dot: "<<i<<"\n"<<allx_dot[i].transpose()<<std::endl;
        
        this-> del_x.push_back(alldesired_x[i] - allx[i]);
        //std::cout<<"del_x: "<<i<<"\n"<<del_x[i].transpose()<<std::endl;
        //std::cout<<"deisred_x_dot: "<<i<<"\n"<<alldesired_x_dot[i].transpose()<<std::endl;
        //std::cout<<"alldeisred_x_2dot: "<<i<<"\n"<<alldesired_x_2dot[i].transpose()<<std::endl;
        
        this-> desired_x_2dot.push_back(alldesired_x_2dot[i] + Kp[i]*(del_x[i]) + Kd[i]*(alldesired_x_dot[i] - allx_dot[i]));
        //std::cout<<"deisred_x_2dot: "<<i<<"\n"<<desired_x_2dot[i].transpose()<<std::endl;
        
        //std::cout<<i<<std::endl;
    } 

    WBIC_setTaskGains(Kp,Kd);   
}

void WBIC::WBIC_castWBIC2qpHessian(unsigned int& dof,Eigen::MatrixXd& q1,Eigen::MatrixXd& q2,Eigen::VectorXd& Fr)
{
    
    
    hessian.resize(6 + dof + 2*Fr.size(),6 + dof + 2*Fr.size());
    // Eigen::MatrixXd Hessian = Eigen::MatrixXd::Identity(6 + dof + 2*Fr.size(),6 + dof + 2*Fr.size());
    
    // Hessian.block(0,0,Fr.size(),Fr.size()) = q1;
    // Hessian.block(Fr.size(),Fr.size(),6,6) = q2;


    //hessian = Hessian.sparseView();
     for(int i = 0;i<2*Fr.size()+6+dof;i++)
     {
         if(i<Fr.size())
         {
             hessian.insert(i,i) = q1(i,i);
         }
         else if(i<Fr.size()+6) 
         {
             hessian.insert(i,i) = q2(i-Fr.size(), i-Fr.size());
         }
         else
         {
            hessian.insert(i,i) = 1;
         }
     }
}
void WBIC::WBIC_castWBIC2qpConstraintMatrix(unsigned int& dof, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>& alljacobian, 
                                Eigen::MatrixXd& a, double& Fc)
{
    this -> linearMatrix.resize(6 + dof + Fr.size() + 5*Fr.size()/3,2*Fr.size()+ dof + 6);
    Eigen::MatrixXd LinearMatrix = Eigen::MatrixXd::Zero(6 + dof + Fr.size() + 5*Fr.size()/3,2*Fr.size()+ dof + 6);
    
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

void WBIC::WBIC_castWBIC2qpConstraintVector(unsigned int dof, Eigen::VectorXd& Fr, Eigen::VectorXd& desired_q_2dot,Eigen::VectorXd& b
                                        ,Eigen::VectorXd& g)
{
    
    
    
    Eigen::MatrixXd Sf = Eigen::MatrixXd::Zero(6,dof);
    Sf.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    if(Fr.size()!=0)
    {   
        ////////std::cout<<"contact detected"<<std::endl;    
        Eigen::VectorXd MinusInfinity = Eigen::VectorXd::Zero(5*Fr.size()/3);
        Eigen::VectorXd zero = Eigen::VectorXd::Zero(5*Fr.size()/3);
        
        lowerBound.resize(dof + 6 + Fr.size() + 5*Fr.size()/3);
        upperBound.resize(dof + 6 + Fr.size() + 5*Fr.size()/3);
        
        for(int i = 0;i<MinusInfinity.size();i++)
        {
            MinusInfinity(i) = -OsqpEigen::INFTY;
        }
        ////////std::cout<<"set contact constraints"<<std::endl;    
        Eigen::VectorXd model_constraint = -Sf*b-Sf*g; 


        lowerBound << desired_q_2dot, model_constraint, Fr, zero;
        upperBound << desired_q_2dot, model_constraint, Fr, -1*MinusInfinity;
        

    }
    else
    {
        lowerBound.resize(dof + 6);
        upperBound.resize(dof + 6);
        
        lowerBound << desired_q_2dot, -Sf*b-Sf*g;
        upperBound << desired_q_2dot, -Sf*b-Sf*g;
    }
   
}

void WBIC::WBIC_castWBIC2qpGradient(unsigned int& dof, Eigen::VectorXd& Fr)
{
    gradient.resize(2*Fr.size() + dof + 6);
    this -> gradient = Eigen::VectorXd::Zero(2*Fr.size() + dof + 6);
}

void WBIC::WBIC_setFrictinConstant(double& Fc)
{
    this -> fc = Fc;
}

bool WBIC::WBIC_Init(unsigned int& nt, unsigned int& dof, Eigen::MatrixXd& q1,Eigen::MatrixXd& q2, Eigen::VectorXd& Fr, std::vector<Eigen::MatrixXd>&alljacobian,
                                Eigen::MatrixXd& a, Eigen::VectorXd& b, Eigen::VectorXd& g,double& Fc, Eigen::VectorXd Q,Eigen::VectorXd Q_dot
                                ,OsqpEigen::Solver& solver)
{
    this-> Nt = nt;
    this-> Dof = dof;
    this-> fc = Fc;
    
    WBIC_setModel(a,b,g);
    //std::cout<<"set model"<<std::endl;
    WBIC_setJointStates(Q, Q_dot);
    //std::cout<<"set Joint states"<<std::endl;
    WBIC_setFrictinConstant(Fc);
    //std::cout<<"set fc"<<std::endl;
    WBIC_setWeightMatrices(q1,q2);
    //std::cout<<"set weight"<<std::endl;
    WBIC_setJacobianMatrices(alljacobian);
    //std::cout<<"set jacobians"<<std::endl;
    WBIC_setJpre(nt,dof,alljacobian);
    //std::cout<<"set jpre"<<std::endl;
    WBIC_getJointCommands(nt, dof, alljacobian, Jpres, del_x, desired_x_dot, desired_x_2dot, a, Q, Q_dot);
    //std::cout<<"get joint commands"<<std::endl;
    ////std::cout<<"desired x 2 dot 0: \n"<<desired_x_2dot[0].transpose()<<std::endl;
    ////std::cout<<"desired x 2 dot 1: \n"<<desired_x_2dot[1].transpose()<<std::endl;
    
    
    //////std::cout<<"q1: \n"<<q1<<std::endl;
    //////std::cout<<"q2: \n"<<q2<<std::endl;
    WBIC_castWBIC2qpHessian(dof,q1,q2,Fr);
    //std::cout<<"set hessian"<<std::endl;
    //////std::cout<<hessian<<std::endl;
    
    WBIC_castWBIC2qpGradient(dof,Fr);
    //std::cout<<"set gradient"<<std::endl;
    //////std::cout<<gradient<<std::endl;
    
    WBIC_castWBIC2qpConstraintMatrix(dof, Fr, alljacobian, a, Fc);
    //std::cout<<"set constraint"<<std::endl;
    //////std::cout<<linearMatrix<<std::endl;
    
    WBIC_castWBIC2qpConstraintVector(dof, Fr, desired_q_2dot, b, g);
    //std::cout<<"set constraint vector"<<std::endl;
    //////std::cout<<lowerBound<<std::endl;
    //////std::cout<<upperBound<<std::endl;
    
    solver.settings()->setWarmStart(false);

    //////std::cout<<" set the initial data of the QP solver"<<std::endl;
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

void WBIC::WBIC_solve_problem(unsigned int& dof, Eigen::VectorXd& Fr,OsqpEigen::Solver& solver)
{
    // solve the QP problem
    solver.solveProblem();
    motor_cmd.clear();
    // get the controller input
    QPSolution = solver.getSolution();
    ctr.resize(dof + Fr.size());
    ctr = QPSolution.block(6 + Fr.size(), 0, dof + Fr.size(), 1);
    
    q2dot = ctr.block(0,0,dof,1);
    desired_fr = ctr.block(dof,0,Fr.size(),1);
    //std::cout<<"desired fr: \n"<<desired_fr.transpose()<<std::endl;
    //std::cout<<" q2dot: \n"<<q2dot.transpose()<<std::endl;
    //std::cout<<" a: \n"<<A<<std::endl;
    //std::cout<<" B: \n"<<B.transpose()<<std::endl;
    
    tau = A*q2dot + B + G - jacobians[0].transpose()*desired_fr;
    
    //std::cout<<"desired q2dot: \n"<<desired_q_2dot.block(0,0,dof,1).transpose()<<std::endl;
    //std::cout<<" tau: \n"<<tau.block(0,0,dof,1).transpose()<<std::endl;
   
    
    motor_cmd.push_back(desired_q.block(6,0,dof-6,1));
    motor_cmd.push_back(desired_q_dot.block(6,0,dof-6,1));
    motor_cmd.push_back(tau.block(6,0,dof-6,1)); 
    
}

std::vector<Eigen::VectorXd> WBIC::WBIC_getCtr()
{
    return this -> motor_cmd;
}
