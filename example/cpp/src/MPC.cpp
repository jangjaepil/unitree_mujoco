#include "MPC.hpp"

void MPC::setDynamicsMatrices(double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, std::vector<Eigen::VectorXd>& r,double m,int rSize)
{
    a = Eigen::MatrixXd::Identity(15,15);
    a.block(0,6,3,3) = Rz*delT;
    a.block(3,9,3,3) = delT*Eigen::MatrixXd::Identity(3,3);
    a.block(9,12,3,3) = (delT/m)*Eigen::MatrixXd::Identity(3,3);
    
    b = Eigen::MatrixXd::Zero(15,3*rSize);
    //std::cout<<"gI: \n"<<gI<<std::endl;
    for(int i = 0;i<rSize;i++)
    {
        Eigen::MatrixXd upR = Eigen::MatrixXd::Zero(3,3);
        Eigen::VectorXd rV = r[i];
                    upR << 0,-rV(2),rV(1),
                            rV(2),0,-rV(0),
                            -rV(1),rV(0),0;
        b.block(6,3*i,3,3) = gI.inverse()*upR*delT;
        //std::cout<<"rV: \n"<<rV.transpose()<<std::endl;
        //std::cout<<"gI inverse: \n"<<gI.inverse()<<std::endl;
        //std::cout<<"upR: \n"<<upR<<std::endl;
        //std::cout<<"delT: \n"<<delT<<std::endl;
        b.block(9,3*i,3,3) = Eigen::MatrixXd::Identity(3,3)*(delT/m);
    }

}

void MPC::setInequalityConstraints(Eigen::VectorXd& xMax,
                              Eigen::VectorXd& xMin,
                              Eigen::VectorXd& uMax,
                              Eigen::VectorXd& uMin,
                              int rSize)
{
    uMin.resize(3*rSize);
    uMax.resize(3*rSize);
    xMin.resize(15);
    xMax.resize(15);
    double m = 47;
    // input inequality constraints
    for(int i = 0; i<3*rSize;i++)
    {
        uMin(i) = 0;
        uMax(i) = 5000;    
    }
    
    // state inequality constraints
    xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY,
            -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY,-OsqpEigen::INFTY,
            0, 0, -9.8*m;

    xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY,
            OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, 0, 0, -9.8*m;
    
}

void MPC::setWeightMatrices(Eigen::MatrixXd& Qn, Eigen::MatrixXd& Q, Eigen::MatrixXd& R,int rSize)
{
    this->Q = Eigen::MatrixXd::Identity(15,15);
    this->Qn = Eigen::MatrixXd::Identity(15,15);
    this->R = Eigen::MatrixXd::Identity(3*rSize,3*rSize);
    
    this-> Qn.diagonal()<<100,100,100,100,100,100,100,100,100,100,100,100,0,0,0;
    this-> Q.diagonal()<<100,100,100,100,100,100,100,100,100,100,100,100,0,0,0;
    this-> R = R*0.008;
}

void MPC::castMPCToQPHessian(const Eigen::MatrixXd& Qn, const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R,
                        int mpcWindow,
                        int rSize,
                        Eigen::SparseMatrix<double>& hessianMatrix)
{

    hessianMatrix.resize(15 * (mpcWindow + 1) + 3*rSize * mpcWindow,
                         15 * (mpcWindow + 1) + 3*rSize * mpcWindow);

    // populate hessian matrix
    for (int i = 0; i < 15 * (mpcWindow + 1) + 3*rSize * mpcWindow; i++)
    {
        if (i < 15 * (mpcWindow + 1))
        {
            if(i >= 15 * (mpcWindow))
            {
                int posQ = i % 15;
                float value = Qn.diagonal()[posQ];
                if (value != 0)
                hessianMatrix.insert(i, i) = value;
            }
            else
            {
                int posQ = i % 15;
                float value = Q.diagonal()[posQ];
                if (value != 0)
                hessianMatrix.insert(i, i) = value;
            }
            
        } 
        else
        {
            int posR = i % (3*rSize);
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
    }
  
}

void MPC::castMPCToQPGradient(const Eigen::MatrixXd& Qn,const Eigen::MatrixXd& Q,
                         const Eigen::VectorXd& xRef,
                         int mpcWindow,
                         int rSize,
                         Eigen::VectorXd& gradient)
{

    Eigen::VectorXd Qx_ref;
    Eigen::VectorXd Qnx_ref;
    Qx_ref = -xRef.transpose()*Q;
    Qnx_ref = -xRef.transpose()*Qn;
    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(15 * (mpcWindow + 1) + 3*rSize * mpcWindow, 1);
    for (int i = 0; i < 15 * (mpcWindow + 1); i++)
    {
        if(i>=15 * (mpcWindow))
        {
            int posQ = i % 15;
            float value = Qnx_ref(posQ, 0);
            gradient(i, 0) = value;
        }
        else
        {
            int posQ = i % 15;
            float value = Qx_ref(posQ, 0);
            gradient(i, 0) = value;
        }
    }
}

void MPC::castMPCToQPConstraintMatrix(const Eigen::MatrixXd& dynamicMatrix,
                                 const Eigen::MatrixXd& controlMatrix,
                                 int mpcWindow,
                                 int rSize,
                                 Eigen::SparseMatrix<double>& constraintMatrix)
{
    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(15 * (mpcWindow + 1) + 15 * (mpcWindow + 1) + 3*rSize * mpcWindow,
                            15 * (mpcWindow + 1) + 3*rSize * mpcWindow);

    
    // populate linear constraint matrix
    for (int i = 0; i < 15 * (mpcWindow + 1); i++)
    {
        temp(i, i) = -1;
    }
    
    for (int i = 0; i< mpcWindow;i++)
    {
        temp.block(i*15+15,i*15,15,15) = dynamicMatrix;
        temp.block(i*15+15,15*(mpcWindow+1)+(3*rSize*i),15,3*rSize) = controlMatrix;
    }
    
    for (int i = 0; i < 15 * (mpcWindow + 1) + 3*rSize * mpcWindow; i++)
    {
        temp(i + (mpcWindow + 1) * 15, i) = 1;
    }
    
    constraintMatrix.resize(15 * (mpcWindow + 1) + 15 * (mpcWindow + 1) + 3*rSize * mpcWindow,
                            15 * (mpcWindow + 1) + 3*rSize * mpcWindow);
    constraintMatrix = temp.sparseView();
    
}

void MPC::castMPCToQPConstraintVectors(const Eigen::VectorXd& xMax,
                                  const Eigen::VectorXd& xMin,
                                  const Eigen::VectorXd& uMax,
                                  const Eigen::VectorXd& uMin,
                                  const Eigen::VectorXd& x0,
                                  int mpcWindow,
                                  int rSize,
                                  Eigen::VectorXd& lowerBound,
                                  Eigen::VectorXd& upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality
        = Eigen::MatrixXd::Zero(15 * (mpcWindow + 1) + 3*rSize * mpcWindow, 1);
    Eigen::VectorXd upperInequality
        = Eigen::MatrixXd::Zero(15 * (mpcWindow + 1) + 3*rSize * mpcWindow, 1);
    for (int i = 0; i < mpcWindow + 1; i++)
    {
        lowerInequality.block(15 * i, 0, 15, 1) = xMin;
        upperInequality.block(15 * i, 0, 15, 1) = xMax;
    }
    for (int i = 0; i < mpcWindow; i++)
    {
        lowerInequality.block(3*rSize * i + 15 * (mpcWindow + 1), 0, 3*rSize, 1) = uMin;
        upperInequality.block(3*rSize * i + 15 * (mpcWindow + 1), 0, 3*rSize, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(15 * (mpcWindow + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, 15, 1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2 * 15 * (mpcWindow + 1) + 3*rSize * mpcWindow, 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2 * 15 * (mpcWindow + 1) + 3*rSize * mpcWindow, 1);
    upperBound << upperEquality, upperInequality;
}

void MPC::updateConstraintVectors(const Eigen::VectorXd& x0,
                             Eigen::VectorXd& lowerBound,
                             Eigen::VectorXd& upperBound)
{
    lowerBound.block(0, 0, 15, 1) = -x0;
    upperBound.block(0, 0, 15, 1) = -x0;
}

double MPC::getErrorNorm(const Eigen::VectorXd& x, const Eigen::VectorXd& xRef)
{
    // evaluate the error
    Eigen::VectorXd error = x - xRef;

    // return the norm
    return error.norm();
}
bool MPC::init(Eigen::VectorXd& x_init,Eigen::VectorXd& x_ref,double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, std::vector<Eigen::VectorXd>& r,double m,OsqpEigen::Solver& solver)
{   
    mpcWindow = 10;
    int rSize = r.size();
    this -> rSize = r.size();
     std::cout<<"rSize: "<<rSize<<std::endl;
     std::cout<<"set the initial and the desired states"<<std::endl;
    x0  = x_init;
    xRef = x_ref;
    


    std::cout<<"set MPC problem quantities"<<std::endl;
    setDynamicsMatrices(delT, Rz, gI, r, m, rSize); //delT,Rz,Inertia,r1,r2,m
    std::cout<<"set dynamics matrices"<<std::endl;
    // std::cout<<"a: \n"<<a<<std::endl;
    // std::cout<<"b: \n"<<b<<std::endl;
    
    setInequalityConstraints(xMax, xMin, uMax, uMin,rSize);
     std::cout<<"set inequality constraints"<<std::endl;
    
    setWeightMatrices(Qn, Q, R,rSize);
    std::cout<<"set weights"<<std::endl;
    // std::cout<<"Qn: \n"<<Qn<<std::endl;
    // std::cout<<"Q: \n"<<Q<<std::endl;
    // std::cout<<"R: \n"<<R<<std::endl;
    
    std::cout<<"cast the MPC problem as QP problem"<<std::endl;
    castMPCToQPHessian(Qn, Q, R, mpcWindow, rSize, hessian);
    //std::cout<<"Hessian: \n"<<hessian<<std::endl;
 
    
    castMPCToQPGradient(Qn, Q, xRef, mpcWindow, rSize, gradient);
    //std::cout<<"gradient:\n"<<gradient.transpose()<<std::endl;
 
    castMPCToQPConstraintMatrix(a, b, mpcWindow, rSize, linearMatrix);
    //std::cout<<"constraint matrix:\n"<<linearMatrix<<std::endl;
    
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, rSize, lowerBound, upperBound);
    // std::cout<<"lower bounds:\n"<<lowerBound.transpose()<<std::endl;
    // std::cout<<"upper bounds:\n"<<upperBound.transpose()<<std::endl;
  
   
    solver.settings()->setWarmStart(false);

    std::cout<<" set the initial data of the QP solver"<<std::endl;
    solver.data()->setNumberOfVariables(15 * (mpcWindow + 1) + 3*rSize * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 15 * (mpcWindow + 1) + 3*rSize * mpcWindow);
    if (!solver.data()->setHessianMatrix(hessian))
    {   std::cout<<"hessian1"<<std::endl;
        return 1;
    }
    if (!solver.data()->setGradient(gradient))
    {   std::cout<<"gradient"<<std::endl;
        return 1;
    }
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
    {   std::cout<<"linearMatrix"<<std::endl;
        return 1;
    }
    if (!solver.data()->setLowerBound(lowerBound))
    {   std::cout<<"lowerBound"<<std::endl;
        return 1;
    }
    if (!solver.data()->setUpperBound(upperBound))
    {   std::cout<<"upperBound"<<std::endl;       
        return 1;
    }
    // instantiate the solver
    if (!solver.initSolver())
    {   std::cout<<"init"<<std::endl;
        return 1;
    }
    
    return 0;
}

bool MPC::solveProblem(OsqpEigen::Solver& solver)
{

        std::cout<<"xRef: "<<xRef.transpose()<<std::endl;
        
        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        QPSolution = solver.getSolution();
        ctr = QPSolution.block(15 * (mpcWindow + 1), 0, 3*rSize, 1);
        //ctr = QPSolution;
        //x0 = a*x0 + b* ctr;
      
    return 0;
}

bool MPC::updateProblem(double delT, Eigen::Matrix3d& Rz, Eigen::Matrix3d& gI, Eigen::VectorXd x0, 
                        std::vector<Eigen::VectorXd>& r, double m,OsqpEigen::Solver& solver)
{
    
            // update the model, gradient, constraint matrix
        setDynamicsMatrices(delT, Rz, gI, r, m, rSize);
        std::cout<<"Update dynamics"<<std::endl;

        castMPCToQPConstraintMatrix(a, b, mpcWindow, rSize, linearMatrix);
        std::cout<<"Update constraint matrix"<<std::endl;

        setWeightMatrices(Qn, Q, R,rSize);


        castMPCToQPHessian(Qn, Q, R, mpcWindow, rSize, hessian);

        setInequalityConstraints(xMax, xMin, uMax, uMin,rSize);

        castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, rSize, lowerBound, upperBound);

        castMPCToQPGradient(Qn, Q, xRef, mpcWindow, rSize, gradient);

    
        if (!solver.updateHessianMatrix(hessian))
        {    std::cout<<"hessian"<<std::endl;
            return 1;
        }
        if (!solver.updateLinearConstraintsMatrix(linearMatrix))
        {    std::cout<<"linearMatrix"<<std::endl;
            return 1;
        }
        if (!solver.updateBounds(lowerBound,upperBound))
        {    std::cout<<"Bound"<<std::endl;
            return 1;
        }    
        if (solver.updateGradient(gradient))
        {    std::cout<<"gradient"<<std::endl;
            return 1;
        }
    //}
    
    
    
    
   
    return 0;
}

Eigen::VectorXd MPC::getStates()
{
    return x0;
}

Eigen::VectorXd MPC::getCtr()
{
    return ctr;
}

void MPC::setRef(Eigen::VectorXd dx)
{
    xRef = dx;
}



