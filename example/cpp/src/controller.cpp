#include <controller.hpp>
#include <thread>

controller::controller()
{
    
}

void controller::State2Vector(const unitree_go::msg::dds_::LowState_ low_state, const unitree_go::msg::dds_::SportModeState_ high_state)
{   
    

    imu_quat.w() = low_state.imu_state().quaternion()[0];
    imu_quat.x() = low_state.imu_state().quaternion()[1];
    imu_quat.y() = low_state.imu_state().quaternion()[2];
    imu_quat.z() = low_state.imu_state().quaternion()[3]; //world

    imu_gyr(0) = low_state.imu_state().gyroscope()[0];
    imu_gyr(1) = low_state.imu_state().gyroscope()[1];
    imu_gyr(2) = low_state.imu_state().gyroscope()[2]; // ref base
    
    imu_acc(0) = low_state.imu_state().accelerometer()[0];
    imu_acc(1) = low_state.imu_state().accelerometer()[1];
    imu_acc(2) = low_state.imu_state().accelerometer()[2]; // ref base

    base_pos(0) = high_state.position()[0];
    base_pos(1) = high_state.position()[1];
    base_pos(2) = high_state.position()[2];  //ref world
    
    base_vel(0) = high_state.velocity()[0]; //ref world
    base_vel(1) = high_state.velocity()[1];
    base_vel(2) = high_state.velocity()[2];  
    
    Eigen::VectorXd base_ang_vel = imu_gyr;
     
    q(7)=low_state.motor_state()[3].q();
    q(8)=low_state.motor_state()[4].q();
    q(9)=low_state.motor_state()[5].q();
    q(10)=low_state.motor_state()[0].q();
    q(11)=low_state.motor_state()[1].q();
    q(12)=low_state.motor_state()[2].q();
    q(13)=low_state.motor_state()[9].q();
    q(14)=low_state.motor_state()[10].q();
    q(15)=low_state.motor_state()[11].q();
    q(16)=low_state.motor_state()[6].q();
    q(17)=low_state.motor_state()[7].q();
    q(18)=low_state.motor_state()[8].q();
   
    
    dq(6)=low_state.motor_state()[3].dq();
    dq(7)=low_state.motor_state()[4].dq();
    dq(8)=low_state.motor_state()[5].dq();
    dq(9)=low_state.motor_state()[0].dq();
    dq(10)=low_state.motor_state()[1].dq();
    dq(11)=low_state.motor_state()[2].dq();
    dq(12)=low_state.motor_state()[9].dq();
    dq(13)=low_state.motor_state()[10].dq();
    dq(14)=low_state.motor_state()[11].dq();
    dq(15)=low_state.motor_state()[6].dq();
    dq(16)=low_state.motor_state()[7].dq();
    dq(17)=low_state.motor_state()[8].dq();
    

    tau(0)=low_state.motor_state()[3].tau_est();
    tau(1)=low_state.motor_state()[4].tau_est();
    tau(2)=low_state.motor_state()[5].tau_est();
    tau(3)=low_state.motor_state()[0].tau_est();
    tau(4)=low_state.motor_state()[1].tau_est();
    tau(5)=low_state.motor_state()[2].tau_est();
    tau(6)=low_state.motor_state()[9].tau_est();
    tau(7)=low_state.motor_state()[10].tau_est();
    tau(8)=low_state.motor_state()[11].tau_est();
    tau(9)=low_state.motor_state()[6].tau_est();
    tau(10)=low_state.motor_state()[7].tau_est();
    tau(11)=low_state.motor_state()[8].tau_est();
    
    
    q(0) = base_pos(0);
    q(1) = base_pos(1);
    q(2) = base_pos(2);
    q(3) = imu_quat.x(); // For pinocchio lib repersentation order
    q(4) = imu_quat.y();
    q(5) = imu_quat.z();
    q(6) = imu_quat.w();

    wRb = imu_quat.toRotationMatrix();
    Eigen::VectorXd base_velB = wRb.transpose()*base_vel;

    dq(0) = base_velB(0); //ref: body frame
    dq(1) = base_velB(1);
    dq(2) = base_velB(2);
    dq(3) = base_ang_vel(0); // ref: body frame
    dq(4) = base_ang_vel(1);
    dq(5) = base_ang_vel(2);
    
    euler_angle = imu_quat.toRotationMatrix().eulerAngles(0,1,2); //roll pitch yaw
    
    q_wbic = Eigen::VectorXd::Zero(dof);
    //qd_wbic = Eigen::VectorXd::Zero(dof);
    q_wbic << base_pos,euler_angle,q.block(7,0,dof-6,1);
    //qd_wbic << base_vel,oriVel,dq.block(6,0,dof-6,1);
    
   
    oriVel =  wRb*base_ang_vel; 
    
    Rz <<cos(euler_angle(2)), 0, -sin(euler_angle(2)),  0,
                        0, 1, 0, 0,
                        sin(euler_angle(2)), 0, cos(euler_angle(2)), 0,
                        0, 0, 0, 1;

    Rz = Eigen::Matrix3d::Identity(3,3);
    mpc_states<< euler_angle,base_pos, oriVel, base_vel,0,0,-9.8*m;//euler angle,position,angular velocity, linear velocity, gravity
    
        
    //std::cout<<"q: \n"<<q.transpose()<<std::endl;
    //std::cout<<"dq: \n"<<dq.transpose()<<std::endl;
    //std::cout<<"tau: \n"<<tau.transpose()<<std::endl;
    //std::cout<<"imu_quat: \n"<<imu_quat.x()<<"\n"<< imu_quat.y()<<"\n"<<imu_quat.z()<<"\n"<<imu_quat.w()<<std::endl;
    //std::cout<<"imu_gry: \n"<<imu_gyr.transpose()<<std::endl;
    //std::cout<<"imu_acc: \n"<<imu_acc.transpose()<<std::endl;
    
    //std::cout<<"high_state_pos: "<<high_state.position()[0]<<std::endl;
    std::cout<<"base_pos: \n"<<base_pos.transpose()<<std::endl;
    std::cout<<"base_vel: \n"<<base_vel.transpose()<<std::endl;
    
    if(state_flag == 0)
    {    
        for(int i = 0; i< number_of_joints ;i++)
        {
            if(q(i + 7) != 0)
            {
                state_flag = 1;
                break;
            }
        }
    }   

}

void controller::getmodel(pinocchio::Model &model, pinocchio::Data &data)
{
    
    pinocchio::computeAllTerms(model, data, q, dq);   
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);   
    a = data.M.triangularView<Eigen::Upper>();
    a.triangularView<Eigen::StrictlyLower>() = a.transpose();
    
    b = data.C*dq;
    g = data.g;
    
    // std::cout<< "a : \n"<<a<<std::endl;
    // std::cout<< "b : \n"<<b<<std::endl;
    // std::cout<< "g : \n"<<g.transpose()<<std::endl;

    pinocchio::centerOfMass(model,data,q,dq,false);
    CoM = data.com[0];
    CoMV = data.vcom[0];
    Jcom = Eigen::MatrixXd::Zero(6,dof);
    Jcom = pinocchio::jacobianCenterOfMass(model,data,false);
    
    //torso orientation, swing foot position, joint pose? 
    std::string frame_name[5];
    frame_name[0] = "base";
    frame_name[1] = "RL_foot";
    frame_name[2] = "RR_foot";
    frame_name[3] = "FL_foot";
    frame_name[4] = "FR_foot";
   
    
    Jbody = Eigen::MatrixXd::Zero(3,dof);
    Jrl = Eigen::MatrixXd::Zero(3,dof);
    Jrr = Eigen::MatrixXd::Zero(3,dof);
    Jfl = Eigen::MatrixXd::Zero(3,dof);
    Jfr = Eigen::MatrixXd::Zero(3,dof);
    
    Jpose = Eigen::MatrixXd::Identity(dof,dof);
    JbodyOri = Eigen::MatrixXd::Zero(3,dof);
    Jf = Eigen::MatrixXd::Zero(12,dof);
    Jc = Eigen::MatrixXd::Zero(12,dof);
    
    //contact positions for mpc
    Eigen::VectorXd rl  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rr  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd fl  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd fr  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd base = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd frame_position = Eigen::VectorXd::Zero(3);
    
    int length = sizeof(frame_name)/sizeof(frame_name[0]);
    pinocchio::FrameIndex tcp_idx[length];
    
    for(int i = 0 ; i<length;i++)
    {   
        tcp_idx[i] = model.getFrameId(frame_name[i]);
        jacobian = Eigen::MatrixXd::Zero(6,dof);
        pinocchio::computeFrameJacobian(model, data, q,tcp_idx[i], pinocchio::ReferenceFrame::WORLD, jacobian);
        frame_position = data.oMf[tcp_idx[i]].translation();
        
        // std::cout << "tcp idx " << tcp_idx[i] << ": Name = " << model.frames[tcp_idx[i]].name << std::endl;

        if(i==0) 
        {   
            
            Jbody = jacobian.block(0,0,3,dof);
            JbodyOri = jacobian.block(3,0,3,dof);
            base = frame_position;
            
        }
        if(i==1)
        {
            
            Jrl = jacobian.block(0,0,3,dof);
            rl = frame_position;
        }
        if(i==2)
        {
            rr = frame_position;
            
            Jrr = jacobian.block(0,0,3,dof);
            
            
        }
        if(i==3)
        {
            Jfl = jacobian.block(0,0,3,dof);
            fl = frame_position;
        }
        if(i==4)
        {
            Jfr = jacobian.block(0,0,3,dof);
            fr = frame_position;
        }
        
    }
    
    
    Jf.block(0,0,3,dof) = Jfl;
    Jf.block(3,0,3,dof) = Jfr;
    Jf.block(6,0,3,dof) = Jrl;
    Jf.block(9,0,3,dof) = Jrr;
    //std::cout<<"contact jacobian"<<std::endl;
    std::cout<<"calculated base vel: \n"<<Jbody*dq<<JbodyOri*dq<<std::endl;
    std::cout<<"meausred base vel: \n"<<base_vel<<" "<<oriVel<<std::endl;
   
    
    Jc = Jf;
    //Jp = Eigen::MatrixXd::Zero()
    
    std::cout<<"JbodyOri: "<<JbodyOri<<std::endl;
    std::cout<<"Jbody: "<<Jbody<<std::endl;
    
    std::cout<<"Jc: \n"<<Jc<<std::endl;
    std::cout<<"Jfl: \n"<<Jfl<<std::endl;
    std::cout<<"Jfr: \n"<<Jfr<<std::endl;
    std::cout<<"Jrl: \n"<<Jrl<<std::endl;
    std::cout<<"Jrr: \n"<<Jrr<<std::endl;
   


    alljacobian.clear();
    alljacobian.push_back(Jc);
    alljacobian.push_back(JbodyOri);
    alljacobian.push_back(Jbody);
   
    // std::cout<<"joint q: "<<q.transpose()<<std::endl;
    // std::cout<<"torso: "<<base.transpose()<<std::endl;
    // std::cout<<"rear left: "<<rl.transpose()<<std::endl;
    // std::cout<<"rear right: "<<rr.transpose()<<std::endl;
    // std::cout<<"front left: "<<fl.transpose()<<std::endl;
    // std::cout<<"front right: "<<fr.transpose()<<std::endl;
    
    r.clear();
    r.push_back(fl);
    r.push_back(fr);
    r.push_back(rl);
    r.push_back(fr);
    

    //Task current states
    Eigen::VectorXd fakeOrientation = Eigen::VectorXd::Zero(3);
    allx.clear();
    allx_dot.clear();

    allx.push_back(fakeOrientation);
    allx_dot.push_back(oriVel);

    allx.push_back(base_pos);
    allx_dot.push_back(base_vel);
    //std::cout<<"base_pose(urdf): \n"<<base.transpose()<<std::endl;
    
    
    
}
void controller::setDesireds()
{
    Eigen::VectorXd DesiredBodyOrientation  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyPosition  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredFootPositions  = Eigen::VectorXd::Zero(6);
    
    Eigen::VectorXd DesiredBodyOriVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredFootVels  = Eigen::VectorXd::Zero(6);
    
    Eigen::VectorXd DesiredBodyOriAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredFootAccs  = Eigen::VectorXd::Zero(6);
    

    DesiredBodyPosition<<0,0,0.14;
    DesiredFootPositions<<0,-0.2,0,0,0.2,0;
    DesiredBodyVel  << 0,0,0;
    x_ref << DesiredBodyOrientation,DesiredBodyPosition,DesiredBodyOriVel,DesiredBodyVel,0,0,-9.8*m;
    

    Eigen::Matrix3d wRd = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d bRd = wRb.transpose()*wRd;
    Eigen::Matrix3d wRe = wRb*bRd;
    
    Eigen::AngleAxisd AngleAxis(wRe);
    Eigen::VectorXd axis = AngleAxis.axis();
    double radian = AngleAxis.angle();

    DesiredBodyOrientation  = radian * axis;
    
    alldesired_x.clear();
    alldesired_x_dot.clear();
    alldesired_x_2dot.clear();

    alldesired_x.push_back(DesiredBodyOrientation);
    alldesired_x.push_back(DesiredBodyPosition);
    //alldesired_x.push_back(DesiredFootPositions);

    alldesired_x_dot.push_back(DesiredBodyOriVel);
    alldesired_x_dot.push_back(DesiredBodyVel);
    //alldesired_x_dot.push_back(DesiredFootVels);
    
    alldesired_x_2dot.push_back(DesiredBodyOriAcc);
    alldesired_x_2dot.push_back(DesiredBodyAcc);
    //alldeisred_x_2dot.push_back(DesiredFootAccs);

    Eigen::MatrixXd kp_temp = 10*Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd kd_temp = 1*Eigen::MatrixXd::Identity(3,3);
        
    for(int i = 0;i<alldesired_x.size();i++)
    {
    
        kp.push_back(kp_temp);
        kd.push_back(kd_temp);
    }
    
}
void controller::convertCmd(std::vector<Eigen::VectorXd>& Cmds)
{
        Eigen::VectorXd q_cmd = Cmds[0];
        Eigen::VectorXd dq_cmd = Cmds[1];
        Eigen::VectorXd tau_cmd = Cmds[2];

        
        Eigen::VectorXd Crt_q = Eigen::VectorXd::Zero(dof -6);
        Eigen::VectorXd Crt_dq = Eigen::VectorXd::Zero(dof -6);
        Eigen::VectorXd Crt_tau = Eigen::VectorXd::Zero(dof -6);
        
        Crt_q(3) = q_cmd(0);
        Crt_q(4) = q_cmd(1);
        Crt_q(5) = q_cmd(2);
        Crt_q(0) = q_cmd(3);
        Crt_q(1) = q_cmd(4);
        Crt_q(2) = q_cmd(5);
        Crt_q(9) = q_cmd(6);
        Crt_q(10) = q_cmd(7);
        Crt_q(11) = q_cmd(8);
        Crt_q(6) = q_cmd(9);
        Crt_q(7) = q_cmd(10);
        Crt_q(8) = q_cmd(11);
       

        Crt_dq(3) = dq_cmd(0);
        Crt_dq(4) = dq_cmd(1);
        Crt_dq(5) = dq_cmd(2);
        Crt_dq(0) = dq_cmd(3);
        Crt_dq(1) = dq_cmd(4);
        Crt_dq(2) = dq_cmd(5);
        Crt_dq(9) = dq_cmd(6);
        Crt_dq(10) = dq_cmd(7);
        Crt_dq(11) = dq_cmd(8);
        Crt_dq(6) = dq_cmd(9);
        Crt_dq(7) = dq_cmd(10);
        Crt_dq(8) = dq_cmd(11);
        

        Crt_tau(3) = tau_cmd(0);
        Crt_tau(4) = tau_cmd(1);
        Crt_tau(5) = tau_cmd(2);
        Crt_tau(0) = tau_cmd(3);
        Crt_tau(1) = tau_cmd(4);
        Crt_tau(2) = tau_cmd(5);
        Crt_tau(9) = tau_cmd(6);
        Crt_tau(10) = tau_cmd(7);
        Crt_tau(11) = tau_cmd(8);
        Crt_tau(6) = tau_cmd(9);
        Crt_tau(7) = tau_cmd(10);
        Crt_tau(8) = tau_cmd(11);
       
        // std::cout<<"cmds: "<<std::endl;
        // std::cout<<"q: \n"<<Crt_q.transpose()<<std::endl;
        // std::cout<<"dq: \n"<<Crt_dq.transpose()<<std::endl;
        // std::cout<<"tau: \n"<<Crt_tau.transpose()<<std::endl;
        setCmd(Crt_q,Crt_dq,Crt_tau);
        
}

void controller::run()
{   
    std::string urdf_filename = "/home/jang/unitree_mujoco/example/cpp/robots/go2_description/urdf/go2_description.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(),model);
    pinocchio::Data data(model);
    std::cout<<"model.nq: "<<model.nq<<std::endl;
    int total_joints = model.joints.size();

    // Number of physical joints (excluding the virtual root and floating base joint)
    number_of_joints = total_joints - 2;
    
    // // Print the joint information
    // std::cout << "Total number of joints (including virtual root and floating base joint): " << total_joints << std::endl;
    // std::cout << "Number of physical joints: " << number_of_joints << std::endl;

    // Optionally, list all joints with their IDs and names
    std::cout << "List of joints (ID and Name):" << std::endl;
    for (int joint_id = 0; joint_id < total_joints; ++joint_id) {
        std::string joint_name = model.names[joint_id];
        std::cout << "Joint ID " << joint_id << ": Name = " << joint_name << std::endl;
    }
   
    q = Eigen::VectorXd::Zero(number_of_joints + 6+1);
    dq = Eigen::VectorXd::Zero(number_of_joints + 6);
    tau = Eigen::VectorXd::Zero(number_of_joints);
    dof = 6 + number_of_joints;
    unsigned int nt = 3; // contact constraint,body orientation task, CoM position task, 
    
    m = 0.0;
    for(size_t i = 0; i < model.inertias.size(); ++i) {
        m += model.inertias[i].mass();
    }
    std::cout << "Total mass of the robot: " << m << " kg" << std::endl;
    //m = 47;

    double delT = 0.04;
    double Fc = 0.01;
    
    Eigen::Matrix3d bI = 40*Eigen::MatrixXd::Identity(3,3);
 
    MPC mpc;
    WBIC wbic;
    
    while (1) 
    {
        
        State2Vector(getLowState(),getHighState());
        
        if(state_flag == 1)
        {   
            Eigen::Matrix3d gI = Rz*bI*Rz.transpose();
            getmodel(model, data);
            setDesireds();
            OsqpEigen::Solver qp;
            mpc.init(mpc_states,x_ref,delT,Rz,gI,r,m,Fc,qp); 
            mpc.solveProblem(qp);
            Eigen::VectorXd Fr =  mpc.getCtr();
            std::cout<<"Fr: "<<Fr.transpose()<<std::endl;
            qp.clearSolver();

            rSize = r.size();
            Eigen::MatrixXd q1 = 10*Eigen::MatrixXd::Identity(3*rSize,3*rSize);
            Eigen::MatrixXd q2 = 10*Eigen::MatrixXd::Identity(3*rSize,3*rSize);


            OsqpEigen::Solver QP;

            wbic.WBIC_setCartesianCommands(nt,alldesired_x, alldesired_x_dot, alldesired_x_2dot, allx, allx_dot, kp, kd);
            wbic.WBIC_Init(nt, dof,  q1, q2, Fr, alljacobian, a, b, g, Fc, q_wbic, dq ,QP);

            wbic.WBIC_solve_problem(dof, Fr,QP); 
            std::vector<Eigen::VectorXd> cmd = wbic.WBIC_getCtr(); 
            convertCmd(cmd);



            QP.clearSolver();
    
        }
               
       
    }
}