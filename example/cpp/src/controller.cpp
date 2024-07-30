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
    imu_quat.z() = low_state.imu_state().quaternion()[3];

    imu_gyr(0) = low_state.imu_state().gyroscope()[0];
    imu_gyr(1) = low_state.imu_state().gyroscope()[1];
    imu_gyr(2) = low_state.imu_state().gyroscope()[2];

    imu_acc(0) = low_state.imu_state().accelerometer()[0];
    imu_acc(1) = low_state.imu_state().accelerometer()[1];
    imu_acc(2) = low_state.imu_state().accelerometer()[2];

    base_pos(0) = high_state.position()[0];
    base_pos(1) = high_state.position()[1];
    base_pos(2) = high_state.position()[2];   
    
    base_vel(0) = high_state.velocity()[0];
    base_vel(1) = high_state.velocity()[1];
    base_vel(2) = high_state.velocity()[2];  
    
     //pinocchio
    //Joint ID 2: Name = left_hip_yaw_joint(7)
    //Joint ID 3: Name = left_hip_roll_joint(3)
    //Joint ID 4: Name = left_hip_pitch_joint(4)
    //Joint ID 5: Name = left_knee_joint(5)
    //Joint ID 6: Name = left_ankle_joint(10)
    //Joint ID 7: Name = right_hip_yaw_joint(8)
    //Joint ID 8: Name = right_hip_roll_joint(0)
    //Joint ID 9: Name = right_hip_pitch_joint(1)
    //Joint ID 10: Name = right_knee_joint(2)
    //Joint ID 11: Name = right_ankle_joint(11)
    //Joint ID 12: Name = torso_joint(6)
    //Joint ID 13: Name = left_shoulder_pitch_joint(16)
    //Joint ID 14: Name = left_shoulder_roll_joint(17)
    //Joint ID 15: Name = left_shoulder_yaw_joint(18)
    //Joint ID 16: Name = left_elbow_joint(19)
    //Joint ID 17: Name = right_shoulder_pitch_joint(12)
    //Joint ID 18: Name = right_shoulder_roll_joint(13)
    //Joint ID 19: Name = right_shoulder_yaw_joint(14)
    //Joint ID 20: Name = right_elbow_joint(15)
    
    //Sensor
    ////sensor_index: 0, name: right_hip_roll_pos, dim: 1                                      
    ////sensor_index: 1, name: right_hip_pitch_pos, dim: 1    
    ////sensor_index: 2, name: right_knee_pos, dim: 1
    ////sensor_index: 3, name: left_hip_roll_pos, dim: 1
    ////sensor_index: 4, name: left_hip_pitch_pos, dim: 1
    ////sensor_index: 5, name: left_knee_pos, dim: 1
    ////sensor_index: 6, name: torso_pos, dim: 1
    ////sensor_index: 7, name: left_hip_yaw_pos, dim: 1
    ////sensor_index: 8, name: right_hip_yaw_pos, dim: 1
    ////sensor_index: 9, name: not_use_pos, dim: 1
    ////sensor_index: 10, name: left_ankle_pos, dim: 1
    ////sensor_index: 11, name: right_ankle_pos, dim: 1
    //sensor_index: 12, name: right_shoulder_pitch_pos, dim: 1
    //sensor_index: 13, name: right_shoulder_roll_pos, dim: 1
    //sensor_index: 14, name: right_shoulder_yaw_pos, dim: 1
    //sensor_index: 15, name: right_elbow_pos, dim: 1
    //sensor_index: 16, name: left_shoulder_pitch_pos, dim: 1
    //sensor_index: 17, name: left_shoulder_roll_pos, dim: 1
    //sensor_index: 18, name: left_shoulder_yaw_pos, dim: 1
    //sensor_index: 19, name: left_elbow_pos, dim: 1


    Eigen::VectorXd base_ang_vel = Eigen::VectorXd::Zero(3);
     
    q(7)=low_state.motor_state()[7].q();
    q(8)=low_state.motor_state()[3].q();
    q(9)=low_state.motor_state()[4].q();
    q(10)=low_state.motor_state()[5].q();
    q(11)=low_state.motor_state()[10].q();
    q(12)=low_state.motor_state()[8].q();
    q(13)=low_state.motor_state()[0].q();
    q(14)=low_state.motor_state()[1].q();
    q(15)=low_state.motor_state()[2].q();
    q(16)=low_state.motor_state()[11].q();
    q(17)=low_state.motor_state()[6].q();
    q(18)=low_state.motor_state()[16].q();
    q(19)=low_state.motor_state()[17].q();
    q(20)=low_state.motor_state()[18].q();
    q(21)=low_state.motor_state()[19].q();
    q(22)=low_state.motor_state()[12].q();
    q(23)=low_state.motor_state()[13].q();
    q(24)=low_state.motor_state()[14].q();
    q(25)=low_state.motor_state()[15].q();
    
    dq(6)=low_state.motor_state()[7].dq();
    dq(7)=low_state.motor_state()[3].dq();
    dq(8)=low_state.motor_state()[4].dq();
    dq(9)=low_state.motor_state()[5].dq();
    dq(10)=low_state.motor_state()[10].dq();
    dq(11)=low_state.motor_state()[8].dq();
    dq(12)=low_state.motor_state()[0].dq();
    dq(13)=low_state.motor_state()[1].dq();
    dq(14)=low_state.motor_state()[2].dq();
    dq(15)=low_state.motor_state()[11].dq();
    dq(16)=low_state.motor_state()[6].dq();
    dq(17)=low_state.motor_state()[16].dq();
    dq(18)=low_state.motor_state()[17].dq();
    dq(19)=low_state.motor_state()[18].dq();
    dq(20)=low_state.motor_state()[19].dq();
    dq(21)=low_state.motor_state()[12].dq();
    dq(22)=low_state.motor_state()[13].dq();
    dq(23)=low_state.motor_state()[14].dq();
    dq(24)=low_state.motor_state()[15].dq();

    tau(0)=low_state.motor_state()[7].tau_est();
    tau(1)=low_state.motor_state()[3].tau_est();
    tau(2)=low_state.motor_state()[4].tau_est();
    tau(3)=low_state.motor_state()[5].tau_est();
    tau(4)=low_state.motor_state()[10].tau_est();
    tau(5)=low_state.motor_state()[8].tau_est();
    tau(6)=low_state.motor_state()[0].tau_est();
    tau(7)=low_state.motor_state()[1].tau_est();
    tau(8)=low_state.motor_state()[2].tau_est();
    tau(9)=low_state.motor_state()[11].tau_est();
    tau(10)=low_state.motor_state()[6].tau_est();
    tau(11)=low_state.motor_state()[16].tau_est();
    tau(12)=low_state.motor_state()[17].tau_est();
    tau(13)=low_state.motor_state()[18].tau_est();
    tau(14)=low_state.motor_state()[19].tau_est();
    tau(15)=low_state.motor_state()[12].tau_est();
    tau(16)=low_state.motor_state()[13].tau_est();
    tau(17)=low_state.motor_state()[14].tau_est();
    tau(18)=low_state.motor_state()[15].tau_est();
    
    q(0) = base_pos(0);
    q(1) = base_pos(1);
    q(2) = base_pos(2);
    q(3) = imu_quat.x(); // For pinocchio lib repersentation order
    q(4) = imu_quat.y();
    q(5) = imu_quat.z();
    q(6) = imu_quat.w();


    dq(0) = base_vel(0); //ref: body frame
    dq(1) = base_vel(1);
    dq(2) = base_vel(2);
    dq(3) = base_ang_vel(0); // ref: body frame
    dq(4) = base_ang_vel(1);
    dq(5) = base_ang_vel(2);
    
    euler_angle = imu_quat.toRotationMatrix().eulerAngles(0,1,2); //roll pitch yaw
    Eigen::Matrix3d wRb = imu_quat.toRotationMatrix();
    oriVel =  wRb*base_ang_vel; 
    
    Rz <<cos(euler_angle(2)), 0, -sin(euler_angle(2)),  0,
                        0, 1, 0, 0,
                        sin(euler_angle(2)), 0, cos(euler_angle(2)), 0,
                        0, 0, 0, 1;

    Rz = Eigen::Matrix3d::Identity(3,3);
    mpc_states<< euler_angle,base_pos, oriVel, wRb*base_vel,0,0,-9.8;//euler angle,position,angular velocity, linear velocity, gravity
    mpc_states<<0,0,0,0,0,1,0,0,0,0,0,0,0,0,-9.8*47;
    //std::cout<<"q: "<<q.transpose()<<std::endl;
        
    //std::cout<<"q: \n"<<q.transpose()<<std::endl;
    //std::cout<<"dq: \n"<<dq.transpose()<<std::endl;
    //std::cout<<"tau: \n"<<tau.transpose()<<std::endl;
    //std::cout<<"imu_quat: \n"<<imu_quat.x()<<"\n"<< imu_quat.y()<<"\n"<<imu_quat.z()<<"\n"<<imu_quat.w()<<std::endl;
    //std::cout<<"imu_gry: \n"<<imu_gyr.transpose()<<std::endl;
    //std::cout<<"imu_acc: \n"<<imu_acc.transpose()<<std::endl;
    
    //std::cout<<"high_state_pos: "<<high_state.position()[0]<<std::endl;
    //std::cout<<"base_pos: \n"<<base_pos.transpose()<<std::endl;
    //std::cout<<"base_vel: \n"<<base_vel.transpose()<<std::endl;
   

}

void controller::getmodel(pinocchio::Model &model, pinocchio::Data &data)
{
    
    pinocchio::computeAllTerms(model, data, q, dq);   
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);   
    a = data.M.triangularView<Eigen::Upper>();
    a.triangularView<Eigen::StrictlyLower>() = a.transpose();
    b = data.C*dq;
    g = data.g;

    pinocchio::centerOfMass(model,data,q,dq,false);
    CoM = data.com[0];
    CoMV = data.vcom[0];
    Jcom = Eigen::MatrixXd::Zero(6,dof);
    Jcom = pinocchio::jacobianCenterOfMass(model,data,false);
    
  
    //torso orientation, swing foot position, joint pose? 
    std::string frame_name[3];
    frame_name[0] = "torso_link";
    frame_name[1] = "right_ankle_link";
    frame_name[2] = "left_ankle_link";
    
    pinocchio::FrameIndex tcp_idx[3];
    jacobian = Eigen::MatrixXd::Zero(6,dof);
    Jbody = Eigen::MatrixXd::Zero(6,dof);
    Jrf = Eigen::MatrixXd::Zero(6,dof);
    Jlf = Eigen::MatrixXd::Zero(6,dof);
    Jpose = Eigen::MatrixXd::Identity(dof,dof);
    JbodyOri = Eigen::MatrixXd::Zero(3,dof);
    Jf = Eigen::MatrixXd::Zero(6,dof);
    Jf = Eigen::MatrixXd::Zero(6,dof);
    //contact positions for mpc
    Eigen::VectorXd rightF  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd leftF  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd frame_position = Eigen::VectorXd::Zero(3);
    int length = sizeof(frame_name)/sizeof(frame_name[0]);
    //std::cout<<"length: "<<length<<std::endl;

    for(int i = 0 ; i<length;i++)
    {   
        tcp_idx[i] = model.getFrameId(frame_name[i]);
        
        pinocchio::computeFrameJacobian(model, data, q,tcp_idx[i], pinocchio::ReferenceFrame::WORLD, jacobian);
        frame_position = data.oMf[tcp_idx[i]].translation();
        if(i==0) 
        {   
            Jbody = jacobian;
            JbodyOri = jacobian.block(3,0,3,dof);
        }
        if(i==1)
        {
            Jrf = jacobian;
            leftF = frame_position;
        }
        if(i==2)
        {
            Jlf = jacobian;
            rightF = frame_position;
        }
    }
    
    
    Jf.block(0,0,3,dof) = Jrf.block(0,0,3,dof);
    Jf.block(3,0,3,dof) = Jlf.block(0,0,3,dof);
    Jc = Jf;
    //Jp = Eigen::MatrixXd::Zero()
    std::cout<<"JbodyOri: "<<JbodyOri<<std::endl;
    std::cout<<"Jrf: "<<Jrf<<std::endl;
    std::cout<<"Jlf: "<<Jlf<<std::endl;
    std::cout<<"Jf: "<<Jf<<std::endl;
    
    alljacobian.clear();
    alljacobian.push_back(Jc);
    alljacobian.push_back(JbodyOri);
    alljacobian.push_back(Jcom);
    //alljacobian.push_back(Jf);
    
    
    r.clear();
    r.push_back(rightF);
    r.push_back(leftF);
    

    //Task current states
    
    allx.clear();
    allx_dot.clear();

    allx.push_back(euler_angle);
    allx_dot.push_back(oriVel);

    allx.push_back(CoM);
    allx_dot.push_back(CoMV);
    
    
    
    
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
    

    DesiredBodyPosition<<0,0,1;
    DesiredFootPositions<<0,-0.2,0,0,0.2,0;
    
    x_ref << DesiredBodyOrientation,DesiredBodyPosition,DesiredBodyOriVel,DesiredBodyVel,0,0,-9.8*47;

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

    Eigen::MatrixXd kp_temp = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd kd_temp = 0.1*Eigen::MatrixXd::Identity(3,3);
        
    for(int i = 0;i<alldesired_x.size();i++)
    {
    
        kp.push_back(kp_temp);
        kd.push_back(kd_temp);
    }
    
}

void controller::run()
{   
    std::string urdf_filename = "/home/jang/unitree_mujoco/example/cpp/robots/h1_description/urdf/h1.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

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
    unsigned int nt = 3; // body orientation task, CoM position task, swing foot positin task, joint position
    
   


    double delT = 0.04;
    double m = 47;
    double Fc = 1;
    Eigen::Matrix3d bI = 80*Eigen::MatrixXd::Identity(3,3);
 
    MPC mpc;
    WBIC wbic;
    
    while (1) //using threads? or other iterruptor to maintain the controll rate as delT
    {
        
        State2Vector(getLowState(),getHighState());
        Eigen::Matrix3d gI = Rz*bI*Rz.transpose();
        getmodel(model, data);
        setDesireds();


        OsqpEigen::Solver qp;
        mpc.init(mpc_states,x_ref,delT,Rz,gI,r,m,qp); 
        mpc.solveProblem(qp);
        Eigen::VectorXd Fr = mpc.getCtr();
        std::cout<<"Fr: "<<Fr.transpose()<<std::endl;
        qp.clearSolver();

        rSize = r.size();
        Eigen::MatrixXd q1 = 10*Eigen::MatrixXd::Identity(3*rSize,3*rSize);
        Eigen::MatrixXd q2 = 10*Eigen::MatrixXd::Identity(6,6);
        
        
        OsqpEigen::Solver QP;
    
        wbic.WBIC_setCartesianCommands(nt,alldesired_x, alldesired_x_dot, alldesired_x_2dot, allx, allx_dot, kp, kd);
        wbic.WBIC_Init(nt, dof,  q1, q2, Fr, alljacobian, a, b, g, Fc, q, dq ,QP);
        
        Eigen::MatrixXd  P  = Eigen::MatrixXd::Identity(dof - 6,dof - 6);
        Eigen::MatrixXd  D  = Eigen::MatrixXd::Identity(dof - 6,dof - 6);
        Eigen::VectorXd tau_cmd = wbic.WBIC_solve_problem(dof, Fr, P, D, QP);
        std::cout<<"tau_cmd: "<<tau_cmd.transpose()<<std::endl;
        QP.clearSolver();
        
       //transfer the tau_cmds to the real robot using communcation

    }
}