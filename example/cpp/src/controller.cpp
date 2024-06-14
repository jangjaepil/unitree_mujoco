#include <controller.hpp>


controller::controller()
{
    

    
    // wholeM<<mobile_mass, Z37,
    //        Z37.transpose(),M_j_;

    

 
    

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
    std::cout<<"q: "<<q.transpose()<<std::endl;
    Eigen::MatrixXd M_j_;
    Eigen::MatrixXd C_j_;
    Eigen::VectorXd G_j_;
    Eigen::VectorXd position;
    //std::cout<<"q  : "<<q.transpose()<<std::endl;
    pinocchio::computeAllTerms(model, data, q, dq);   
    M_j_ = data.M.triangularView<Eigen::Upper>();
    M_j_.triangularView<Eigen::StrictlyLower>() = M_j_.transpose();
    C_j_ = data.C;
    G_j_ = data.g;

    //std::cout<<"Inertia: "<<M_j_<<std::endl;
    //std::cout<<"Coriolli: "<<C_j_<<std::endl;
    //std::cout<<"Gravity: "<<G_j_.transpose()<<std::endl;
    int length = 5;
    std::string frame_name[length];
    frame_name[0] = "pelvis";
    frame_name[1] = "left_ankle_link";
    frame_name[2] = "right_ankle_link";
    frame_name[3] = "right_elbow_link";
    frame_name[4] = "left_elbow_link";
    pinocchio::FrameIndex tcp_idx[5];
        
    for(int i = 0;i<length;i++)
    {
        tcp_idx[i] = model.getFrameId(frame_name[i]);
    
        //Eigen::MatrixXd jacobian_tmp = Eigen::MatrixXd::Zero(6, model_pino_.nv);
        //Eigen::MatrixXd jacobian_dot_tmp = Eigen::MatrixXd::Zero(6, model_pino_.nv);
        //pinocchio::getFrameJacobian(model_pino_, data_pino, tcp_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_tmp);
        //pinocchio::getFrameJacobianTimeVariation(model_pino_, data_pino, tcp_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot_tmp);
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
    
        position = data.oMf[tcp_idx[i]].translation();
        std::cout<<" position " <<i<<": "<<position.transpose()<<std::endl;
        //j = jacobian_tmp;
        //j_dot = jacobian_dot_tmp;
        //jt = j.block(0,0,3,7);
        //jt_dot = j_dot.block(0,0,3,7);
        //vel = R_m*jt*q_dot;
        //twist = R_m_e*j*q_dot + mobile_twist;
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

    // Print the joint information
    std::cout << "Total number of joints (including virtual root and floating base joint): " << total_joints << std::endl;
    std::cout << "Number of physical joints: " << number_of_joints << std::endl;

    // Optionally, list all joints with their IDs and names
    std::cout << "List of joints (ID and Name):" << std::endl;
    for (int joint_id = 0; joint_id < total_joints; ++joint_id) {
        std::string joint_name = model.names[joint_id];
        std::cout << "Joint ID " << joint_id << ": Name = " << joint_name << std::endl;
    }
    
    q = Eigen::VectorXd::Zero(number_of_joints + 6+1);
    dq = Eigen::VectorXd::Zero(number_of_joints + 6);
    tau = Eigen::VectorXd::Zero(number_of_joints);
    
    while (1)
    {
        State2Vector(getLowState(),getHighState());
        getmodel(model, data);
        //sleep(1);
    }

}