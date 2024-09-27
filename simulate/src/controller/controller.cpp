#include <controller.hpp>
#include <thread>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>  // Everything needed to embed Python
#include <pybind11/numpy.h>

namespace py = pybind11;
controller::controller(mjModel *model, mjData *data) : mj_model_(model), mj_data_(data)
{
   
}

void controller::State2Vector()
{   
    
    
    dim_motor_sensor_ = MOTOR_SENSOR_NUM * (num_motor_+1);

    for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
    {
        const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        if (strcmp(name, "imu_quat") == 0)
        {
            have_imu_ = true;
        }
        if (strcmp(name, "frame_pos") == 0)
        {
            have_frame_sensor_ = true;
        }
    }


     if (mj_data_)
    {        
        // mani,uni,fl,fr,rl,rr   
        q(7)=mj_data_->sensordata[12];
        q(8)=mj_data_->sensordata[13];
        q(9)=mj_data_->sensordata[14];
        q(10)= mj_data_->sensordata[15]; 
        q(11)= mj_data_->sensordata[16]; 
        q(12)= mj_data_->sensordata[17]; 
        q(13)= mj_data_->sensordata[18]; 
        q(14)= mj_data_->sensordata[3]; 
        q(15)= mj_data_->sensordata[4]; 
        q(16)= mj_data_->sensordata[5]; 
        q(17)= mj_data_->sensordata[0]; 
        q(18)= mj_data_->sensordata[1]; 
        q(19)= mj_data_->sensordata[2]; 
        q(20)= mj_data_->sensordata[9];
        q(21)= mj_data_->sensordata[10];
        q(22)= mj_data_->sensordata[11];
        q(23)= mj_data_->sensordata[6];
        q(24)= mj_data_->sensordata[7];
        q(25)= mj_data_->sensordata[8];
        
       
        dq(6)=mj_data_->sensordata[12+num_motor_+1];
        dq(7)=mj_data_->sensordata[13+num_motor_+1];
        dq(8)=mj_data_->sensordata[14+num_motor_+1];
        dq(9)= mj_data_->sensordata[15+num_motor_+1]; 
        dq(10)= mj_data_->sensordata[16+num_motor_+1]; 
        dq(11)= mj_data_->sensordata[17+num_motor_+1]; 
        dq(12)= mj_data_->sensordata[18+num_motor_+1]; 
        dq(13)= mj_data_->sensordata[3+num_motor_+1]; 
        dq(14)= mj_data_->sensordata[4+num_motor_+1]; 
        dq(15)= mj_data_->sensordata[5+num_motor_+1]; 
        dq(16)= mj_data_->sensordata[0+num_motor_+1]; 
        dq(17)= mj_data_->sensordata[1+num_motor_+1];
        dq(18)= mj_data_->sensordata[2+num_motor_+1];
        dq(19)= mj_data_->sensordata[9+num_motor_+1];
        dq(20)= mj_data_->sensordata[10+num_motor_+1];
        dq(21)= mj_data_->sensordata[11+num_motor_+1];
        dq(22)= mj_data_->sensordata[6+num_motor_+1];
        dq(23)= mj_data_->sensordata[7+num_motor_+1];
        dq(24)= mj_data_->sensordata[8+num_motor_+1];
        

        
       

        if(have_imu_)
        {
            imu_quat.w()= mj_data_->sensordata[dim_motor_sensor_ + 0];
            imu_quat.x()= mj_data_->sensordata[dim_motor_sensor_ + 1];
            imu_quat.y()= mj_data_->sensordata[dim_motor_sensor_ + 2];
            imu_quat.z()= mj_data_->sensordata[dim_motor_sensor_ + 3];
        
            imu_gyr(0) = mj_data_->sensordata[dim_motor_sensor_ + 4];
            imu_gyr(1) = mj_data_->sensordata[dim_motor_sensor_ + 5];
            imu_gyr(2) = mj_data_->sensordata[dim_motor_sensor_ + 6];
        
            imu_acc(0) = mj_data_->sensordata[dim_motor_sensor_ + 7];
            imu_acc(1) = mj_data_->sensordata[dim_motor_sensor_ + 8];
            imu_acc(2) = mj_data_->sensordata[dim_motor_sensor_ + 9];
        }

        if(have_frame_sensor_)
        {
            base_pos(0) = mj_data_->sensordata[dim_motor_sensor_ + 10];
            base_pos(1) = mj_data_->sensordata[dim_motor_sensor_ + 11];
            base_pos(2) = mj_data_->sensordata[dim_motor_sensor_ + 12];

            base_vel(0) = mj_data_->sensordata[dim_motor_sensor_ + 13];
            base_vel(1) = mj_data_->sensordata[dim_motor_sensor_ + 14];
            base_vel(2) = mj_data_->sensordata[dim_motor_sensor_ + 15];
        }
    }
   
    
    
    Eigen::VectorXd base_ang_vel = imu_gyr;
    
    q(0) = base_pos(0);
    q(1) = base_pos(1);
    q(2) = base_pos(2);
    q(3) = imu_quat.x(); // For pinocchio lib repersentation order
    q(4) = imu_quat.y();
    q(5) = imu_quat.z();
    q(6) = imu_quat.w();

    wRb = imu_quat.toRotationMatrix();
    Eigen::VectorXd base_velB = wRb.transpose()*base_vel;

    dq(0) = base_vel(0); //ref: world frame
    dq(1) = base_vel(1);
    dq(2) = base_vel(2);
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
    mpc_states<< euler_angle,base_pos, oriVel, base_vel,0,0,-9.81*m;//euler angle,position,angular velocity, linear velocity, gravity
    
        
    std::cout<<"q: \n"<<q.transpose()<<std::endl;
    //std::cout<<"dq: \n"<<dq.transpose()<<std::endl;
    //std::cout<<"tau: \n"<<tau.transpose()<<std::endl;
    //std::cout<<"imu_quat: \n"<<imu_quat.x()<<"\n"<< imu_quat.y()<<"\n"<<imu_quat.z()<<"\n"<<imu_quat.w()<<std::endl;
    //std::cout<<"imu_gry: \n"<<imu_gyr.transpose()<<std::endl;
    //std::cout<<"imu_acc: \n"<<imu_acc.transpose()<<std::endl;
    
    //std::cout<<"base_pos: \n"<<base_pos.transpose()<<std::endl;
    //std::cout<<"base_vel: \n"<<base_vel.transpose()<<std::endl;
    
   

}

void controller::getmodel()
{
    int nv = mj_model_->nv;
    ////std::cout<<"nv: "<<nv<<std::endl;
    if(mj_data_)
    {
        g = Eigen::Map<Eigen::VectorXd>(mj_data_->qfrc_gravcomp, nv);

    
        b = Eigen::Map<Eigen::VectorXd>(mj_data_->qfrc_bias, nv);

        double* full_mass_matrix = new double[nv * nv];

        mj_fullM(mj_model_, full_mass_matrix, mj_data_->qM);
        if(full_mass_matrix[nv*nv-1]!=0)
        {
            a = Eigen::Map<Eigen::MatrixXd>(full_mass_matrix, nv, nv);
            past_a = a;
        }
        else
        {
            a = past_a;
        }
        
        delete[] full_mass_matrix; 
        //std::cout<< "a_POST : \n"<<a<<std::endl;
        // //std::cout<< "b : \n"<<b<<std::endl;
        // //std::cout<< "g : \n"<<g.transpose()<<std::endl;
  
        double jacp[3 * mj_model_->nv];  // 3D position Jacobian
        double jacr[3 * mj_model_->nv];  // 3D rotation Jacobian

        std::string frame_name[6];
        frame_name[0] = "base_link";
        frame_name[1] = "RL_foot";
        frame_name[2] = "RR_foot";
        frame_name[3] = "FL_foot";
        frame_name[4] = "FR_foot";
        frame_name[5] = "link06";
        

        int length = sizeof(frame_name)/sizeof(frame_name[0]);
    
        for(int i = 0 ; i<length;i++)
        {   
            int body_id = mj_name2id(mj_model_, mjOBJ_BODY, frame_name[i].c_str());
            ////std::cout<<"body id: "<<body_id<<std::endl;
            mj_jacBody(mj_model_, mj_data_, jacp, jacr, body_id);

            jacobian_pos = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic,Eigen::RowMajor>>(jacp, 3, nv);
            jacobian_rot = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic,Eigen::RowMajor>>(jacr, 3, nv);

            const mjtNum* pos = mj_data_->xpos + 3 * body_id;  
             
            frame_position = Eigen::Map<const Eigen::Vector3d>(pos);


            if(i==0) 
            {   

                Jbody = jacobian_pos;
                JbodyOri = jacobian_rot;
                base = frame_position;

            }
            if(i==1)
            {

                Jrl = jacobian_pos;
                rl = frame_position;
            }
            if(i==2)
            {
                rr = frame_position;

                Jrr = jacobian_pos;


            }
            if(i==3)
            {
                Jfl = jacobian_pos;
                fl = frame_position;
            }
            if(i==4)
            {
                Jfr = jacobian_pos;
                fr = frame_position;
            }
            if(i==5)
            {
                Jee = jacobian_pos;
                JeeOri = jacobian_rot;
                ee_pos = frame_position;
                ee_vel  = Jee*dq;
                ee_OriVel  = JeeOri*dq; 
                const mjtNum* rmatrix = mj_data_->xmat + 9* body_id;
               
                wRe = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rmatrix);

                 //std::cout<<"wRe: \n"<<wRe<<std::endl;
            
               }

        }
    

    Jc = Eigen::MatrixXd::Zero(12,nv);
    Jc.block(0,0,3,dof) = Jfl;
    Jc.block(3,0,3,dof) = Jfr;
    Jc.block(6,0,3,dof) = Jrl;
    Jc.block(9,0,3,dof) = Jrr;
    
    Eigen::MatrixXd Jcom(3, mj_model_->nv);
    Jcom.setZero();  // Initialize to zero
    Eigen::Vector3d com(0, 0, 0);
    double total_mass = 0.0;
    Eigen::MatrixXd J_pos(3, mj_model_->nv);
    J_pos.setZero();

    for (int i = 0; i < mj_model_->nbody; i++) 
    {
        double mass = mj_model_->body_mass[i];
        Eigen::Vector3d body_com(mj_data_->xipos[3 * i], mj_data_->xipos[3 * i + 1], mj_data_->xipos[3 * i + 2]);
        com += mass * body_com;
        
           
        mj_jacBodyCom(mj_model_, mj_data_, jacp, NULL, i);
        J_pos = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic,Eigen::RowMajor>>(jacp, 3, nv);
        
        //std::cout<<"jpos for com: \n"<<J_pos<<std::endl;
        Jcom += mass * J_pos;
        
        total_mass += mass;
    }

    com /= total_mass;  // Normalize by total mass to get the CoM
    Jcom /= total_mass;
    
    Eigen::VectorXd comVel = Jcom*dq;
    //std::cout<<"Jcom: \n"<<Jcom<<std::endl; 
    //std::cout<<"com: \n"<<com<<std::endl; 
    
    ////std::cout<<"JbodyOri: \n"<<JbodyOri<<std::endl;
    ////std::cout<<"Jbody: \n"<<Jbody<<std::endl;
    //std::cout<<"Jc: \n"<<Jc<<std::endl;
    ////std::cout<<"Jfl: \n"<<Jfl<<std::endl;
    ////std::cout<<"Jfr: \n"<<Jfr<<std::endl;
    ////std::cout<<"Jrl: \n"<<Jrl<<std::endl;
    ////std::cout<<"Jrr: \n"<<Jrr<<std::endl;
    //std::cout<<"Jee: \n"<<Jee<<std::endl;
    //std::cout<<"JeeOri: \n"<<JeeOri<<std::endl; 
    
    
    //std::cout<<"ee_pos: \n"<<ee_pos<<std::endl;
    // //std::cout<<"base_vel: \n"<<base_vel.transpose()<<std::endl;
    // //std::cout<<"base_vel calculated: \n"<<Jbody*dq<<std::endl;
    // //std::cout<<"base_orivel: \n"<<oriVel.transpose()<<std::endl;
    // //std::cout<<"base_vel calculated: \n"<<JbodyOri*dq<<std::endl;
        
    alljacobian.clear();
    alljacobian.push_back(Jc);
    alljacobian.push_back(JbodyOri);
    alljacobian.push_back(Jcom);
    //alljacobian.push_back(Jbody);
    alljacobian.push_back(JeeOri);
    alljacobian.push_back(Jee);
    
    // //std::cout<<"base: "<<base.transpose()<<std::endl;
    // //std::cout<<"rear left: "<<rl.transpose()<<std::endl;
    // //std::cout<<"rear right: "<<rr.transpose()<<std::endl;
    // //std::cout<<"front left: "<<fl.transpose()<<std::endl;
    // //std::cout<<"front right: "<<fr.transpose()<<std::endl;
    
    r.clear();
    r.push_back(fl);
    r.push_back(fr);
    r.push_back(rl);
    r.push_back(rr);
    

    //Task current states
    Eigen::VectorXd fakeOrientation = Eigen::VectorXd::Zero(3);
    allx.clear();
    allx_dot.clear();

    allx.push_back(fakeOrientation);
    allx.push_back(com);
    //allx.push_back(base_pos);
    allx.push_back(fakeOrientation);
    allx.push_back(ee_pos);
    
    
    allx_dot.push_back(oriVel);
    allx_dot.push_back(comVel);
    //allx_dot.push_back(base_vel);
    allx_dot.push_back(ee_OriVel);
    allx_dot.push_back(ee_vel);
    
   
    //////std::cout<<"base_pose(urdf): \n"<<base.transpose()<<std::endl;
    
    
   } 
}
void controller::setDesireds()
{
    Eigen::VectorXd DesiredBodyOrientation  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredComPosition  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyPosition  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEOrientation  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEPosition  = Eigen::VectorXd::Zero(3);
    
    Eigen::VectorXd DesiredBodyOriVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredComVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEOriVel  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEVel  = Eigen::VectorXd::Zero(3);
    
    Eigen::VectorXd DesiredBodyOriAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredComAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredBodyAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEOriAcc  = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd DesiredEEAcc  = Eigen::VectorXd::Zero(3);
    
    DesiredComPosition<<0,0,0.35;
    DesiredBodyPosition<<-0.06,0,0.35;
    DesiredBodyVel  << 0,0,0;
    
    double current_time = mj_data_-> time;
    double duration = 3;
    double y_position = 0.25*sin(3*3.14*current_time);
    DesiredEEPosition << 0.1,y_position,0.65;
    x_ref << DesiredBodyOrientation,DesiredBodyPosition,DesiredBodyOriVel,DesiredBodyVel,0,0,-9.81*m;
    

    Eigen::Matrix3d wRd = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d wRed = Eigen::MatrixXd::Identity(3,3);
    
    Eigen::Matrix3d bRd = wRb.transpose()*wRd;
    Eigen::Matrix3d eRed = wRe.transpose()*wRed;
    
    Eigen::AngleAxisd AngleAxis(bRd);
    
    Eigen::VectorXd axis = AngleAxis.axis();
    double radian = AngleAxis.angle();
    
    Eigen::AngleAxisd AngleAxis2(eRed);
    
    Eigen::VectorXd axis2 = AngleAxis2.axis();
    double radian2 = AngleAxis.angle();
    
    //std::cout<<"raidan: "<<radian<<std::endl;
    //std::cout<<"axis: "<<axis.transpose()<<std::endl;
    //std::cout<<"raidan2: "<<radian2<<std::endl;
    //std::cout<<"axis2: "<<axis2.transpose()<<std::endl;
    
    DesiredBodyOrientation  = radian * wRb * axis;
    DesiredEEOrientation  = radian2 * wRe * axis2;
    
    alldesired_x.clear();
    alldesired_x_dot.clear();
    alldesired_x_2dot.clear();

    alldesired_x.push_back(DesiredBodyOrientation);
    alldesired_x.push_back(DesiredComPosition);
    //alldesired_x.push_back(DesiredBodyPosition);
    alldesired_x.push_back(DesiredEEOrientation);
    alldesired_x.push_back(DesiredEEPosition);
    
    alldesired_x_dot.push_back(DesiredBodyOriVel);
    alldesired_x_dot.push_back(DesiredComVel);
    //alldesired_x_dot.push_back(DesiredBodyVel);
    alldesired_x_dot.push_back(DesiredEEOriVel);
    alldesired_x_dot.push_back(DesiredEEVel);
    
    alldesired_x_2dot.push_back(DesiredBodyOriAcc);
    alldesired_x_2dot.push_back(DesiredComAcc);
    //alldesired_x_2dot.push_back(DesiredBodyAcc);
    alldesired_x_2dot.push_back(DesiredEEOriAcc);
    alldesired_x_2dot.push_back(DesiredEEAcc);
    
    Eigen::MatrixXd kp_temp = 500*Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd kd_temp = 15*Eigen::MatrixXd::Identity(3,3);
    
    kp.clear();
    kd.clear();    
    
    for(int i = 0;i<alldesired_x.size();i++)
    {
    
        kp.push_back(kp_temp);
        kd.push_back(kd_temp);
    }
    
}
void controller::convertCmd(std::vector<Eigen::VectorXd>& Cmds)
{   
    // mani,uni,fl,fr,rl,rr
    Eigen::VectorXd q_cmd = Cmds[0];
    Eigen::VectorXd dq_cmd = Cmds[1];
    Eigen::VectorXd tau_cmd = Cmds[2];
    
        //fr,fl,rr,rl,mani,uni :sensor
        //fr,fl,rr,rl,mani
        Eigen::VectorXd Crt_q = Eigen::VectorXd::Zero(dof -7);
        Eigen::VectorXd Crt_dq = Eigen::VectorXd::Zero(dof -7);
        Eigen::VectorXd Crt_tau = Eigen::VectorXd::Zero(dof -7);
        
        Crt_q(12) = q_cmd(0);
        Crt_q(13) = q_cmd(1);
        Crt_q(14) = q_cmd(2);
        Crt_q(15) = q_cmd(3);
        Crt_q(16) = q_cmd(4);
        Crt_q(17) = q_cmd(5);
        
        Crt_q(3) = q_cmd(7);
        Crt_q(4) = q_cmd(8);
        Crt_q(5) = q_cmd(9);
        Crt_q(0) = q_cmd(10);
        Crt_q(1) = q_cmd(11);
        Crt_q(2) = q_cmd(12);
        Crt_q(9) = q_cmd(13);
        Crt_q(10) = q_cmd(14);
        Crt_q(11) = q_cmd(15);
        Crt_q(6) = q_cmd(16);
        Crt_q(7) = q_cmd(17);
        Crt_q(8) = q_cmd(18);

        
        Crt_dq(12) = dq_cmd(0);
        Crt_dq(13) = dq_cmd(1);
        Crt_dq(14) = dq_cmd(2);
        Crt_dq(15) = dq_cmd(3);
        Crt_dq(16) = dq_cmd(4);
        Crt_dq(17) = dq_cmd(5);
        Crt_dq(3) = dq_cmd(7);
        Crt_dq(4) = dq_cmd(8);
        Crt_dq(5) = dq_cmd(9);
        Crt_dq(0) = dq_cmd(10);
        Crt_dq(1) = dq_cmd(11);
        Crt_dq(2) = dq_cmd(12);
        Crt_dq(9) = dq_cmd(13);
        Crt_dq(10) = dq_cmd(14);
        Crt_dq(11) = dq_cmd(15);
        Crt_dq(6) = dq_cmd(16);
        Crt_dq(7) = dq_cmd(17);
        Crt_dq(8) = dq_cmd(18);

        Crt_tau(12) = tau_cmd(0);
        Crt_tau(13) = tau_cmd(1);
        Crt_tau(14) = tau_cmd(2);
        Crt_tau(15) = tau_cmd(3);
        Crt_tau(16) = tau_cmd(4);
        Crt_tau(17) = tau_cmd(5);
        Crt_tau(3) = tau_cmd(7);
        Crt_tau(4) = tau_cmd(8);
        Crt_tau(5) = tau_cmd(9);
        Crt_tau(0) = tau_cmd(10);
        Crt_tau(1) = tau_cmd(11);
        Crt_tau(2) = tau_cmd(12);
        Crt_tau(9) = tau_cmd(13);
        Crt_tau(10) = tau_cmd(14);
        Crt_tau(11) = tau_cmd(15);
        Crt_tau(6) = tau_cmd(16);
        Crt_tau(7) = tau_cmd(17);
        Crt_tau(8) = tau_cmd(18);
    
    //std::cout<<"Crt_tau: \n"<<Crt_tau.transpose()<<std::endl;
    //std::cout<<"Crt_q: \n"<<Crt_q.transpose()<<std::endl;
    //std::cout<<"Crt_dq: \n"<<Crt_dq.transpose()<<std::endl;

    double kp = 10;
    double kd = 6;
    if (mj_data_)
    {
        for (int i = 0; i < num_motor_; i++)
        {
           if(isnan(Crt_tau(i) +kp * (Crt_q(i) - mj_data_->sensordata[i]) + kd * (Crt_dq(i) - mj_data_->sensordata[i + num_motor_])))
            {
                while(1);
            }

            mj_data_->ctrl[i] = Crt_tau(i) +
                                kp * (Crt_q(i) - mj_data_->sensordata[i]) +
                                kd * (Crt_dq(i) - mj_data_->sensordata[i + num_motor_+1]);
        }
    }
        
       
        
}

void controller::run()
{   
    num_motor_ = mj_model_->nu;
    //std::cout<<"num_motor: "<<num_motor_<<std::endl;
    
    q = Eigen::VectorXd::Zero(mj_model_->nq);
    dq = Eigen::VectorXd::Zero(mj_model_->nv);
    tau = Eigen::VectorXd::Zero(mj_model_->nu);
    
    dof = mj_model_->nv;
    //std::cout<<"dof: "<<dof<<std::endl;
    unsigned int nt = 5; // contact constraint,body orientation,com position ,body position, ee ori, ee position 
    
    m = 0.0;
    for(size_t i = 0; i < mj_model_->nbody; ++i) {
        m += mj_model_->body_mass[i];
        std::cout<<i<<" th mass: "<<mj_model_->body_mass[i]<<std::endl;
    }
    
    //std::cout << "Total mass of the robot: " << m << " kg" << std::endl;
   

    double delT = 0.04;
    double Fc = 0.01;
    
    Eigen::Matrix3d bI = 40*Eigen::MatrixXd::Identity(3,3);
   
    py::scoped_interpreter guard{};
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")("/home/jang/unitree_mujoco/simulate/src/RL/");
    
    py::module_ rl_module = py::module_::import("Policy"); 

    MPC mpc;
    WBIC wbic;
  
    while (1) 
    {
        
            State2Vector();
            if(q.norm() ==  dq.norm() && dq.norm() == tau.norm() && tau.norm() == 0)
            {
                continue;
            }
       
            Eigen::Matrix3d gI = Rz*bI*Rz.transpose();
            getmodel();
            setDesireds();
            OsqpEigen::Solver qp;
            mpc.init(mpc_states,x_ref,delT,Rz,gI,r,m,Fc,qp); 
            mpc.solveProblem(qp);
            Eigen::VectorXd Fr =  mpc.getCtr();
            //std::cout<<"Fr: "<<Fr.transpose()<<std::endl;
            qp.clearSolver();

            rSize = r.size();
            Eigen::MatrixXd q1 = 10*Eigen::MatrixXd::Identity(3*rSize,3*rSize);
            Eigen::MatrixXd q2 = 10*Eigen::MatrixXd::Identity(3*rSize,3*rSize);


            OsqpEigen::Solver QP;
        
            std::vector<double> state = {0, 0, -0.4, 0.9};  // Example state
            std::cout<<"state done"<<std::endl;
            py::array_t<double> state_array(state.size(), state.data());

            // Call the Python function to get the control action
            py::object action = rl_module.attr("get_action")(state_array);
            
            
            // Convert the result back to C++ (vector of doubles)
            py::array_t<double> action_array = action.cast<py::array_t<double>>();
            py::buffer_info buf = action_array.request();

            double *ptr = static_cast<double *>(buf.ptr);
            size_t size = buf.size;
            
            // Convert NumPy array to std::vector<double>
            Eigen::Map<Eigen::VectorXd> eigen_action(ptr, size);

            // Print the action (or apply it to your MuJoCo controller)
            std::cout<<"control action: "<< eigen_action<<std::endl; 
            wbic.WBIC_setCartesianCommands(nt,alldesired_x, alldesired_x_dot, alldesired_x_2dot, allx, allx_dot, kp, kd);
            //std::cout<<"before wb init a: \n"<<a<<std::endl;
            wbic.WBIC_Init(nt, dof,  q1, q2, Fr, alljacobian, a, b, g, Fc, q_wbic, dq ,QP);

            wbic.WBIC_solve_problem(dof, Fr,QP); 
            std::vector<Eigen::VectorXd> cmd = wbic.WBIC_getCtr(); 
            convertCmd(cmd);
            QP.clearSolver();
  
    }
}



 