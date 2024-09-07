
#include <MPC.hpp>
#include <WBIC.hpp>
#include <Requiredheaders.hpp>
#include <mujoco/mujoco.h>
#define MOTOR_SENSOR_NUM 3
class controller
{
    public:
            controller(mjModel *model, mjData *data);
            void run();
            void State2Vector();
            void convertCmd(std::vector<Eigen::VectorXd>& Cmds);
            void getmodel();
            void setJacobian();
            void setDesireds();
    private:
        
        
        
        mjData *mj_data_;
        mjModel *mj_model_;
        int num_motor_ = 0;
        int dim_motor_sensor_ = 0;
        int have_imu_ = false;
        int have_frame_sensor_ = false;
        
        
        
        unsigned int dof = 0; //number of joints + 6
        int number_of_joints = 0;
       
        Eigen::VectorXd q;
        Eigen::VectorXd q_wbic;
        Eigen::VectorXd dq;
        Eigen::VectorXd tau;
        Eigen::Quaterniond imu_quat;
        Eigen::VectorXd imu_acc = Eigen::VectorXd::Zero(3); 
        Eigen::VectorXd imu_gyr = Eigen::VectorXd::Zero(3); 
        Eigen::VectorXd euler_angle = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd oriVel = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd base_pos = Eigen::VectorXd::Zero(3); 
        Eigen::VectorXd base_vel = Eigen::VectorXd::Zero(3); 
        Eigen::VectorXd mpc_states = Eigen::VectorXd::Zero(15);
        Eigen::Matrix3d Rz = Eigen::MatrixXd::Identity(3,3);
        Eigen::MatrixXd a;
        Eigen::MatrixXd past_a;
        Eigen::VectorXd b;
        Eigen::VectorXd g;
        Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(15);


        Eigen::VectorXd ee_OriVel = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd ee_pos = Eigen::VectorXd::Zero(3); 
        Eigen::VectorXd ee_vel = Eigen::VectorXd::Zero(3); 

        Eigen::MatrixXd jacobian_pos;
        Eigen::MatrixXd jacobian_rot;
        Eigen::MatrixXd Jc;
        Eigen::MatrixXd Jcom;
        Eigen::MatrixXd Jbody;
        Eigen::MatrixXd JbodyOri;
        Eigen::MatrixXd Jpose;
        Eigen::MatrixXd Jrl;
        Eigen::MatrixXd Jrr;
        Eigen::MatrixXd Jfl;
        Eigen::MatrixXd Jfr;
        Eigen::MatrixXd Jf;
         Eigen::MatrixXd Jee;
        Eigen::MatrixXd JeeOri; 
        
        //TASK gains
        std::vector<Eigen::MatrixXd> kp;
        std::vector<Eigen::MatrixXd> kd;
        
        std::vector<Eigen::VectorXd> r;
        std::vector<Eigen::MatrixXd> alljacobian;
        std::vector<Eigen::VectorXd> alldesired_x;
        std::vector<Eigen::VectorXd> alldesired_x_dot;
        std::vector<Eigen::VectorXd> alldesired_x_2dot;
        std::vector<Eigen::VectorXd> allx;
        std::vector<Eigen::VectorXd> allx_dot;
        Eigen::Matrix3d wRb;
        Eigen::Matrix3d wRe;
        //contact positions for mpc
        Eigen::VectorXd rl  = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd rr  = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd fl  = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd fr  = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd base = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd frame_position = Eigen::VectorXd::Zero(3);
        
        double m;
        double Total_mass = 0;
        Eigen::VectorXd MassPosition = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd CoM = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd CoMV = Eigen::VectorXd::Zero(3);

        int rSize = 0;
        Eigen::VectorXd motor_cmd;
        
        bool state_flag = 0;
        
};