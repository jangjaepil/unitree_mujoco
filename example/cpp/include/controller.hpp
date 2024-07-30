#include <communication.hpp>
#include <MPC.hpp>
#include <WBIC.hpp>
#include <Requiredheaders.hpp>

class controller : public Comm,public MPC,public WBIC{
    public:
            controller();
            void run();
            void State2Vector(const unitree_go::msg::dds_::LowState_ low_state, const unitree_go::msg::dds_::SportModeState_ high_state);
            void getmodel(pinocchio::Model &model, pinocchio::Data &data);
            void setJacobian();
            void setDesireds();
    private:
        unsigned int dof = 0; //number of joints + 6
        int number_of_joints = 0;
        bool sensor_flag = 0;
        Eigen::VectorXd q;
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
        Eigen::VectorXd b;
        Eigen::VectorXd g;
        Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(15);

        Eigen::MatrixXd jacobian;
        Eigen::MatrixXd Jc;
        Eigen::MatrixXd Jcom;
        Eigen::MatrixXd Jbody;
        Eigen::MatrixXd JbodyOri;
        Eigen::MatrixXd Jpose;
        Eigen::MatrixXd Jlf;
        Eigen::MatrixXd Jrf;
        Eigen::MatrixXd Jf;
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
        
        double Total_mass = 0;
        Eigen::VectorXd MassPosition = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd CoM = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd CoMV = Eigen::VectorXd::Zero(3);

        int rSize = 0;
        Eigen::VectorXd motor_cmd;
        
};