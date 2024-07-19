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
    private:
            int number_of_joints = 0;
            bool sensor_flag = 0;
            Eigen::VectorXd q;
            Eigen::VectorXd dq;
            Eigen::VectorXd tau;
            Eigen::Quaterniond imu_quat;
            Eigen::VectorXd imu_acc = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd imu_gyr = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd base_pos = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd base_vel = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd mpc_states = Eigen::VectorXd::Zero(15);
            Eigen::Matrix3d Rz = Eigen::MatrixXd::Identity(3,3);
            Eigen::VectorXd r1 = Eigen::VectorXd::Zero(3);
            Eigen::VectorXd r2 = Eigen::VectorXd::Zero(3);
            std::vector<Eigen::VectorXd> r;
            std::vector<Eigen::MatrixXd> alljacobian;
            Eigen::MatrixXd a;
            Eigen::MatrixXd b;
            int rSize = 0;
            unsigned int iteration = 0;
};