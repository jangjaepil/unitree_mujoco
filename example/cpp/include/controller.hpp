#include<communication.hpp>
#include <Requiredheaders.hpp>

class controller : public Comm{
    public:
            controller();
            void run();
            void State2Vector(const unitree_go::msg::dds_::LowState_ low_state, const unitree_go::msg::dds_::SportModeState_ high_state);
            void getmodel(pinocchio::Model &model, pinocchio::Data &data);
    private:
            int number_of_joints = 0;
            Eigen::VectorXd q;
            Eigen::VectorXd dq;
            Eigen::VectorXd tau;
            Eigen::Quaterniond imu_quat;
            Eigen::VectorXd imu_acc = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd imu_gyr = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd base_pos = Eigen::VectorXd::Zero(3); 
            Eigen::VectorXd base_vel = Eigen::VectorXd::Zero(3); 
};