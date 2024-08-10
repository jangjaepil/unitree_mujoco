#include "communication.hpp"

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Comm::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Comm::LowStateMessageHandler, this, std::placeholders::_1), 1);

    highstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    highstate_subscriber->InitChannel(std::bind(&Comm::HighStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Comm::LowCmdWrite, this);
}

void Comm::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Comm::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    
}

void Comm::HighStateMessageHandler(const void *message)
{
    high_state = *(unitree_go::msg::dds_::SportModeState_ *)message;
   
}

unitree_go::msg::dds_::LowState_ Comm::getLowState()
{
    return this -> low_state;
}

unitree_go::msg::dds_::SportModeState_ Comm::getHighState()
{
    
    return this -> high_state;
}

void Comm::LowCmdWrite()
{   
    double kp = 0.5;
    double kd = 0.1;
    for (int i = 0; i < 12; i++)
    {
        
        low_cmd.motor_cmd()[i].q() = q(i);
        low_cmd.motor_cmd()[i].dq() = dq(i);
        low_cmd.motor_cmd()[i].kp() = kp;
        low_cmd.motor_cmd()[i].kd() = kd;
        low_cmd.motor_cmd()[i].tau() = tau(i);
    }
    
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

void Comm::setCmd(Eigen::VectorXd& Q, Eigen::VectorXd& dQ,Eigen::VectorXd& Tau)
{
    this -> q = Q;
    this -> dq = dQ;
    this -> tau = Tau;
}

