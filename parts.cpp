#include "capstone.hpp"

#if USE_PID
MotorHandler::MotorHandler(const int id, const float Kp, const float Ki, const float Kd)
    : p_ctrl(0.0), pid_for_p(Kp, Ki, Kd, &data_from_motor.p, &p_ctrl, &data_into_motor.p, P_MIN, P_MAX)
{
    tx_msg.len = 8;
    tx_msg.id = id;
    this->motor_id = id;
}
#else
MotorHandler::MotorHandler(const int id)
{
    tx_msg.len = 8;
    tx_msg.id = id;
    this->motor_id = id;
}
#endif

bool MotorHandler::isWellFormed() const
{
    return tx_msg.len == 8 && tx_msg.id == motor_id;
}

void MotorHandler::sendMsg()
{
    this->pack(this->tx_msg);
}

void MotorHandler::sendBin(const UCh8 &msg)
{
    for (std::size_t i = 0; i < len(tx_msg.data); i++) {
        tx_msg.data[i] = msg.data[i];
    }
}

int MotorHandler::id() const
{
    return tx_msg.id;
}

#if USE_PID
bool MotorHandler::pidInit()
{
    bool okay = pid_for_p.init();
    if (!okay) {
        printf("\rFailed to initialize the PID controller of the motor #%d...\n", motor_id);
    }
    return okay;
}

bool MotorHandler::pidCompute()
{
    bool okay = pid_for_p.compute();
    if (okay) {
        data_into_motor.p = p_ctrl;
    }
    return okay;
}

void MotorHandler::set_Kp(const float Kp)
{
    pid_for_p.Kp = Kp;
}

void MotorHandler::set_Ki(const float Ki)
{
    pid_for_p.Ki = Ki;
}

void MotorHandler::set_Kd(const float Kd)
{
    pid_for_p.Kd = Kd;
}
#endif

void CANHandler::init(const unsigned int id, const unsigned int mask, void (*const to_be_attached)(void))
{
    can.frequency(1000000);
    can.attach(to_be_attached);
    can.filter(id, mask, CANStandard, 0);
}

void CANHandler::onMsgReceived()
{
    can.read(rx_msg);
    for (std::size_t i = 0; i < motor_handlers_vec_size; i++) {
        motor_handlers_vec_ptr[i]->unpack(rx_msg);
    }
}

void CANHandler::sendMsg()
{
    for (std::size_t i = 0; i < motor_handlers_vec_size; i++) {
        motor_handlers_vec_ptr[i]->pack(motor_handlers_vec_ptr[i]->tx_msg);
        can.write(motor_handlers_vec_ptr[i]->tx_msg);
    }
}
