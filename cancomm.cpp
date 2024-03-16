#include "capstone.hpp"

Motor::PutData decodeTx(const unsigned char (*const data_into_motor)[8])
{
    const unsigned char *const data = *data_into_motor;

    const int p_int  = (data[0] << 8) | (data[1]);
    const int v_int  = (data[2] << 4) | (data[3] >> 4);
    const int kp_int = ((data[3] & 0x0F) << 8) | (data[4]);
    const int kd_int = (data[5] << 4) | (data[6] >> 4);
    const int t_int  = ((data[6] & 0x0F) << 8) | (data[7]);

    Motor::PutData res = {
        .p    = int2float(p_int, P_MIN, P_MAX, 16),
        .v    = int2float(v_int, V_MIN, V_MAX, 12),
        .kp   = int2float(kp_int, KP_MIN, KP_MAX, 12),
        .kd   = int2float(kd_int, KD_MIN, KD_MAX, 12),
        .t_ff = int2float(t_int, T_MIN, T_MAX, 12),
    };

    return res;
}

UCh8 encodeTx(const Motor::PutData &data_into_motor)
{
    UCh8 msg = { .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, } };

    const float p  = middle(P_MIN, data_into_motor.p, P_MAX);
    const float v  = middle(V_MIN, data_into_motor.v, V_MAX);
    const float kp = middle(KP_MIN, data_into_motor.kp, KP_MAX);
    const float kd = middle(KD_MIN, data_into_motor.kd, KP_MAX);
    const float t  = middle(T_MIN, data_into_motor.t_ff, T_MAX);

    const int p_int  = float2int(p, P_MIN, P_MAX, 16);
    const int v_int  = float2int(v, V_MIN, V_MAX, 12);
    const int kp_int = float2int(kp, KP_MIN, KP_MAX, 12);
    const int kd_int = float2int(kd, KD_MIN, KD_MAX, 12);
    const int t_int  = float2int(t, T_MIN, T_MAX, 12);

    msg.data[0] = p_int >> 8;
    msg.data[1] = p_int & 0xFF;
    msg.data[2] = v_int >> 4;
    msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.data[4] = kp_int & 0xFF;
    msg.data[5] = kd_int >> 4;
    msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg.data[7] = t_int & 0xff;

    return msg;
}

GetDataWithId decodeRx(const unsigned char *const data)
{
    const int id    = data[0];
    const int p_int = (data[1]<<8)|data[2];
    const int v_int = (data[3]<<4)|(data[4]>>4);
    const int i_int = ((data[4]&0xF)<<8)|data[5];

    const float p = int2float(p_int, P_MIN, P_MAX, 16);
    const float v = int2float(v_int, V_MIN, V_MAX, 12);
    const float i = int2float(i_int, -I_MAX, I_MAX, 12);

    const GetDataWithId res = { .motor_id = id, .p = p, .v = v, .i = i, };

    return res;
}

void Motor::setInputWithHexademical(const UCh8 &encoded_input)
{
    this->data_into_motor = decodeTx(&encoded_input.data);
}

void Motor::pack(CANMessage &can_msg) const
{
    const UCh8 msg = encodeTx(data_into_motor);

    for (int i = 0; i < len(can_msg.data); i++) {
        can_msg.data[i] = msg.data[i];
    }
}

void Motor::unpack(const CANMessage &can_msg)
{
    const GetDataWithId data_from_motor = decodeRx(can_msg.data);

    if (this->motor_id == data_from_motor.motor_id) {
        this->data_from_motor.p = data_from_motor.p;
        this->data_from_motor.v = data_from_motor.v;
        this->data_from_motor.i = data_from_motor.i;
    }
}
