#ifndef CAPSTONE
#define CAPSTONE "CAPSTONE-V9"

#include <cstdio>
#include <cstring>
#include <cstddef>

#include "mbed.h"

#include "changelog.h"

#define USE_PID             true
#define RUNTIME_TICK_MAX    1000
#define Tick_dt             0.01

#define ESC                 (27)
#define LEFT_DIRECTION      (75)
#define RIGHT_DIRECTION     (77)
#define DEL_KEY             (83)
#define NOT_A_SPECIAL_KEY   (-1)

#define pi          (3.14159265359)

#define P_MIN       (-12.5f)
#define P_MAX       (12.5f)
#define V_MIN       (-45.0f)
#define V_MAX       (45.0f)
#define KP_MIN      (0.0f)
#define KP_MAX      (500.0f)
#define KD_MIN      (0.0f)
#define KD_MAX      (5.0f)
#define T_MIN       (-18.0f)
#define T_MAX       (18.0f)
#define I_MIN       (-40.0f)
#define I_MAX       (40.0f)

#define len(arr)    ((sizeof(arr)) / (sizeof((arr)[0])))
#define max(x,y)    (((x) >= (y)) ? (x) : (y))
#define min(x,y)    (((y) >= (x)) ? (x) : (y))

#ifndef USE_PID
#define USE_PID     0
#endif

struct UCh8 { unsigned char data[8]; };

struct GetDataWithId { int motor_id; float p; float v; float i; };

typedef enum { SetzeroMode = 0, RuntimeMode = 1, ObserveMode = 2, ReadcmdMode = 3, SitdownMode = 4, } Mode_t;

class Motor {
public:
    struct PutData { float p; float v; float kp; float kd; float t_ff; };
    struct GetData { float p; float v; float i; };
public:
    PutData data_into_motor;
    GetData data_from_motor;
    int motor_id;
public:
    void setInputWithHexademical(const UCh8 &encoded_input);
    void pack(CANMessage &tx_msg) const;
    void unpack(const CANMessage &rx_msg);
};

class PIDController {
public:
    typedef volatile float Real_t;
private:
    Real_t last_time;
    Real_t last_error;
    Real_t error_sum;
public:
    Real_t Kp;
    Real_t Ki;
    Real_t Kd;
    Real_t *PV;
    Real_t *MV;
    Real_t *SP;
    const Real_t MV_MIN;
    const Real_t MV_MAX;
public:
    PIDController(Real_t Kp, Real_t Ki, Real_t Kd, Real_t *PV, Real_t *MV, Real_t *SP, Real_t MV_MIN, Real_t MV_MAX);
    bool init(void);
    bool compute(void);
    Real_t get_last_time(void) const;
    Real_t get_last_error(void) const;
    Real_t get_error_sum(void) const;
};

class Gear {
public:
    int gear;
private:
    int gear_cnt;
public:
    Gear(int gear);
    Gear(const Gear &other);
    ~Gear();
    bool go(void);
    void reset(void);
};

class IO {
private:
    char buffer[64];
    int cursor;
    int theend;
    char *result;
    void (*prompt)(const char *msg);
public:
    void setPrompt(void (*prompt)(const char *msg));
    bool runPrompt(void);
    static int getc(void);
private:
    bool takech(int ch);
    void print(void) const;
    void clear(void);
    void sync(char *&msg) const;
};

class MotorHandler : public Motor {
public:
#if USE_PID
    PIDController::Real_t p_ctrl;
    PIDController pid_for_p;
    CANMessage *const tx_msg;
#else
    CANMessage *const tx_msg;
#endif
public:
#if USE_PID
    MotorHandler(CANMessage *const tx_msg, int id, float Kp, float Ki, float Kd);
    bool isWellFormed(void) const;
    void packTxMsg(void);
    int id(void) const;
    bool pidInit(void);
    bool pidCompute(void);
    void set_Kp(float Kp);
    void set_Ki(float Ki);
    void set_Kd(float Kd);
    void sendBin(const UCh8 &data);
#else
    MotorHandler(CANMessage *const tx_msg, int id);
    bool isWellFormed(void) const;
    void packTxMsg(void);
    int id(void) const;
    void sendBin(const UCh8 &msg);
#endif
};

class CANHandler {
public:
    CAN *const can;
    MotorHandler *const *const motor_handlers_vec_arr;
    const std::size_t motor_handlers_vec_size;
    CANMessage *const rx_msg;
public:
    template<std::size_t LEN>
    CANHandler(CAN *const can, MotorHandler *(*const motor_handlers_vec_arr_ptr)[LEN], CANMessage *const rx_msg)
        : can(can), motor_handlers_vec_arr(*motor_handlers_vec_arr_ptr), motor_handlers_vec_size(LEN), rx_msg(rx_msg)
    {
        rx_msg->len = 6;
    }
    void init(void (*to_be_attached)(void));
    void pullMsg(void);
    void pushMsg(void);
};

extern const Motor::PutData reftbl1[1000][3];
extern int                  special_key_flag;
extern IO                   terminal;
extern Timer                timer;
extern Ticker               send_can;
extern Serial               pc;
extern const char           *log_msg;

Motor::PutData              decodeTx(const unsigned char (*input_data)[8]);
UCh8                        encodeTx(const Motor::PutData &input_data);
GetDataWithId               decodeRx(const unsigned char *output_data);
void                        limitNorm(float &x, float &y, float limit);
int                         float2int(float x, float x_min, float x_max, int bits);
float                       int2float(int x_int, float x_min, float x_max, int bits);
float                       middle(float x, float y, float z);
double                      getTime(void);
bool                        areSameStr(const char *lhs, const char *rhs);
bool                        inRange(float left, float x, float right);
bool                        betweenEps(float x, float y, float epsilon);

#endif
