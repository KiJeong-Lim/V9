#include "capstone.hpp"

static void             printManual(void);

static void             onMsgReceived1(void);
static void             onMsgReceived2(void);
static void             transmitMsg(void);

static void             start(void);
static void             halt(void);
static void             observe(void);
static void             overwatch(void);
static void             guard(void);

static bool             loadRefTbl1(bool until);

static void             jump(void);
static void             standUp(void);
#if USE_PID
static void             jump1(void);
static void             standUp1(void);
#endif
static void             jump2(void);
static void             standUp2(void);

static void             serial_isr(void);

static void             interact(void);
static void             prompt(const char *msg);

#if USE_PID
static void             pidInit(void);
static void             pidCompute(void);
#endif
static Motor::PutData   sitDown_calc(int count_down, const Motor::PutData &datum);

static void             printAll(void);

IO                      terminal;
Timer                   timer;
Ticker                  send_can;
Serial                  pc(PA_2, PA_3);

static bool             debug               = false;
static Mode_t           mode                = SetzeroMode;
static long int         turn_cnt            = -2;
void                    (*operation)(void)  = jump2;
static const int        count_down_MAX_CNT  = -100;
#if USE_PID
static long int         PID_START_TICK      = 390;
#endif

MotorHandler motor_handlers[] = {
#if USE_PID
    //           #  Kp    Ki    Kd
    MotorHandler(1, 1.30, 0.10, 0.00),
    MotorHandler(2, 1.25, 0.30, 0.00),
    MotorHandler(3, 2.00, 1.00, 0.00),
    MotorHandler(4, 1.30, 0.10, 0.00),
    MotorHandler(5, 1.25, 0.30, 0.00),
    MotorHandler(6, 2.00, 1.00, 0.00),
#else
    //           #
    MotorHandler(1),
    MotorHandler(2),
    MotorHandler(3),
    MotorHandler(4),
    MotorHandler(5),
    MotorHandler(6),
#endif
};

#if USE_PID
MotorHandler    *transceiver1[] = { &motor_handlers[0], &motor_handlers[1], &motor_handlers[2], };
MotorHandler    *transceiver2[] = { &motor_handlers[3], &motor_handlers[4], &motor_handlers[5], };
#else
MotorHandler    *transceiver1[] = { &motor_handlers[0], &motor_handlers[1], &motor_handlers[2], };
MotorHandler    *transceiver2[] = { &motor_handlers[3], &motor_handlers[4], &motor_handlers[5], };
#endif

CANHandler      cans[] = { CANHandler(PB_8, PB_9, &transceiver1), CANHandler(PB_5, PB_6, &transceiver2), };
void            (*const onMsgReceived[])(void) = { onMsgReceived1, onMsgReceived2, };

int main(void)
{
    pc.baud(921600);
    pc.attach(interact);

    for (std::size_t i = 0; i < len(cans); i++) {
        cans[i].init(0x01 << 21, 0xFFE00004, onMsgReceived[i]);
    }

    printf("\n\r<< %s >>\n", CAPSTONE);
    printf("\rVERSION = %s\n", VERSION);
#if USE_PID
    printf("\rUSE_PID = true\n");
#else
    printf("\rUSE_PID = false\n");
#endif
    printf("\rRUNTIME_TICK_MAX = %d\n", RUNTIME_TICK_MAX);
    printf("\rTick_dt = %lf[s]\n", Tick_dt);
    printf("\r\n");

    printManual();

    terminal.setPrompt(prompt);
    timer.start();
    send_can.attach(serial_isr, Tick_dt);
    halt();
    transmitMsg();
}

void printManual()
{
    printf(
        "\r[[MANUAL]]\n"
        "\r[#1 Tera term setting]\n"
        "\r  SERIAL.BAUDRATE    = 921600\n"
        "\r  TERMINAL.NEWLINE.R = LF\n"
        "\r  TERMINAL.NEWLINE.M = LF\n"
        "\r[#2 Key and action]\n"
        "\r  r                  = Start operation\n"
        "\r  b                  = Put p=0,v=0,kp=0,kd=0,t_ff=0\n"
        "\r  o                  = Print angles and angular velocities\n"
        "\r  (Esc)              = Let all motors exit motor mode\n"
        "\r  m                  = Let all motors enter motor mode\n"
        "\r  1                  = Let 1st motor rest\n"
        "\r  2                  = Let 2nd motor rest\n"
        "\r  3                  = Let 3rd motor rest\n"
        "\r  4                  = Let 4th motor rest\n"
        "\r  5                  = Let 5th motor rest\n"
        "\r  6                  = Let 6th motor rest\n"
        "\r  l                  = Listen command\n"
        "\r  (Space bar)        = Emergency Stop\n"
        "\r  (Back space)       = Turn debugger off\n"
        "\r  z                  = Set zero\n"
        "\r  .                  = Decrease p,v,kp,kd,t_ff -> 0\n"
        "\r[#3 Command]\n"
        "\r  (Esc)              = Quit listening\n"
        "\r  debug              = Turn debugger on\n"
#if USE_PID
        "\r  pid start = $int   = Start pid at $int-th tick\n"
        "\r  print all          = print values of all variables\n"
        "\r  Kp $int = $float   = Set Kp of $int-th motor to $float\n"
        "\r  Ki $int = $float   = Set Ki of $int-th motor to $float\n"
        "\r  Kd $int = $float   = Set Kd of $int-th motor to $float\n"
#endif
        "\r  help               = Print manual\n"
        "\r  $string            = Let operation be $string\n"
    );
    printf("\r\n");
}

void onMsgReceived1()
{
    cans[0].onMsgReceived();
}

void onMsgReceived2()
{
    cans[1].onMsgReceived();
}

void transmitMsg()
{
    for (std::size_t i = 0; i < len(cans); i++) {
        cans[i].pushMsg();
    }
}

void start()
{
    turn_cnt = 0;
}

void halt()
{
    const Motor::PutData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor = zero_data;
    }
    mode = SetzeroMode;
    turn_cnt = -2;
}

void observe()
{
    static long int row = 0;
    static Gear gear_obs = Gear(20);

    if (gear_obs.go()) {
        if (turn_cnt >= 0) {
            row++;
        }
        else {
            row = 0;
        }
        for (std::size_t i = 0; i < len(motor_handlers); i++) {
            const Motor::GetData data = motor_handlers[i].data_from_motor; // SENSITIVE POINT
            const int id = motor_handlers[i].motor_id;
            printf("\rtheta%d(%ld) = %f; omega%d(%ld) = %f;\n", id, row, data.p, id, row, data.v);
        }
        printf("\n");
    }
}

void guard()
{
    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        const Motor::GetData data = motor_handlers[i].data_from_motor; // SENSITIVE POINT
        const int id = motor_handlers[i].motor_id;

        const float p = middle(-0.2, motor_handlers[i].data_into_motor.p, 0.2);
        const float v = middle(-0.05, motor_handlers[i].data_into_motor.v, 0.05);
        const float kp = middle(-0.5, motor_handlers[i].data_into_motor.kp, 20.0);
        const float kd = middle(-0.05, motor_handlers[i].data_into_motor.kd, 4.0);
        const float t_ff = middle(-0.05, motor_handlers[i].data_into_motor.t_ff, 0.05);

        if (!betweenEps(motor_handlers[i].data_into_motor.p, p, 0.001)) {
            motor_handlers[i].data_into_motor.p = p;
            printf("\r%%guard:p(%d)=%lf\n", id, p);
        }
        if (!betweenEps(motor_handlers[i].data_into_motor.v, v, 0.001)) {
            motor_handlers[i].data_into_motor.v = v;
            printf("\r%%guard:v(%d)=%lf\n", id, v);
        }
        if (!betweenEps(motor_handlers[i].data_into_motor.kp, kp, 0.001)) {
            motor_handlers[i].data_into_motor.kp = kp;
            printf("\r%%guard:kp(%d)=%lf\n", id, kp);
        }
        if (!betweenEps(motor_handlers[i].data_into_motor.kd, kd, 0.001)) {
            motor_handlers[i].data_into_motor.kd = kd;
            printf("\r%%guard:kd(%d)=%lf\n", id, kd);
        }
        if (!betweenEps(motor_handlers[i].data_into_motor.t_ff, t_ff, 0.001)) {
            motor_handlers[i].data_into_motor.t_ff = t_ff;
            printf("\r%%guard:t_ff(%d)=%lf\n", id, t_ff);
        }
    }
    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].packTxMsg();
    }
}

void overwatch()
{
    static Gear gear_dbg = Gear(20);

    if (gear_dbg.go()) {
        if (turn_cnt < 0) {
            printf("\t1\t2\t3\t4\t5\t6\t7\t8\n");
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                printf("#%d\t", motor_handlers[i].id());
                for (std::size_t j = 0; j < len(motor_handlers[i].tx_msg.data); j++) {
                    printf("%X\t", motor_handlers[i].tx_msg.data[j]);
                }
                printf("\n");
            }
        }
        else {
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const Motor::PutData data = decodeTx(&motor_handlers[i].tx_msg.data);
                printf("\n\r%%motor#%d = { .p=%.2lf, .v=%.2lf, .kp=%.2lf, .kd=%.2lf, .t_ff=%.2lf }\n", motor_handlers[i].motor_id, data.p, data.v, data.kp, data.kd, data.t_ff);
            }
        }
        printf("\n");
    }
}

bool loadRefTbl1(const bool until)
{
    static Motor::PutData last_data[len(motor_handlers)];

    if ((turn_cnt < len(reftbl1)) && until) {
        for (std::size_t i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].data_into_motor = reftbl1[turn_cnt][(motor_handlers[i].id() - 1) % 3]; // SENSITIVE POINT
            last_data[i] = reftbl1[turn_cnt][(motor_handlers[i].id() - 1) % 3]; // SENSITIVE POINT
        }
        return true;
    }
    else {
        for (std::size_t i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].data_into_motor = last_data[i];
        }
        return false;
    }
}

void jump()
{
    loadRefTbl1(turn_cnt < len(reftbl1));
}

void standUp()
{
    const unsigned char lines[3][8] = {
        { 0x7C, 0xA5, 0x96, 0xB0, 0x7A, 0x99, 0x97, 0xFF, },
        { 0x7C, 0xED, 0x96, 0xB0, 0x7A, 0x99, 0x95, 0xC6, },
        { 0x7F, 0xBA, 0x7F, 0xF0, 0x39, 0x00, 0x07, 0x8D, },
    };

    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor = decodeTx(&lines[(motor_handlers[i].id() - 1) % 3]); // SENSITIVE POINT
    }
}

#if USE_PID
void jump1()
{
    loadRefTbl1(turn_cnt <= PID_START_TICK);
    if (turn_cnt == PID_START_TICK) {
        pidInit();
    }
    else if (turn_cnt > PID_START_TICK) {
        pidCompute();
    }
}

void standUp1()
{
    if (turn_cnt < PID_START_TICK) {
        standUp();
        return;
    }
    if (turn_cnt == PID_START_TICK) {
        standUp();
        pidInit();
        return;
    }
    if (turn_cnt > PID_START_TICK) {
        pidCompute();
        return;
    }
}
#endif

void jump2()
{
    loadRefTbl1(turn_cnt < 199);

    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor.p *= -1.0f;
    }
}

void standUp2()
{
    standUp();

    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor.p *= 0.25f;
        motor_handlers[i].data_into_motor.v *= 0.25f;
        motor_handlers[i].data_into_motor.kp *= 0.25f;
        motor_handlers[i].data_into_motor.kd *= 0.25f;
        motor_handlers[i].data_into_motor.t_ff *= 0.25f;
    }
}

void serial_isr()
{
    switch (mode) {
    case RuntimeMode:
        if (turn_cnt > RUNTIME_TICK_MAX) {
            turn_cnt = -2;
            halt();
            break;
        }
        if (turn_cnt >= 0) {
            operation();
            observe();
            turn_cnt++;
        }
        for (std::size_t i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].packTxMsg();
        }
        break;
    case ObserveMode:
        turn_cnt = -2;
        observe();
        break;
    case ReadcmdMode:
        turn_cnt = -2;
        break;
    case SetzeroMode:
        turn_cnt = -2;
        halt();
        break;
    case SitdownMode:
        if (turn_cnt >= -2) {
            turn_cnt = -2;
            halt();
        }
        else if (turn_cnt >= count_down_MAX_CNT) {
            observe();
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                motor_handlers[i].data_into_motor = sitDown_calc(-turn_cnt, motor_handlers[i].data_into_motor);
                motor_handlers[i].packTxMsg();
            }
            turn_cnt++;
        }
        break;
    default:
        printf("\n\r%% Undefined mode %%\n");
        mode = SetzeroMode;
        turn_cnt = -2;
        halt();
        break;
    }

    if (debug) {
        overwatch();
    }
    guard();
    transmitMsg();
}

void interact()
{
    int ch = 0, k = -1;

    if (mode == ReadcmdMode) {
        const bool prompt_routine_breaked = terminal.runPrompt();
        if (prompt_routine_breaked) {
            mode = SetzeroMode;
        }
        turn_cnt = -2;
        return;
    }

    ch = IO::getc();

    if (special_key_flag == NOT_A_SPECIAL_KEY) {
        switch (ch) {
        case '\b':
            debug = false;
            return;
        case ESC:
            printf("\n\r%% Exiting motor mode %%\n");
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, } };
                motor_handlers[i].sendBin(msg);
            }
            halt();
            transmitMsg();
            turn_cnt = -2;
            return;
        case 'm':
            printf("\n\r%% Entering motor mode %%\n");
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, } };
                motor_handlers[i].sendBin(msg);
            }
#if 0
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF, } };
                printf("\n\r%% Motor #%d rest position %%\n", motor_handlers[i].id());
                motor_handlers[i].sendBin(msg);
            }
#endif
            turn_cnt = -2;
            return;
        case 'z':
            printf("\n\r%% Set zero %%\n");
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, } };
                motor_handlers[i].sendBin(msg);
            }
            turn_cnt = -2;
            return;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
            for (int i = 0; i < len(motor_handlers); i++) {
                if ("123456"[i] == ch) {
                    k = i;
                    break;
                }
            }
            if (k >= 0) {
                const UCh8 msg = { .data = { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF, } };
                printf("\n\r%% Motor #%c rest position %%\n", ch);
                motor_handlers[k].sendBin(msg);
            }
            k = -1;
            return;
        case 'r':
            printf("\n\r%% Run %%\n");
            mode = RuntimeMode;
            start();
            return;
        case 'o':
            printf("\n\r%% Observe %%\n");
            mode = ObserveMode;
            turn_cnt = -2;
            return;
        case 'b':
            printf("\n\r%% Break %%\n");
            mode = SetzeroMode;
            turn_cnt = -2;
            halt();
            return;
        case ' ':
            halt();
            transmitMsg();
            for (std::size_t i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, } };
                motor_handlers[i].sendBin(msg);
            }
            transmitMsg();
            printf("\n\r%% Abort %%\n");
            mode = SetzeroMode;
            turn_cnt = -2;
            return;
        case '.':
            printf("\n\r%% Sit down %%\n");
            mode = SitdownMode;
            turn_cnt = count_down_MAX_CNT;
            return;
        case 'l':
            if (mode == SetzeroMode && turn_cnt < 0) {
                printf("\n\r%% Listen %%\n");
                mode = ReadcmdMode;
                turn_cnt = -2;
                debug = false;
            }
            return;
        }
    }
    else {
        return;
    }
}

void prompt(const char *const msg)
{
    int sscanf_res = 0;
    bool res = false;
    char str1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
    int n1 = 0;
    float x1 = 0.0;

    if (msg == NULL) {
        printf("\n\r%% Leaving listening mode %%\n");
        mode = SetzeroMode;
        return;
    }

#if USE_PID
    sscanf_res = sscanf(msg, "%s %d = %f", str1, &n1, &x1);
    if (sscanf_res == 3) {
        int idx = -1;
        for (int i = 0; i < len(motor_handlers); i++) {
            if (motor_handlers[i].id() == n1) {
                idx = i;
                break;
            }
        }
        if (idx < 0) {
            res = false;
            goto RET;
        }
        else if (areSameStr("Kp", str1)) {
            motor_handlers[idx].set_Kp(x1); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else if (areSameStr("Ki", str1)) {
            motor_handlers[idx].set_Ki(x1); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else if (areSameStr("Kd", str1)) {
            motor_handlers[idx].set_Kd(x1); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }

    sscanf_res = sscanf(msg, "pid start = %d", &n1);
    if (sscanf_res == 1) {
        if (PID_START_TICK >= 0) {
            PID_START_TICK = n1;
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }
#endif

    sscanf_res = sscanf(msg, "print %s", str1);
    if (sscanf_res == 1) {
        if (areSameStr(str1, "all")) {
            printAll();
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }

    sscanf_res = sscanf(msg, "%s", str1);
    if (sscanf_res == 1) {
        if (areSameStr(str1, "help")) {
            printManual();
        }
        else if (areSameStr(str1, "debug")) {
            if (debug) {
                debug = false;
            }
            else {
                debug = true;
            }
            res = true;
            goto RET;
        }
        else if (areSameStr(str1, "jump")) {
            operation = jump;
            res = true;
            goto RET;
        }
        else if (areSameStr(str1, "standUp")) {
            operation = standUp;
            res = true;
            goto RET;
        }
#if USE_PID
        else if (areSameStr(str1, "jump1")) {
            operation = jump1;
            res = true;
            goto RET;
        }
        else if (areSameStr(str1, "standUp1")) {
            operation = standUp1;
            res = true;
            goto RET;
        }
#endif
        else if (areSameStr(str1, "jump2")) {
            operation = jump2;
            res = true;
            goto RET;
        }
        else if (areSameStr(str1, "standUp2")) {
            operation = standUp2;
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }

RET:
    if (res) {
        printf("\n\rCommand well recieved\n");
    }
    else {
        printf("\n\rUnknown command or wrong command\n");
    }
}

#if USE_PID
void pidInit()
{
    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].pidInit();
    }
}

void pidCompute()
{
    for (std::size_t i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].pidCompute();
    }
}
#endif

Motor::PutData sitDown_calc(const int count_down, const Motor::PutData &datum)
{
    const Motor::PutData res = {
        .p    = (datum.p    * abs(count_down)) / abs(count_down_MAX_CNT),
        .v    = (datum.v    * abs(count_down)) / abs(count_down_MAX_CNT),
        .kp   = (datum.kp   * abs(count_down)) / abs(count_down_MAX_CNT),
        .kd   = (datum.kd   * abs(count_down)) / abs(count_down_MAX_CNT),
        .t_ff = (datum.t_ff * abs(count_down)) / abs(count_down_MAX_CNT),
    };

    return res;
}

void printAll()
{
    printf("\n\rTick_dt = %lf\n", Tick_dt);
#if USE_PID
    printf("\n\rPID_START_TICK = %ld\n", PID_START_TICK);
#endif
    if (operation == standUp) {
        printf("\n\roperation = standUp\n");
    }
    if (operation == jump) {
        printf("\n\roperation = jump\n");
    }
#if USE_PID
    if (operation == standUp1) {
        printf("\n\roperation = standUp1\n");
    }
    if (operation == jump1) {
        printf("\n\roperation = jump1\n");
    }
#endif
    if (operation == standUp2) {
        printf("\n\roperation = standUp2\n");
    }
    if (operation == jump2) {
        printf("\n\roperation = jump2\n");
    }
}
