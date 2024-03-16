#ifndef VERSION
#define VERSION "v9.0.1"

/* VERSION=v9.0.1
Fix: compile error
    -- Details: file=main.cpp line=#70-#73
        #if USE_PID
        MotorHandler    transceiver1[] = { motor_handlers[0], motor_handlers[1], motor_handlers[2], };
        MotorHandler    transceiver2[] = { motor_handlers[3], motor_handlers[4], motor_handlers[5], };
        #else
Add: MotorHandler::sendBin
Remove: MotorHandler::putTxMsg
*/

#endif
