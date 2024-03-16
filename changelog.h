#ifndef VERSION
#define VERSION "v9.1.0"

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

/* VERSION=v9.0.2
Fix:
    - Before:
        MotorHandler    transceiver1[] = { motor_handlers[0], motor_handlers[1], motor_handlers[2], };
        MotorHandler    transceiver2[] = { motor_handlers[3], motor_handlers[4], motor_handlers[5], };
    - After:
        MotorHandler    *transceiver1[] = { &motor_handlers[0], &motor_handlers[1], &motor_handlers[2], };
        MotorHandler    *transceiver2[] = { &motor_handlers[3], &motor_handlers[4], &motor_handlers[5], };
    -- Because: motor_handlers[$i] will be copied not refered.
*/

/* VERSION=v9.0.3
Change: CANHandler
    - After:
        class CANHandler {
        private:
            CAN can;
            MotorHandler *const *const motor_handlers_vec_arr;
            const std::size_t motor_handlers_vec_size;
            CANMessage rx_msg;
        public:
            template<std::size_t LEN>
            CANHandler(const PinName &rd, const PinName &td, MotorHandler *(*const motor_handlers_vec_arr_ptr)[LEN])
                : can(rd, td), motor_handlers_vec_arr(*motor_handlers_vec_arr_ptr), motor_handlers_vec_size(LEN), rx_msg(0)
            {
                rx_msg.len = 6;
            }
            void init(unsigned int id, unsigned int mask, void (*to_be_attached)(void));
            void read(CANMessage &rx_msg);
            void write(CANMessage &tx_msg);
            void onMsgReceived(void);
            void sendMsg(void);
        };
*/

/* VERSION=v9.0.4
Change: CANHandler::sendMsg
    - Before:
        void CANHandler::sendMsg()
        {
            for (std::size_t i = 0; i < motor_handlers_vec_size; i++) {
                motor_handlers_vec_arr[i]->pack(motor_handlers_vec_arr[i]->tx_msg);
                can.write(motor_handlers_vec_arr[i]->tx_msg);
            }
        }
    - After:
        void CANHandler::sendMsg()
        {
            for (std::size_t i = 0; i < motor_handlers_vec_size; i++) {
                can.write(motor_handlers_vec_arr[i]->tx_msg);
            }
        }
*/

#endif
