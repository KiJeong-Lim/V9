#include "capstone.hpp"

/************************************************< PID controller >****************************************************
* [Figure]                                                                                                            *
*                               +-------> [Kp * e(t)] ------------------+                                             *
*                               |                                       |                                             *
*                               |                                       v                                             *
*             +-----+ e(t)      |                                    +-(+)-+ u(t)                                     *
* r(t) ----> (+)    +-----------+-------> [Ki * int e(t)dt] ------> (+)    +-------------> [System] ------+----> y(t) *
*             +-(-)-+           |                                    +-(+)-+                              |           *
*                ^              |                                       ^                                 |           *
*                |              |                                       |                                 |           *
*                |              +-------> [Kd * de(t)/dt] --------------+                                 |           *
*                |                                                                                        |           *
*                +----------------------------------------------------------------------------------------+           *
* [Discription]                                                                                                       *
* PV = y(t): process variable                                                                                         *
* MV = u(t): manipulated variable                                                                                     *
* SP = r(t): setpoint                                                                                                 *
**********************************************************************************************************************/

/// PV = y(t): Process variable -- measured value;
/// MV = u(t): Manipulated variable -- PID output;
/// SP = r(t): Setpoint -- reference value.
PIDController::PIDController(PIDController::Real_t Kp, PIDController::Real_t Ki, PIDController::Real_t Kd, PIDController::Real_t *const PV, PIDController::Real_t *const MV, PIDController::Real_t *const SP, const PIDController::Real_t MV_MIN, const PIDController::Real_t MV_MAX)
    : last_time(0.0), last_error(0.0), error_sum(0.0), Kp(Kp), Ki(Ki), Kd(Kd), PV(PV), MV(MV), SP(SP), MV_MIN(MV_MIN), MV_MAX(MV_MAX)
{
}

/// Check whether the PID controller is well initialized.
bool PIDController::init()
{
    if (PV == NULL || MV == NULL || SP == NULL) {
        return false;
    }
    if (Kp < 0.0f || Ki < 0.0f || Kd < 0.0f) {
        return false;
    }
    if (MV_MIN >= MV_MAX) {
        return false;
    }

    last_time  = getTime();
    last_error = (*SP - *PV);
    error_sum  = 0.0f;

    return true;
}

/// Calculate u(t) with the PID algorithm and assign it to MV.
bool PIDController::compute()
{
    if (PV == NULL || MV == NULL || SP == NULL) {
        return false;
    }
    else {
        Real_t now     = getTime();
        Real_t dt      = (now - last_time);
        Real_t errval  = (*SP - *PV);
        Real_t derrval = (errval - last_error);

        error_sum += errval * dt;
        *MV = middle(MV_MIN, Kp * errval + Ki * error_sum + Kd * (derrval / dt), MV_MAX);

        last_time  = now;
        last_error = errval;

        return true;
    }
}

PIDController::Real_t PIDController::get_last_time() const
{
    return last_time;
}

PIDController::Real_t PIDController::get_last_error() const
{
    return last_error;
}

PIDController::Real_t PIDController::get_error_sum() const
{
    return error_sum;
}
