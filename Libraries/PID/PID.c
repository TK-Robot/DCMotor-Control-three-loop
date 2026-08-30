#include "PID.h"
#include "FixedPointMath.h"
#include <limits.h>
#include <stdlib.h>

#define CURRENT_FEEDBACK_ARM_UPDATES 2L
#define SPEED_BREAKAWAY_KI_MULTIPLIER 4U
#define SPEED_BREAKAWAY_TARGET_MULTIPLIER 3L

void PID_Init(Param *param)
{
    param->Pid_Pos.Kp = 10500;
    param->Pid_Pos.Ki = 0;
    param->Pid_Pos.Kd = 13;
    param->Pid_Pos.out_min = 200;
    param->Pid_Pos.out_max = 30000;
    param->Pid_Pos.integral_max = 35000;

    param->Pid_PosVel.Kp = 80;
    param->Pid_PosVel.Ki = 5;
    param->Pid_PosVel.Kd = 0;
    param->Pid_PosVel.out_min = 0;
    param->Pid_PosVel.out_max = 400;
    param->Pid_PosVel.integral_max = 8000;

    param->Pid_PosEle.Kp = 250;
    param->Pid_PosEle.Ki = 30;
    param->Pid_PosEle.Kd = 0;
    param->Pid_PosEle.out_min = 0;
    param->Pid_PosEle.out_max = 500;
    param->Pid_PosEle.integral_max = 16000;

}

void PID_MigrateLegacyCurrentTuning(Param *param)
{
    if (param->Pid_Pos.out_max > 30000U)
    {
        param->Pid_Pos.out_max = 30000U;
    }
    if (param->Pid_Pos.Kd > POSITION_PID_KD_MAX)
    {
        param->Pid_Pos.Kd = 13U;
    }
    if (param->Pid_PosVel.Kp == 20U &&
        param->Pid_PosVel.Ki == 0U &&
        param->Pid_PosVel.Kd == 0U)
    {
        param->Pid_PosVel.Kp = 80U;
        param->Pid_PosVel.Ki = 5U;
        PID_Reset(&param->Pid_PosVel);
    }
    if (param->Pid_PosEle.Kp == 250U &&
        param->Pid_PosEle.Ki == 30U &&
        param->Pid_PosEle.Kd == 0U &&
        param->Pid_PosEle.integral_max == 8000 &&
        param->Pid_PosEle.out_max == 500U &&
        param->Pid_PosEle.out_min == 0U)
    {
        param->Pid_PosEle.integral_max = 16000;
    }
    if (param->Pid_PosEle.Kp > CURRENT_PID_KP_MAX ||
        param->Pid_PosEle.Ki > CURRENT_PID_KI_MAX ||
        param->Pid_PosEle.Kd != 0U ||
        param->Pid_PosEle.integral_max < 0 ||
        param->Pid_PosEle.integral_max > CURRENT_PID_INTEGRAL_MAX ||
        param->Pid_PosEle.out_max > CURRENT_PID_VOLTAGE_MAX_MV ||
        param->Pid_PosEle.out_min > param->Pid_PosEle.out_max)
    {
        param->Pid_PosEle.Kp = 250U;
        param->Pid_PosEle.Ki = 30U;
        param->Pid_PosEle.Kd = 0U;
        param->Pid_PosEle.integral_max = 16000;
        param->Pid_PosEle.out_min = 0U;
        param->Pid_PosEle.out_max = 500U;
        PID_Reset(&param->Pid_PosEle);
    }
    if (param->Pid_PosVel.Kd != 0U || param->Pid_PosVel.out_min != 0U)
    {
        param->Pid_PosVel.Kd = 0U;
        param->Pid_PosVel.out_min = 0U;
        PID_Reset(&param->Pid_PosVel);
    }
}

void PID_Reset(PID_Int *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->prev_feedback = 0;
    pid->prev_out = 0;
}

int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue)
{
    int32_t err,out;
    int32_t previous_integral;
    int32_t candidate_integral;
    int32_t K,I,D;

    err = setValue - CurrentValue;
    if(abs(err)<=PID_AbsDEADBAND) err=0;

    previous_integral = pid->integral;
    candidate_integral = pid->integral + err;
    if (candidate_integral > pid->integral_max) candidate_integral = pid->integral_max;
    else if (candidate_integral < -pid->integral_max) candidate_integral = -pid->integral_max;

    K=pid->Kp*err/PID_SCALE;
    I=pid->Ki*candidate_integral/PID_SCALE;
    /* Derivative on feedback avoids a kick when the position target changes. */
    /* 对反馈量微分，避免位置目标阶跃造成微分冲击。 */
    D = pid->prev_feedback == 0 ? 0
        : -pid->Kd*(CurrentValue-pid->prev_feedback)
          / PID_POSITION_UPDATE_PERIOD_MS;
    out=K+I+D;
    /* Do not integrate further into output saturation; allow reverse error to unwind. */
    /* 输出饱和且误差同向时停止积分；误差反向时允许积分退出饱和。 */
    if ((out > pid->out_max && err > 0) ||
        (out < -pid->out_max && err < 0))
    {
        candidate_integral = previous_integral;
    }
    pid->integral = candidate_integral;
    I=pid->Ki*pid->integral/PID_SCALE;
    out=K+I+D;
    pid->prev_error=err;
    pid->prev_feedback=CurrentValue;

    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;
    if (out > 0 && out < pid->out_min) out = pid->out_min;
    else if (out < 0 && -out < pid->out_min) out = -(int32_t)pid->out_min;

    return out;
}

static int16_t PID_Vel_Calc(PID_Int* pid, int32_t target, int32_t feedback,
                            uint16_t update_period_ms, uint16_t output_limit,
                            uint16_t breakaway_speed_threshold_cps)
{
    int32_t error;
    int32_t P, I;
    int32_t out;
    int64_t candidate_integral;
    int64_t integral_limit;
    uint32_t integral_gain = pid->Ki;

    error = target - feedback;
    if (output_limit == 0U || output_limit > pid->out_max)
        output_limit = pid->out_max;
    if (breakaway_speed_threshold_cps != 0U &&
        abs(target) >= (int32_t)breakaway_speed_threshold_cps
                       * SPEED_BREAKAWAY_TARGET_MULTIPLIER &&
        abs(feedback) <= (int32_t)breakaway_speed_threshold_cps)
    {
        integral_gain *= SPEED_BREAKAWAY_KI_MULTIPLIER;
    }
    P = pid->Kp * error / PID_SCALE;
    integral_limit = (int64_t)pid->integral_max * 50000LL;
    if (integral_limit > INT32_MAX) integral_limit = INT32_MAX;
    candidate_integral = (int64_t)pid->integral
                       + (int64_t)integral_gain * error * update_period_ms;
    if (candidate_integral > integral_limit)
        candidate_integral = integral_limit;
    else if (candidate_integral < -integral_limit)
        candidate_integral = -integral_limit;
    I = FixedPoint_DivideS64ByU32(candidate_integral, 50000U);

    out = P + I;
    if (!((out >= output_limit && error > 0) ||
          (out <= -(int32_t)output_limit && error < 0)))
    {
        pid->integral = (int32_t)candidate_integral;
    }
    else
    {
        I = pid->integral / 50000L;
        out = P + I;
    }
    if (out > output_limit) out = output_limit;
    if (out < -(int32_t)output_limit) out = -(int32_t)output_limit;
    if (out > 0 && out < pid->out_min) out = pid->out_min;
    else if (out < 0 && -out < pid->out_min) out = -(int32_t)pid->out_min;

    return (int16_t)out;
}

int32_t PID_PositionLoop(Param *param, int32_t target_position)
{
    int32_t error = target_position - param->EncoderMultiTurnValue;

    if (abs(error) <= param->PositionDeadbandCounts)
    {
        param->Pid_Pos.integral = 0;
        return 0;
    }
    int32_t speed = PID_AbsCalculate(&param->Pid_Pos, target_position, param->EncoderMultiTurnValue);

    if (speed > param->SpeedMax) speed = param->SpeedMax;
    else if (speed < -(int32_t)param->SpeedMax) speed = -(int32_t)param->SpeedMax;

    return speed;
}

static int16_t PID_SpeedLoopLimited(Param *param, int32_t target_speed,
                                    uint16_t update_period_ms,
                                    uint16_t current_limit_mA)
{
    int32_t previous_planned_speed = param->SpeedRef;
    int32_t planned_speed = SpeedPlan_Update(param, target_speed,
                                             update_period_ms);
    if (planned_speed == 0
        || (previous_planned_speed ^ planned_speed) < 0)
    {
        PID_Reset(&param->Pid_PosVel);
    }
    int16_t current = PID_Vel_Calc(&param->Pid_PosVel, planned_speed,
                                   param->EncoderSpeed, update_period_ms,
                                   current_limit_mA,
                                   param->StallSpeedThreshold_cps);

    param->EncoderSpeedExpect = planned_speed;
    param->ExpectMA = current;
    return current;
}

int16_t PID_SpeedLoop(Param *param, int32_t target_speed,
                      uint16_t update_period_ms)
{
    return PID_SpeedLoopLimited(param, target_speed, update_period_ms,
                                param->Pid_PosVel.out_max);
}

int32_t PID_SpeedTorqueLoop(Param *param, int32_t target_speed,
                            uint16_t update_period_ms,
                            uint16_t current_limit_mA,
                            int32_t *reference_acceleration_cps2)
{
    int32_t previous_speed_reference = param->SpeedRef;
    int16_t current_equivalent_mA = PID_SpeedLoopLimited(
        param, target_speed, update_period_ms, current_limit_mA);
    int64_t acceleration = 0;

    if (update_period_ms != 0U)
    {
        acceleration = FixedPoint_DivideS64ByU32(
            ((int64_t)param->SpeedRef - previous_speed_reference) * 1000LL,
            update_period_ms);
    }
    if (acceleration > INT32_MAX) acceleration = INT32_MAX;
    else if (acceleration < INT32_MIN) acceleration = INT32_MIN;
    if (reference_acceleration_cps2 != NULL)
    {
        *reference_acceleration_cps2 = (int32_t)acceleration;
    }

    return MotorTorqueModel_CurrentToTorque(&param->MotorTorqueParams,
                                             current_equivalent_mA,
                                             param->MotorWindingTemperature_C);
}

int16_t PID_CurrentLoop(Param *param, int16_t target_current_mA)
{
    bool model_valid = false;
    int32_t model_pwm = 0;
    int32_t model_feedforward_pwm = 0;
    int32_t target_magnitude;
    int32_t target_sign;
    int32_t correction_mV = 0;
    int32_t correction_pwm = 0;
    PID_Int *current_pid = &param->Pid_PosEle;
    bool observable = false;
    bool direction_changed = false;
    bool regenerative_braking = false;

    if (target_current_mA != 0)
    {
        model_pwm = MotorTorqueModel_CurrentToDutyPermille(
            &param->MotorTorqueParams, target_current_mA, param->VCC_mV,
            param->EncoderSpeed, param->TorqueEncoderCountsPerRev,
            param->MotorWindingTemperature_C, &model_valid);
    }
    if (!model_valid) model_pwm = 0;
    target_magnitude = target_current_mA;
    if (target_magnitude < 0) target_magnitude = -target_magnitude;
    target_sign = target_current_mA > 0 ? 1L
                : target_current_mA < 0 ? -1L : 0L;

    /* A model voltage opposite to the requested current is regenerative.
     * The ground shunt cannot close a signed-current loop there. Convert the
     * requested braking current into bounded brake/coast PWM from Ke/R and
     * leave the PI frozen until the motoring quadrant becomes observable. */
    if ((target_sign > 0L && model_pwm < 0L) ||
        (target_sign < 0L && model_pwm > 0L))
    {
        int32_t back_emf_mV = param->MotorTorqueResult.back_emf_mV;
        int32_t brake_voltage_mV;
        int32_t brake_duty = 0;
        uint32_t resistance_mOhm =
            param->MotorTorqueResult.effective_resistance_mOhm;

        if (back_emf_mV < 0) back_emf_mV = -back_emf_mV;
        brake_voltage_mV = back_emf_mV
                         - param->MotorTorqueParams.brush_drop_mV;
        if (brake_voltage_mV > 0 && resistance_mOhm != 0U)
        {
            if (target_magnitude >= 1000L || resistance_mOhm > 4000000UL)
                brake_duty = 1000L;
            else
                brake_duty = (target_magnitude * (int32_t)resistance_mOhm
                              + brake_voltage_mV / 2L) / brake_voltage_mV;
        }
        if (brake_duty > 1000L) brake_duty = 1000L;
        model_pwm = target_sign * brake_duty;
        regenerative_braking = model_pwm != 0L;
    }
    model_feedforward_pwm = model_pwm;

    if (target_sign == 0L)
    {
        PID_Reset(current_pid);
    }
    else
    {
        direction_changed = current_pid->prev_prev_error != target_sign;
        observable = !regenerative_braking &&
            param->CurrentSampleAgeMs <= 3U &&
            param->CurrentSampleValid &&
            ((target_sign > 0L && model_pwm > 0) ||
             (target_sign < 0L && model_pwm < 0));

        if (direction_changed)
        {
            PID_Reset(current_pid);
            current_pid->prev_prev_error = target_sign;
        }

        if (observable && !direction_changed)
        {
            if (current_pid->prev_feedback < CURRENT_FEEDBACK_ARM_UPDATES)
            {
                current_pid->prev_feedback++;
            }
            else
            {
                correction_mV = PID_AbsCalculate(
                    current_pid, target_magnitude,
                    abs(param->CurrentAverage_mA));
            }
        }
        else
        {
            current_pid->prev_feedback = 0;
        }
    }

    if (param->VCC_mV != 0U)
    {
        correction_pwm = correction_mV * target_sign * 1000L
                       / param->VCC_mV;
        model_pwm += correction_pwm;
    }
    /* Current overshoot may remove drive voltage, but the current regulator
     * must not command reverse torque merely to discharge the winding. Decay
     * selection owns that transition. */
    if ((model_feedforward_pwm > 0L && model_pwm < 0L)
        || (model_feedforward_pwm < 0L && model_pwm > 0L))
    {
        model_pwm = 0L;
    }

    /* Diagnostics: publish the model feedforward and PI correction separately
     * so the host can attribute every PWM change to feedforward or feedback. */
    /* 诊断：分别发布模型前馈与 PI 修正，使上位机能够解释每次 PWM 变化的来源。 */
    {
        uint16_t loop_status = 0U;
        bool pi_running = observable && !direction_changed
            && target_sign != 0L
            && current_pid->prev_feedback >= CURRENT_FEEDBACK_ARM_UPDATES;

        if (pi_running) loop_status |= 0x0001U;
        if (param->CurrentSampleValid) loop_status |= 0x0002U;
        if (param->CurrentEstimated) loop_status |= 0x0004U;
        if (target_sign != 0L && !pi_running) loop_status |= 0x0008U;
        if (direction_changed) loop_status |= 0x0010U;
        if (model_pwm >= 1000L || model_pwm <= -1000L) loop_status |= 0x0020U;
        if (param->CurrentPeakLimitActive) loop_status |= 0x0040U;
        if (regenerative_braking) loop_status |= 0x0080U;
        if (pi_running && !param->CurrentPiWasRunning)
        {
            /* Frozen -> running transition closes one frozen interval. */
        }
        if (!pi_running && param->CurrentPiWasRunning)
        {
            param->CurrentPiFrozenCount++;
        }
        param->CurrentPiWasRunning = pi_running;
        param->CurrentLoopStatus = loop_status;
        param->CurrentCorrectionPwm = (int16_t)correction_pwm;
        {
            int32_t model_only = model_pwm - correction_pwm;
            if (model_only > 1000L) model_only = 1000L;
            else if (model_only < -1000L) model_only = -1000L;
            param->CurrentModelPwm = (int16_t)model_only;
        }
    }

    if (model_pwm > 1000) model_pwm = 1000;
    else if (model_pwm < -1000) model_pwm = -1000;

    param->ExpectMA = target_current_mA;
    if (param->EncoderVeer) model_pwm = -model_pwm;
    param->CurrentFeedforwardPwm = (int16_t)model_pwm;
    param->DrivePower = (int16_t)model_pwm;
    return (int16_t)model_pwm;

}

int16_t PID_Pos(Param *param)
{
    int32_t pos_err = param->EncoderMultiTurnValue - param->EncoderExpect;

    if (abs(pos_err) <= 3)
    {
        param->SpeedRef = 0;
        PID_Reset(&param->Pid_Pos);
        PID_Reset(&param->Pid_PosVel);
        return 0;
    }

    (void)PID_PositionLoop(param, param->EncoderExpect);
    (void)PID_SpeedLoop(param, param->EncoderSpeedExpect,
                        param->CycleTimeMs);
    return PID_CurrentLoop(param, param->ExpectMA);
}

int32_t SpeedPlan_Update(Param *param, int32_t SpeedCmd,
                         uint16_t update_period_ms)
{
    int32_t dv = SpeedCmd - param->SpeedRef;
    int32_t dv_limit = (int32_t)(dv > 0 ? param->AccelMax
                                        : param->DecelMax)
                     * update_period_ms;

    if (dv > 0)
    {
        if (dv > dv_limit) dv = dv_limit;
    }
    else
    {
        if (dv < -dv_limit) dv = -dv_limit;
    }

    param->SpeedRef += dv;
    return param->SpeedRef;
}

int16_t FeedForward_LUT(int32_t speed_abs)
{
    if (speed_abs <= speed_lut[0]) return pwm_lut[0];

    for (uint8_t i = 0; i < LUT_SIZE - 1; i++)
    {
        if (speed_abs <= speed_lut[i + 1])
        {
            int32_t ds = speed_lut[i + 1] - speed_lut[i];
            int32_t dp = pwm_lut[i + 1] - pwm_lut[i];
            int32_t dx = speed_abs - speed_lut[i];

            return pwm_lut[i] + (dp * dx) / ds;
        }
    }

    return pwm_lut[LUT_SIZE - 1];
}

int16_t Speed_FeedForward(int32_t speed)
{
    int32_t abs_spd = abs(speed);
    int16_t pwm;

    if (abs_spd < SPEED_LOW) pwm = FeedForward_LUT(abs_spd);
    else if (abs_spd < SPEED_MID) pwm = (abs_spd * KV_Q) / FF_SCALE + PWM_DEAD;
    else pwm = (abs_spd * KV_Q) / FF_SCALE;

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    return (speed >= 0) ? pwm : -pwm;
}
