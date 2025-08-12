#include "pid.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "pid";

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float kf, float output_max, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;

    pid->output_max = output_max;
    pid->dt = dt;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    ESP_LOGI(TAG, "PID initialized: kp=%f, ki=%f, kd=%f, kf=%f, dt=%f", kp, ki, kd, kf, dt);
}

float pid_update(pid_controller_t *pid, float setpoint, float feedback, float feedforward)
{
    float error = setpoint - feedback;
    float derivative = -(error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    float output_unsat =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative +
        pid->kf * feedforward;

    float output = output_unsat;
    if (output > pid->output_max)
        output = pid->output_max;
    if (output < -pid->output_max)
        output = -pid->output_max;

    bool at_upper_limit = (output >= pid->output_max);
    bool at_lower_limit = (output <= -pid->output_max);

    if ((!at_upper_limit && !at_lower_limit) ||
        (at_upper_limit && error < 0) ||
        (at_lower_limit && error > 0))
    {
        pid->integral += error * pid->dt;
    }

    output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative +
        pid->kf * feedforward;

    if (output > pid->output_max)
        output = pid->output_max;
    if (output < -pid->output_max)
        output = -pid->output_max;

    ESP_LOGD(TAG, "setpoint: %f, feedback: %f, feedforward: %f, error: %f, output: %f",
             setpoint, feedback, feedforward, error, output);

    return output;
}
