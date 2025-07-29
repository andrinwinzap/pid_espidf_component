#include "pid.h"
#include "esp_log.h"

static const char *TAG = "pid";

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt = dt;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    ESP_LOGI(TAG, "PID initialized: kp=%.4f, ki=%.4f, kd=%.4f", kp, ki, kd);
}

float pid_update(pid_controller_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    pid->integral += error * pid->dt;
    float derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    ESP_LOGD(TAG, "setpoint: %f, feedback: %f, error: %f, output: %f", setpoint, feedback, error, output);

    return output;
}