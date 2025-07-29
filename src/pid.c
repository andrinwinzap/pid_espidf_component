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

float pid_update(pid_controller_t *pid, float target, float measured)
{
    float error = target - measured;
    pid->integral += error * pid->dt;
    float derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    ESP_LOGD(TAG, "target: %f, measured: %f, error: %f, output: %f", target, measured, error, output);

    return output;
}