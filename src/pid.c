#include "pid.h"
#include "esp_log.h"

static const char *TAG = "pid";

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float kf, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;

    pid->dt = dt;

    pid->integral = 0.0f;
    pid->prev_feedback = 0.0f;

    ESP_LOGI(TAG, "PID initialized: kp=%f, ki=%f, kd=%f, kf=%f, dt=%f", kp, ki, kd, kf, dt);
}

float pid_update(pid_controller_t *pid, float setpoint, float feedback, float feedforward)
{
    float error = setpoint - feedback;
    pid->integral += error * pid->dt;
    float derivative = -(feedback - pid->prev_feedback) / pid->dt;
    pid->prev_feedback = feedback;

    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative +
        pid->kf * feedforward;

    ESP_LOGD(TAG, "setpoint: %f, feedback: %f, feedforward: %f, error: %f, output: %f", setpoint, feedback, feedforward, error, output);

    return output;
}
