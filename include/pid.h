#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;

    float integral;
    float prev_error;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);
float pid_update(pid_controller_t *pid, float setpoint, float feedback);

#endif // PID_H