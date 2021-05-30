#include "pid.h"
#include <math.h>

#define SIGMA 0.000001f

void pid_init(PID_t *pid, float dt_min)
{
    pid->dt_min = dt_min;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->integral_limit = 0.0f;
    pid->output_limit = 0.0f;
    pid->error_previous = 0.0f;
    pid->last_output = 0.0f;
}

bool is_finite(float x){
    return __builtin_isfinite(x);
}

int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    int ret = 0;

    if (is_finite(kp)) {
        pid->kp = kp;

    } else {
        ret = 1;
    }

    if (is_finite(ki)) {
        pid->ki = ki;

    } else {
        ret = 1;
    }

    if (is_finite(kd)) {
        pid->kd = kd;

    } else {
        ret = 1;
    }

    if (is_finite(integral_limit)) {
        pid->integral_limit = integral_limit;

    }  else {
        ret = 1;
    }

    if (is_finite(output_limit)) {
        pid->output_limit = output_limit;

    }  else {
        ret = 1;
    }

    return ret;
}

float pid_calculate(PID_t *pid, float sp, float val, float dt)
{
    if (!is_finite(sp) || !is_finite(val) || !is_finite(dt)) {
        return pid->last_output;
    }

    float i, d;

    /* current error value */
    float error = sp - val;

    /* current error derivative */
    d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
    pid->error_previous = error;

    if (!is_finite(d)) {
        d = 0.0f;
    }

    /* calculate PD output */
    float output = (error * pid->kp) + (d * pid->kd);

    if (pid->ki > SIGMA) {
        // Calculate the error integral and check for saturation
        i = pid->integral + (error * dt);

        /* check for saturation */
        if (is_finite(i)) {
            if ((pid->output_limit < SIGMA || (fabsf(output + (i * pid->ki)) <= pid->output_limit)) &&
                fabsf(i) <= pid->integral_limit) {
                /* not saturated, use new integral value */
                pid->integral = i;
            }
        }

        /* add I component to output */
        output += pid->integral * pid->ki;
    }

    /* limit output */
    if (is_finite(output)) {
        if (pid->output_limit > SIGMA) {
            if (output > pid->output_limit) {
                output = pid->output_limit;

            } else if (output < -pid->output_limit) {
                output = -pid->output_limit;
            }
        }

        pid->last_output = output;
    }

    return pid->last_output;
}

void pid_reset_integral(PID_t *pid)
{
    pid->integral = 0.0f;
}

int main(int argc, char **argv)
{
    return 0;
}
