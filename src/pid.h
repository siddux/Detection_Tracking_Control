#ifndef PID_H_
#define PID_H_

#include <cstdint>

__BEGIN_DECLS

typedef struct {
    float dt_min;
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
    float error_previous;
    float last_output;
} PID_t;

void pid_init(PID_t *pid, float dt_min);
int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float pid_calculate(PID_t *pid, float sp, float val, float dt);
void pid_reset_integral(PID_t *pid);

__END_DECLS

#endif /* PID_H_ */