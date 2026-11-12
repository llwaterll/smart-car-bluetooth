#ifndef CAR_PID_H
#define CAR_PID_H
#include <stdint.h>
/* Parallel speed PID. For compatibility with the Bluetooth tool, Ki/Kd keep
 * their historical 5ms discrete units: I += Ki*error*dt/0.005;
 * D = -Kd*0.005*d(measure)/dt. Feedforward uses PWM-count units, rpm, rpm/s.
 * Configure only while stopped; these pure functions do not arbitrate modes. */
typedef struct {
    float kp,ki,kd,output_limit,integral,last_measure;
    float beta,derivative_tau_s,integral_limit,ks,kv,ka;
    float derivative,p_term,d_term,ff_term,last_output;
    uint8_t started,fault,saturated;
} Car_Pid_Struct;
uint8_t Car_Pid_init(Car_Pid_Struct *p,float kp,float ki,float kd,float limit);
uint8_t Car_Pid_configure(Car_Pid_Struct *p,float beta,float derivative_tau_s,
                         float integral_fraction,float ks,float kv,float ka);
void Car_Pid_reset(Car_Pid_Struct *p);
float Car_Pid_update_dt(Car_Pid_Struct *p,float target,float measure,
                       float target_acceleration,float dt_s);
float Car_Pid_update(Car_Pid_Struct *p,float target,float measure);
#endif
