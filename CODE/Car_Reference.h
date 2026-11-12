#ifndef CAR_REFERENCE_H
#define CAR_REFERENCE_H
/* Paired wheel reference in rpm. Both wheels share progress to avoid changing
 * curvature during an initial acceleration. On retarget, brake reference
 * acceleration to zero before constructing a new bounded quintic profile.
 * This is not physical vehicle braking; emergency STOP calls Reset directly. */
typedef struct {
    float value[2],acceleration[2],goal[2],start[2],brake_acceleration[2];
    float speed_limit,acceleration_limit,jerk_limit,elapsed,duration;
    unsigned phase;
} CarReference;
void CarReference_Init(CarReference *r,float speed_limit,float acceleration_limit,float jerk_limit);
void CarReference_Reset(CarReference *r);
void CarReference_Update(CarReference *r,const float target[2],float dt_s);
#endif
