#include "Car_Reference.h"
#include <math.h>
#include <string.h>

static float absolute(float x) { return x<0?-x:x; }
static float max(float x,float y) { return x>y?x:y; }
static float clip(float x,float lim) { return x>lim?lim:(x<-lim?-lim:x); }
void CarReference_Reset(CarReference *r)
{
    unsigned i;
    for(i=0;i<2;i++)r->value[i]=r->acceleration[i]=r->goal[i]=r->start[i]=r->brake_acceleration[i]=0;
    r->phase=0;r->elapsed=r->duration=0;
}
void CarReference_Init(CarReference *r,float speed,float acceleration,float jerk)
{
    memset(r,0,sizeof(*r));r->speed_limit=speed;r->acceleration_limit=acceleration;r->jerk_limit=jerk;
}
static void begin_profile(CarReference *r)
{
    float distance=max(absolute(r->goal[0]-r->value[0]),absolute(r->goal[1]-r->value[1]));
    r->start[0]=r->value[0];r->start[1]=r->value[1];r->elapsed=0;
    /* Exact maxima of smoothstep5': 1.875, |smoothstep5''|: 10/sqrt(3). */
    r->duration=max(0.005f,max(1.875f*distance/r->acceleration_limit,
                             sqrtf(5.773503f*distance/r->jerk_limit)));
    r->phase=1;
}
void CarReference_Update(CarReference *r,const float target[2],float dt)
{
    unsigned i;float goal[2],peak,remaining,step,t0,t1,progress,rate;
    if(!(dt>0 && dt<=0.1f && r->speed_limit>0 && r->acceleration_limit>0 && r->jerk_limit>0))return;
    for(i=0;i<2;i++) {
        if(!(target[i]>=-100000 && target[i]<=100000))return;
        goal[i]=clip(target[i],r->speed_limit);
    }
    if(goal[0]!=r->goal[0] || goal[1]!=r->goal[1]) {
        r->goal[0]=goal[0];r->goal[1]=goal[1];
        peak=max(absolute(r->acceleration[0]),absolute(r->acceleration[1]));
        if(peak>0.00001f) {
            r->phase=2;r->elapsed=0;r->duration=peak/r->jerk_limit;
            for(i=0;i<2;i++) { r->start[i]=r->value[i];r->brake_acceleration[i]=r->acceleration[i]; }
        } else begin_profile(r);
    }
    remaining=dt;
    if(r->phase==2) {
        step=r->duration-r->elapsed;if(step>remaining)step=remaining;
        r->elapsed+=step;remaining-=step;t1=r->elapsed/r->duration;
        for(i=0;i<2;i++) {
            r->value[i]=r->start[i]+r->brake_acceleration[i]*r->duration*(t1-0.5f*t1*t1);
            r->acceleration[i]=r->brake_acceleration[i]*(1-t1);
        }
        if(r->elapsed>=r->duration) {
            r->acceleration[0]=r->acceleration[1]=0;begin_profile(r);
        }
    }
    if(r->phase==1 && remaining>0) {
        r->elapsed+=remaining;
        t0=r->elapsed/r->duration;if(t0>1)t0=1;
        progress=t0*t0*t0*(10+t0*(-15+6*t0));
        rate=30*t0*t0*(1-t0)*(1-t0)/r->duration;
        for(i=0;i<2;i++) {
            r->value[i]=r->start[i]+(r->goal[i]-r->start[i])*progress;
            r->acceleration[i]=(r->goal[i]-r->start[i])*rate;
        }
        if(r->elapsed>=r->duration) {
            r->phase=0;
            for(i=0;i<2;i++) { r->value[i]=r->goal[i];r->acceleration[i]=0; }
        }
    }
}
