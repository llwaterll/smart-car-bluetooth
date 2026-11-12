#include "Car_PID.h"
#include <float.h>
#include <string.h>

static uint8_t number(float x) { return x<=FLT_MAX && x>=-FLT_MAX; }
static float clip(float x,float limit) { return x>limit ? limit : (x<-limit ? -limit : x); }
void Car_Pid_reset(Car_Pid_Struct *p)
{
    p->integral=p->last_measure=p->derivative=0;
    p->p_term=p->d_term=p->ff_term=p->last_output=0;
    p->started=p->fault=p->saturated=0;
}
uint8_t Car_Pid_init(Car_Pid_Struct *p,float kp,float ki,float kd,float limit)
{
    memset(p,0,sizeof(*p));
    if(!number(kp)||!number(ki)||!number(kd)||!number(limit)||
       kp<0||ki<0||kd<0||kp>655.35f||ki>655.35f||kd>655.35f||limit<=0||limit>10000) {
        p->fault=1;return 0;
    }
    p->kp=kp;p->ki=ki;p->kd=kd;p->output_limit=p->integral_limit=limit;
    p->beta=1;p->derivative_tau_s=0.02f;
    return 1;
}
uint8_t Car_Pid_configure(Car_Pid_Struct *p,float beta,float tau,float fraction,
                         float ks,float kv,float ka)
{
    if(!number(beta)||!number(tau)||!number(fraction)||!number(ks)||!number(kv)||!number(ka)||
       beta<0||beta>1||tau<0||tau>1||fraction<=0||fraction>1||ks<0||kv<0||ka<0||
       ks>p->output_limit||kv>10000||ka>10000||p->output_limit<=0)return 0;
    p->beta=beta;p->derivative_tau_s=tau;p->integral_limit=fraction*p->output_limit;
    p->ks=ks;p->kv=kv;p->ka=ka;Car_Pid_reset(p);return 1;
}
float Car_Pid_update_dt(Car_Pid_Struct *p,float target,float measure,float acceleration,float dt)
{
    float error,raw_d=0,d_alpha,base,candidate,output;
    if(p->fault)return 0;
    if(!number(target)||!number(measure)||!number(acceleration)||!number(dt)||dt<0.0001f||dt>0.1f||
       !number(p->kp)||!number(p->ki)||!number(p->kd)||!number(p->integral)||
       !number(p->output_limit)||p->output_limit<=0||p->output_limit>10000)goto invalid;
    error=target-measure;
    if(p->started)raw_d=(measure-p->last_measure)/dt;
    p->last_measure=measure;p->started=1;
    d_alpha=dt/(p->derivative_tau_s+dt);
    p->derivative+=d_alpha*(raw_d-p->derivative);
    p->p_term=p->kp*(p->beta*target-measure);
    p->d_term=-p->kd*0.005f*p->derivative;
    p->ff_term=p->kv*target+p->ka*acceleration;
    if(target>1)p->ff_term+=p->ks;else if(target<-1)p->ff_term-=p->ks;
    base=p->p_term+p->d_term+p->ff_term;
    candidate=clip(p->integral+p->ki*error*(dt/0.005f),p->integral_limit);
    if(!number(base)||!number(candidate))goto invalid;
    output=base+candidate;
    /* Conditional integration allows unwinding, blocks growth into saturation. */
    if(!((output>p->output_limit && error>0)||(output<-p->output_limit && error<0)))
        p->integral=candidate;
    output=base+p->integral;
    if(!number(output))goto invalid;
    p->saturated=(output>p->output_limit || output<-p->output_limit);
    p->last_output=clip(output,p->output_limit);return p->last_output;
invalid:
    p->fault=1;p->last_output=0;return 0;
}
float Car_Pid_update(Car_Pid_Struct *p,float target,float measure)
{ return Car_Pid_update_dt(p,target,measure,0,0.005f); }
