#include "Car_Core.h"
#include "Car_Config.h"
#include <string.h>

static float absolute(float x) { return x < 0 ? -x : x; }
static float clamp(float x, float lim) { return x > lim ? lim : (x < -lim ? -lim : x); }
static void reset_motion(Car_Core *c)
{
    unsigned i;
    CarReference_Reset(&c->profile);
    for(i=0;i<2;i++) {
        c->target[i]=c->reference[i]=c->output[i]=0;
        c->stall_us[i]=c->direction_us[i]=0;
        c->slow_speed[i]=absolute(c->speed[i]);
        Car_Pid_reset(&c->pid[i]);
    }
}
void Car_Init(Car_Core *c)
{
    unsigned i;
    memset(c,0,sizeof(*c));
    CarReference_Init(&c->profile,CAR_RPM_LIMIT,CAR_ACCEL_LIMIT_RPM_S,CAR_JERK_LIMIT_RPM_S2);
    for(i=0;i<2;i++) {
        Car_Pid_init(&c->pid[i],CAR_DEFAULT_KP,CAR_DEFAULT_KI,CAR_DEFAULT_KD,CAR_PWM_LIMIT);
        if(!Car_Pid_configure(&c->pid[i],CAR_PID_SETPOINT_WEIGHT,CAR_PID_D_TAU_S,1,CAR_FF_KS,CAR_FF_KV,CAR_FF_KA))
            Car_Fault(c,CAR_CONTROLLER_FAULT);
    }
}
void Car_Fault(Car_Core *c, uint8_t fault)
{
    if(c->state!=CAR_FAULT)c->fault=fault;
    reset_motion(c); c->state=CAR_FAULT;
    c->parse_state=0;
}
uint8_t Car_Crc8(uint8_t crc, uint8_t byte)
{
    unsigned i;
    crc^=byte;
    for(i=0;i<8;i++) crc=(uint8_t)((crc&0x80U) ? (crc<<1)^0x07U : crc<<1);
    return crc;
}
static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]*256U+p[1]); }
static float signed_rpm(const uint8_t *p)
{
    uint16_t n=u16(p);
    return (float)((n&0x8000U) ? (int32_t)n-65536 : (int32_t)n);
}
static void execute(Car_Core *c)
{
    c->frames++;
    c->result=0;
    switch(c->cmd) {
    case 1:
        if(c->state!=CAR_STOP) { c->result=1; break; }
        reset_motion(c); c->link_us=0; c->state=CAR_RUN; break;
    case 2:
        reset_motion(c); c->state=CAR_STOP; c->fault=0; c->link_us=0; break;
    case 3:
        if(c->state!=CAR_RUN) { c->result=1; break; }
        c->target[0]=clamp(signed_rpm(c->data),CAR_RPM_LIMIT);
        c->target[1]=clamp(signed_rpm(c->data+2),CAR_RPM_LIMIT);
        if(c->target[0]==0 && c->target[1]==0) reset_motion(c);
        c->link_us=0; break;
    case 4:
        if(c->state!=CAR_STOP) { c->result=1; break; }
        if(c->data[0]>1) { c->result=2; break; }
        {
            Car_Pid_Struct next;
            if(!Car_Pid_init(&next,u16(c->data+1)/100.0f,u16(c->data+3)/100.0f,
                             u16(c->data+5)/100.0f,CAR_PWM_LIMIT) ||
               !Car_Pid_configure(&next,CAR_PID_SETPOINT_WEIGHT,CAR_PID_D_TAU_S,1,
                                   CAR_FF_KS,CAR_FF_KV,CAR_FF_KA)) { c->result=2;break; }
            c->pid[c->data[0]]=next;
        }
        break;
    case 5:
        if(c->state==CAR_RUN) c->link_us=0;
        break;
    default: c->result=3; break;
    }
}
void Car_FeedByte(Car_Core *c, uint8_t byte)
{
    static const uint8_t lengths[6]={255,0,0,4,7,0};
    c->frame_us=0;
    switch(c->parse_state) {
    case 0: if(byte==0xc3) c->parse_state=1; break;
    case 1: c->parse_state=byte==0x3c ? 2 : (byte==0xc3 ? 1 : 0); break;
    case 2:
        c->cmd=byte; c->crc=Car_Crc8(0,byte); c->parse_state=3; break;
    case 3:
        if(c->cmd<1 || c->cmd>5 || byte!=lengths[c->cmd]) {
            c->parse_state=byte==0xc3 ? 1 : 0; c->result=3; break;
        }
        c->length=byte; c->index=0; c->crc=Car_Crc8(c->crc,byte);
        c->parse_state=byte ? 4 : 5; break;
    case 4:
        c->data[c->index++]=byte; c->crc=Car_Crc8(c->crc,byte);
        if(c->index==c->length) c->parse_state=5;
        break;
    default:
        c->parse_state=0;
        if(byte==c->crc) execute(c);
        else { c->result=3; if(byte==0xc3) c->parse_state=1; }
        break;
    }
}
void Car_TickDt(Car_Core *c, int32_t left_count, int32_t right_count, uint32_t dt_us)
{
    int32_t counts[2];
    unsigned i;
    float dt=dt_us*0.000001f;
    if(dt_us<CAR_MIN_CONTROL_US || dt_us>CAR_MAX_CONTROL_US) { Car_Fault(c,CAR_TIMING_FAULT);return; }
    counts[0]=left_count; counts[1]=right_count;
    if(c->parse_state && (c->frame_us+=dt_us)>=CAR_FRAME_TIMEOUT_MS*1000U) c->parse_state=0;
    if(c->state==CAR_RUN && (c->link_us+=dt_us)>=CAR_LINK_TIMEOUT_MS*1000U) Car_Fault(c,CAR_LINK_FAULT);
    if(c->state==CAR_RUN)CarReference_Update(&c->profile,c->target,dt);
    for(i=0;i<2;i++) {
        float raw=(float)counts[i]*(60.0f/(CAR_ENCODER_CPR*dt));
        float abs_speed;
        c->speed[i]+=(dt/(CAR_SPEED_FILTER_TAU_S+dt))*(raw-c->speed[i]);
        if(c->state!=CAR_RUN) { c->output[i]=0; continue; }
        c->reference[i]=c->profile.value[i];
        if(c->reference[i]==0 && c->target[i]==0) {
            c->output[i]=0; Car_Pid_reset(&c->pid[i]);
        } else c->output[i]=Car_Pid_update_dt(&c->pid[i],c->reference[i],c->speed[i],c->profile.acceleration[i],dt);
        if(c->pid[i].fault) { Car_Fault(c,CAR_CONTROLLER_FAULT);continue; }
        abs_speed=absolute(c->speed[i]);
        c->slow_speed[i]+=(dt/(0.095f+dt))*(abs_speed-c->slow_speed[i]);
        if(absolute(c->reference[i])>=40 && absolute(c->output[i])>=1500 && abs_speed<10) {
            c->stall_us[i]+=dt_us;
            if(c->stall_us[i]>=CAR_STALL_TIMEOUT_MS*1000U) Car_Fault(c,CAR_STALL_FAULT);
        } else c->stall_us[i]=0;
        if(c->reference[i]*c->speed[i]<0 && c->output[i]*c->speed[i]<0 &&
           absolute(c->output[i])>0.9f*CAR_PWM_LIMIT && abs_speed>100 &&
           abs_speed>=c->slow_speed[i]-5) {
            c->direction_us[i]+=dt_us;
            if(c->direction_us[i]>=CAR_DIRECTION_TIMEOUT_MS*1000U) Car_Fault(c,CAR_DIRECTION_FAULT);
        } else c->direction_us[i]=0;
    }
}

/* Fixed-period compatibility for host simulations. Hardware calls Car_TickDt. */
void Car_Tick(Car_Core *c,int32_t left_count,int32_t right_count)
{ Car_TickDt(c,left_count,right_count,CAR_PERIOD_MS*1000U); }
