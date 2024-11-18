// Commands for read weight from HX710A/HX711/HX712.
//
// Copyright (C) 2023  guoge <guoge@tiertime.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" //
#include "basecmd.h" //oid_alloc
#include "command.h"  //sendf
#include "sched.h" //DECL_COMMAND
#include "board/gpio.h" //GPIO/read/setup
#include "board/misc.h" // timer_read_time
#include "board/irq.h" // irq_disable


#define HX71X_SAMPLE_START  0x01
#define HX71X_SAMPLE_NOW    0x02

#define MAX_SENSOR      2  //max 6 sensor.

//hx71x唤醒信号
static struct task_wake s_Hx71x_Wake;

static uint32_t s_delayCnt2 = 0;
static uint32_t s_delayclk= 123;

struct hx71x_s {
    struct timer hx71x_timer;       //定时器,启动
    uint32_t sample_ticks;          //采样时钟间隔.
    uint32_t sample_times;          //采样总次数.
    uint32_t sample_cnt;            //采样计数
    uint8_t  flag;                  //标志位.

    uint32_t sensors;               //传感器数量
    struct gpio_out sck_out[MAX_SENSOR];    //传感器管脚
    struct gpio_in dt_in[MAX_SENSOR];
    uint32_t value[MAX_SENSOR];
    uint32_t pulse_cnt;
    uint32_t delayloop;
};

//hx71x定时器回调函数, 设定唤醒信号(系统的task遍历中启动hx71x读取函数)
static uint_fast8_t hx71x_sample_event(struct timer* t)
{
    //设定task唤醒标志位
	sched_wake_task(&s_Hx71x_Wake);
    //从定时器地址来获取hx71x_s的地址.
	struct hx71x_s* dev = container_of(t, struct hx71x_s, hx71x_timer);
    //设定采样标志位.定时器触发时刻
    dev->flag |= HX71X_SAMPLE_NOW;
	dev->hx71x_timer.waketime += dev->sample_ticks;
    //是否继续启动下一个定时.(继续采样或停止)
	return --dev->sample_times>0 ? SF_RESCHEDULE : SF_DONE;
}

uint32_t HX711_Read(struct hx71x_s *dev);

//delay for wait data ready.
static inline void
hx71x_delay_waitdata(uint32_t var, uint32_t loop)
{
    if( loop==0 )
        return;
    for(uint32_t i=0; i<loop; i++)
        var += i;
    s_delayCnt2 += var;
}

// microsecond delay helper
static inline void
hx71x_udelay(uint32_t usecs)
{
    uint32_t end = timer_read_time() + timer_from_us(usecs);
    while (!timer_is_before(end, timer_read_time()));
}

void command_config_hx71x(uint32_t *args)
{
    struct hx71x_s *hx71x = oid_alloc(args[0], command_config_hx71x, sizeof(*hx71x));
    for(uint32_t i=0; i<MAX_SENSOR; i++)
    {
        uint32_t sck = args[i*2+1];
        uint32_t dt = args[i*2+2];
        if( sck==dt ) //相同管脚,停止添加sensor.
            break;
        hx71x->sck_out[i] = gpio_out_setup(sck, 1);
        hx71x->dt_in[i] = gpio_in_setup(dt, 1);
        gpio_out_write(hx71x->sck_out[i], 0);
        hx71x->sensors = i+1;
    }

    //reset value
    for(uint32_t i=0; i<MAX_SENSOR; i++)
        hx71x->value[i] = 0;

    hx71x->hx71x_timer.func = hx71x_sample_event;
    hx71x->sample_ticks = 100000000;
    hx71x->sample_times = 0;
    hx71x->sample_cnt = 0;
}
DECL_COMMAND(command_config_hx71x,
    "config_hx71x oid=%c sa=%u da=%u sb=%u db=%u");// sc=%u dc=%u sd=%u dd=%u se=%u de=%u sf=%u df=%u");


struct hx71x_s * hx71x_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_hx71x);
}

//查询hx71x命令, 不直接返回值,是个config命令.启动定时器,在一定时间后开始返回取值.
void command_query_hx71x(uint32_t * args)
{
    struct hx71x_s *dev = hx71x_oid_lookup(args[0]);
    dev->sample_ticks = args[1];
    dev->sample_times = args[2];
    dev->pulse_cnt = args[3];
    dev->delayloop = args[4];
    dev->sample_cnt = 0;

    // uint32_t t1 = timer_read_time();
    // uint32_t t2 = timer_read_time();
    // hx71x_delay_waitdata(s_delayclk);
    // uint32_t t3 = timer_read_time();
    // s_delayclk = t3 - t2 - (t2-t1);

    sched_del_timer(&dev->hx71x_timer); //删除旧定时器
    //添加新定时器.(需要先关闭中断)
	irq_disable();
	dev->hx71x_timer.waketime = timer_read_time() + dev->sample_ticks;
	sched_add_timer(&dev->hx71x_timer);
	irq_enable();
}
DECL_COMMAND(command_query_hx71x, "query_hx71x oid=%c ticks=%u times=%u pulse_cnt=%u delayloop=%u");


#define foreach_sensor(i, s) for(uint32_t i=0; i<s; i++)

//read data from HX711
uint32_t HX711_Read(struct hx71x_s *dev)
{
    uint32_t s = dev->sensors;
    uint32_t *v = dev->value;
    uint32_t loop = dev->delayloop;
    struct gpio_out *sck = dev->sck_out;
    struct gpio_in *dt = dev->dt_in;

    foreach_sensor(i, s)
    {
        gpio_out_write(sck[i], 0);
        v[i] = 0; //clear data
    }
    // hx71x_udelay(1);

    //wait all dout to low.
    uint32_t nCnt = 0;
    while ( 1 )
    {
        uint32_t n = s;
        //wait all data_in is low
        foreach_sensor(i, s)
        {
            if( !gpio_in_read(dt[i]))
                n--;
        }
        if( n==0 ) //all read 0(low);
            break;
        hx71x_udelay(10);   //10us
        if (nCnt++> 100) //max 1ms.
            return 0; //not change the value of HX71X
    }

    //read 24bit data and mode pulse.
    for (uint32_t j=0; j<dev->pulse_cnt; j++)
    {
        irq_disable();
        foreach_sensor(i, s)
            gpio_out_write(sck[i], 1);
        // hx71x_udelay(1);
        hx71x_delay_waitdata(s_delayclk, loop);

        foreach_sensor(i, s)
            gpio_out_write(sck[i], 0);
        irq_enable();

        // hx71x_udelay(1);
        hx71x_delay_waitdata(s_delayclk, loop);

        if( j>=24) //data read finish.
            continue;

        foreach_sensor(i, s)
        {
            v[i] = v[i]<<1;
            if( gpio_in_read(dt[i]))
                v[i]++;
        }
    }

    foreach_sensor(i, s)
        v[i] ^= 0x800000;

    return s;
}

void
hx71x_query_task(void)
{
    if (!sched_check_wake(&s_Hx71x_Wake))
        return;

    uint8_t oid;
    struct hx71x_s *dev;
    foreach_oid(oid, dev, command_config_hx71x) {
        if( dev->sample_times==0 || !(dev->flag&HX71X_SAMPLE_NOW))
            continue;

        dev->flag &= ~HX71X_SAMPLE_NOW;

        uint32_t next_waketime = dev->hx71x_timer.waketime;

        //读取数据,计数+1
        //uint32_t nRead = HX711_Read(dev);
        HX711_Read(dev);
        dev->sample_cnt++;

        //发送返回数据
        uint32_t* v = dev->value;
        sendf("hx71x_state oid=%c v0=%u v1=%u v2=%u v3=%u v4=%u v5=%u cnt=%u next_clock=%u", oid, v[0], v[1],v[2],v[3],v[4],v[5],dev->sample_cnt, next_waketime);
    }
}
DECL_TASK(hx71x_query_task);

void
hx71x_query_shutdown(void)
{
    uint8_t i;
    struct hx71x_s *dev;
    foreach_oid(i, dev, command_config_hx71x) {
        dev->sample_times = 0;
    }
}
DECL_SHUTDOWN(hx71x_query_shutdown);