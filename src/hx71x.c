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

//hx71x唤醒信号
static struct task_wake s_Hx71x_Wake;

struct hx71x_s {
    struct timer hx71x_timer;       //定时器,启动
    uint32_t sample_ticks;          //采样时钟间隔.
    uint32_t sample_times;          //采样总次数.
    uint32_t sample_cnt;            //采样计数
    uint8_t  flag;                  //标志位.

    struct gpio_out sck_out;
    struct gpio_in dt_in;
    long weight_tare;
    long pulse_cnt;
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

long HX711_Read(struct hx71x_s *dev);
void HX711_Get_WeightTare(struct hx71x_s *dev);
long HX711_Get_Weight(struct hx71x_s *dev);

void command_config_hx71x(uint32_t *args)
{
    struct hx71x_s *hx71x = oid_alloc(args[0], command_config_hx71x
                                     , sizeof(*hx71x));
    hx71x->sck_out = gpio_out_setup(args[1], 1);
    hx71x->dt_in = gpio_in_setup(args[2], 1);
    gpio_out_write(hx71x->sck_out, 0);

    hx71x->weight_tare = 0;

    hx71x->hx71x_timer.func = hx71x_sample_event;
    hx71x->sample_ticks = 100000000;
    hx71x->sample_times = 0;
    hx71x->sample_cnt = 0;
}
DECL_COMMAND(command_config_hx71x,
    "config_hx71x oid=%c sck_pin=%u dout_pin=%u");


struct hx71x_s * hx71x_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_hx71x);
}


// void command_read_hx71x(uint32_t * args)
// {
//     static int s_nCnt = 0;
//     s_nCnt++;

//     uint8_t oid = args[0];
//     struct hx71x_s *dev = hx71x_oid_lookup(args[0]);
//     dev->pulse_cnt = args[1];
//     uint8_t data_len = 4;
//     uint8_t data[data_len];

//     long weight = HX711_Get_Weight(dev);
//     data[0] = weight & 0xFF;
//     data[1] = (weight>>8) & 0xFF;
//     data[2] = (weight>>16) & 0xFF;
//     data[3] = (weight>>24) & 0xFF;
//     sendf("read_hx71x_response oid=%c response=%*s", oid, data_len, data);
// }
// DECL_COMMAND(command_read_hx71x, "read_hx71x oid=%c read_len=%u");

//查询hx71x命令, 不直接返回值,是个config命令.启动定时器,在一定时间后开始返回取值.
void command_query_hx71x(uint32_t * args)
{
    struct hx71x_s *dev = hx71x_oid_lookup(args[0]);
    dev->sample_ticks = args[1];
    dev->sample_times = args[2];
    dev->pulse_cnt = args[3];
    dev->sample_cnt = 0;

    sched_del_timer(&dev->hx71x_timer); //删除旧定时器
    //添加新定时器.(需要先关闭中断)
	irq_disable();
	dev->hx71x_timer.waketime = timer_read_time() + dev->sample_ticks;
	sched_add_timer(&dev->hx71x_timer);
	irq_enable();
}
DECL_COMMAND(command_query_hx71x, "query_hx71x oid=%c ticks=%u times=%u pulse_cnt=%u");


// microsecond delay helper
static inline void
hx71x_udelay(uint32_t usecs)
{
    uint32_t end = timer_read_time() + timer_from_us(usecs);
    while (!timer_is_before(end, timer_read_time()));
}

//read data form HX711
long HX711_Read(struct hx71x_s *dev)
{
    // gpio_out_reset(dev->sck_out, 0);
    gpio_out_write(dev->sck_out, 0);
    hx71x_udelay(1);

    //wait dout to low.
    //gpio_in_reset(dev->dt_in, 1);
    int nCnt = 0;
    while ( gpio_in_read(dev->dt_in) )
    {
        hx71x_udelay(1);
        if (nCnt++> 10 * 1000) //max 10ms.
            return 0;
    }

    //read 24bit data.
    unsigned long count=0;
    for (int i = 0; i < 24; i++)
    {
        gpio_out_write(dev->sck_out, 1);
        hx71x_udelay(1);

        count = count << 1;

        gpio_out_write(dev->sck_out, 0);
        hx71x_udelay(1);

        if( gpio_in_read(dev->dt_in) )
            count++;
    }

    //last clk, total 25/26/27, set next convert parameter.
    int n = 1;
    if( dev->pulse_cnt==26 )
        n = 2;
    else if( dev->pulse_cnt==27 )
        n = 3;

    for (int i = 0; i < n; i++)
    {
        gpio_out_write(dev->sck_out, 1);
        hx71x_udelay(1);
        gpio_out_write(dev->sck_out, 0);
        hx71x_udelay(1);
    }

    //完成采样,计数+1.
    dev->sample_cnt++;

    count ^= 0x800000;
    return count;
}

//set weight tare.
void HX711_Get_WeightTare(struct hx71x_s* dev)
{
    dev->weight_tare = HX711_Read(dev);
}

//get weight
long HX711_Get_Weight(struct hx71x_s* dev)
{
    long value = HX711_Read(dev);

    return value - dev->weight_tare;
}

void
hx71x_query_task(void)
{
    if (!sched_check_wake(&s_Hx71x_Wake))
        return;

    uint8_t oid;
    struct hx71x_s *dev;
    foreach_oid(oid, dev, command_config_hx71x) {
        if( dev->sample_times<=0 || !(dev->flag&HX71X_SAMPLE_NOW))
            continue;

        dev->flag &= ~HX71X_SAMPLE_NOW;

        uint32_t next_waketime = dev->hx71x_timer.waketime;

        //读取并发送返回数据
        long weight = HX711_Get_Weight(dev);
        sendf("hx71x_state oid=%c value=%u cnt=%u next_clock=%u", oid, weight, dev->sample_cnt, next_waketime);
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