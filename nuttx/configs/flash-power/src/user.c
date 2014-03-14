/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>
#include <time.h>
#include <signal.h>
#include <mqueue.h>
#include <nuttx/clock.h>

#include <nuttx/analog/adc.h>

#include "up_arch.h"
#include "stm3210e-internal.h"
#include "up_key.h"
#include "up_display.h"
#include "stm32_dac.h"

#define  TIM_FREQ   (10000)   // (72 000 000) / (7200)
#define  TIM_PERIOD (100)     // 10ms 

#define  KEY_PROCESS_MQ_NAME   ("cf_power/key_mq")
#define  T_FAN_ON   (55.0)
#define  T_FAN_OFF  (45.0)
#define  T_HOTDOWN  (80.0)

struct cali_data {
  uint16_t min;
  uint16_t max;
};

struct volt {
  uint32_t curr_volt;
  sem_t    sem;
};

static struct volt       g_currnt_volt;
static bool              g_force_fan_on = false;
static struct cali_data  g_calibrate_volt = {
  .min = 126,
  .max = 3988,
};
static struct cali_data  g_calibrate_curr = {
  .min = 0x68,
  .max = 0xCB1,
};

static void display_error(int idev) 
{
  int idx;
  idx = cf_display_buffer(idev,"ERR-",1);
  if (idx >= 0) {
    cf_display_lock(idev,idx,true);
  }
  return;
}

static inline uint32_t read_current_volt()
{
  return g_currnt_volt.curr_volt;
}

static int write_current_volt(uint32_t value)
{
  
  if (sem_wait(&g_currnt_volt.sem) != OK) {
    printf("write current volt failed(%d)\n",errno);
    display_error(1);
    return -errno;
  } else {
    if (0 <= value && value <= 3000 && ((value&0x1<<31) == 0) ) {
      g_currnt_volt.curr_volt = value;
    }
    sem_post(&g_currnt_volt.sem);
    return OK;
  }
}

static int display_bcd(int id,uint32_t value,bool dot,int count)
{
  uint8_t disp[4];

  
  disp[3] = value % 10;
  value = value / 10;
  disp[2] = value % 10;
  value = value / 10;
  disp[1] = value % 10;
  value = value / 10;
  disp[0] = value % 10;
  if (dot) {
    disp[1] |= 0x80;
  }
  return cf_display_buffer(id,disp,count);
}

void inline RL2V(bool on)
{
  if (on) {
    stm32_gpiowrite(GPIO_RL2V,1);
  } else {
    stm32_gpiowrite(GPIO_RL2V,0);
  }
}

void inline RL4V(bool on)
{
  if (on) {
    stm32_gpiowrite(GPIO_RL4V,1);
  } else {
    stm32_gpiowrite(GPIO_RL4V,0);
  }
}

void inline RL8V(bool on)
{
  if (on) {
    stm32_gpiowrite(GPIO_RL8V,1);
  } else {
    stm32_gpiowrite(GPIO_RL8V,0);
  }
}

void inline RL16V(bool on)
{
  if (on) {
    stm32_gpiowrite(GPIO_RL16V,1);
  } else {
    stm32_gpiowrite(GPIO_RL16V,0);
  }
}

void setRL(float value)
{
#if 1
  //close all RL
  RL2V(false);
  RL4V(false);
  RL8V(false);
  RL16V(false);
    
  value = (value / 1.2 ) + 2 + 2; //一个2V是整流桥消耗压降，另一个2V是增加的电压差，因为市电有波动，得留出足够的余量来保证正常稳压
    
  if(value >= 16) {
    value = value - 16;
    RL16V(true);
  }
  if(value >= 8) {
    value = value - 8;
    RL8V(true);
  }
  if(value >= 4) {
    value = value - 4;
    RL4V(true);
  }
  if(value >= 2) {
    value = value - 2;
    RL2V(true);
  }
#endif
}

static int dac_set_volt(float value)
{
  float temp;
  uint16_t output_value;

  //volt value on one lsb
  temp = (30.00 - 0.10) / (float)(g_calibrate_volt.max - g_calibrate_volt.min);

  if (value >= 0.1) {
    temp = (value - 0.100) / (float)temp;
    output_value = (uint16_t)temp + g_calibrate_volt.min;
  } else {
    output_value = (uint16_t)(value / temp);
  }
  if (output_value > 4095) {
    output_value = 4095;
  }

  putreg16(output_value,STM32_DAC_DHR12R1);

  setRL(value);

  return output_value;
}

void cf_timer_expiry(int signo, FAR siginfo_t *info, FAR void *context)
{
  
  cf_display_fresh();
  //cf_key_get();
}

int init_timer(void)
{
#if 0
  if (NULL == tim) {
    tim = stm32_tim_init(3);
    STM32_TIM_SETPERIOD(tim, TIM_PERIOD);
    STM32_TIM_SETCLOCK(tim, TIM_FREQ);
    STM32_TIM_SETMODE(tim, STM32_TIM_MODE_UP);
    STM32_TIM_DISABLEINT(tim,0);
    STM32_TIM_SETISR(tim,)
  }
#endif

  struct sigevent toevent;
  struct sigaction act;
  timer_t timer;
  struct itimerspec todelay;
  int ret;

  /* Create a POSIX timer to handle timeouts */

  toevent.sigev_notify          = SIGEV_SIGNAL;
  toevent.sigev_signo           = SIGALRM;
  toevent.sigev_value.sival_int = 0;

  ret = timer_create(CLOCK_REALTIME, &toevent, &timer);
  if (ret < 0)
    {
      printf("ERROR: Failed to create a timer: %d\n", errno);
      return -errno;
    }

  /* Attach a signal handler to catch the timeout */

  act.sa_sigaction = cf_timer_expiry;
  act.sa_flags     = SA_SIGINFO;
  sigemptyset(&act.sa_mask);

  ret = sigaction(SIGALRM, &act, NULL);
  if (ret < 0)
    {
      printf("ERROR: Failed to attach a signal handler: %d\n", errno);
      return -errno;
    }

  /* set timer interval*/
  todelay.it_interval.tv_sec    = 0;  
  todelay.it_interval.tv_nsec   = 1000*1000*10;
  todelay.it_value.tv_sec       = 0;
  todelay.it_value.tv_nsec      = 1000*1000*10;

  ret = timer_settime(timer, 0, &todelay, NULL);
  if (ret < 0)
    {
      printf("ERROR: Failed to set the timer: %d\n", errno);
      return -errno;
    }

  return OK;

}

static int cf_temp_current_task(int argc, char* argv[])
{
  int fd_temp=0, fd_current=0,i;
  struct adc_msg_s temperature[10],current[10];
  float temp_value;
  float curr_value;
  float bit_curr_value;
  uint32_t sum_temp=0, sum_curr=0;
  uint16_t drift;  //compensation value

  printf("cf_temp_current_task:start\n");
  fd_temp = open("/dev/adc2",O_RDONLY);
  if (fd_temp < 0) {
    printf("cf_temp_current_task:open /dev/adc2 failed(%d)\n",errno);
    display_error(1);
    return -errno;
  }

  fd_current = open("/dev/adc1",O_RDONLY);
  if (fd_current < 0) {
    printf("cf_temp_current_task:open /dev/adc1 failed(%d)\n",errno);
    display_error(1);
    return -errno;
  }

  while(1) {

    usleep(1000*800);
    read(fd_temp,&temperature,sizeof(temperature));
    read(fd_current,&current,sizeof(current));
    
    for (i=0; i<10; i++) {
      sum_temp += temperature[i].am_data & 0xFFF;
      sum_curr += current[i].am_data & 0xFFF;
        // printf("(%d,%d)\n",(temperature[i].am_data & 0xFFF),
                        // (current[i].am_data & 0xFFF));
    }
    sum_temp /= 10;
    sum_curr /= 10;

    temp_value = (float)sum_temp * (250.0/4096);

    if (temp_value >= T_HOTDOWN) {
      /*shutdown system*/
      /* TODO ADD logic*/
    } else if (temp_value >= T_FAN_ON) {
      stm32_gpiowrite(GPIO_FAN,1);
    } else {
      if (g_force_fan_on == false) {
        stm32_gpiowrite(GPIO_FAN,0);
      }
    }

    // how many current in one bit
    bit_curr_value = (4.0 - 0.1) / (g_calibrate_curr.max - g_calibrate_curr.min);
    drift = (uint16_t)(0.1 / bit_curr_value);
    if (g_calibrate_curr.min > drift) {
      drift = g_calibrate_curr.min - drift;
      if (sum_curr > drift) {
        curr_value = (sum_curr - drift) * bit_curr_value;
      } else {
        curr_value = 0;
      }
    } else {
      drift = drift - g_calibrate_curr.min;
      if (sum_curr > 10) {
        curr_value = (sum_curr + drift) * bit_curr_value;
      } else {
        curr_value = 0;
      }
    }
    
    cf_display_buffer(1,"TTTT",100);
    display_bcd(1,temp_value*100,true,100);
    cf_display_buffer(1,"AAAA",100);
    display_bcd(1,curr_value*1000,true,100);  
    sum_temp = 0;
    sum_curr = 0;
  }

}

#define  CALI_LONG_PRESS_CNT  (20)
#define  CALI_CTRL_PRESS_CNT  (7)

static int cf_user_key_task(int argc, char* argv[])
{
  mqd_t           mq_key;
  struct mq_attr  attr;
  uint8_t         key = 0;
  uint32_t        curr_volt;
  int             lock_idx;
  static int      ctrl_cnt=0, fn_ctrl_cnt=0;

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = 1;
  attr.mq_flags   = 0;

  mq_key = mq_open(KEY_PROCESS_MQ_NAME,O_RDONLY|O_CREAT,0666, &attr);
  if (mq_key == (mqd_t)ERROR) {
    printf("create mqueue for key process failed(%d)\n",errno);
    display_error(0);
    return -errno;
  }

  if (cf_key_mq_add(KEY_PROCESS_MQ_NAME) == (mqd_t)ERROR) {
    printf("add mqueue into link failed\n");
    display_error(0);
    return ERROR;
  }

  while (1) {
    mq_receive(mq_key,&key,1,NULL);

    switch(key) {
    
    /*OUTPUT Enalbe and disable*/
    case OUTPUT_KEY:
      //disable output
      stm32_gpiowrite(GPIO_OUT_EN,1);
      break;
    case FN_KEY|OUTPUT_KEY:
      // enable output
      stm32_gpiowrite(GPIO_OUT_EN,0);
      break;

    /*Up,Down inc or dec output volt value*/
    case UP_KEY:
      // output +1V
      curr_volt = read_current_volt();
      curr_volt += 100;
      if (write_current_volt(curr_volt) != OK) {
        printf("write current volt value failed\n");
        break;
       }
      dac_set_volt((float)curr_volt / 100.0);
      lock_idx = display_bcd(0,read_current_volt(),true,100);
      cf_display_lock(0,lock_idx,true);
      break;
    case FN_KEY|UP_KEY:
      // output +0.1V
      curr_volt = read_current_volt();
      curr_volt += 10;
      if (write_current_volt(curr_volt) != OK) {
        printf("write current volt value failed\n");
        break;
      }
      dac_set_volt((float)curr_volt / 100.0);
      lock_idx = display_bcd(0,read_current_volt(),true,100);
      cf_display_lock(0,lock_idx,true);
      break;
    case DOWN_KEY:
      // output -1V
      curr_volt = read_current_volt();
      curr_volt -= 100;
      if (write_current_volt(curr_volt) != OK) {
        printf("write current volt value failed\n");
        break;
      }
      dac_set_volt((float)curr_volt / 100.0);
      lock_idx = display_bcd(0,read_current_volt(),true,100);
      cf_display_lock(0,lock_idx,true);
      break;
    case FN_KEY|DOWN_KEY:
      // output -0.1V
      curr_volt = read_current_volt();
      curr_volt -= 10;
      if (write_current_volt(curr_volt) != OK) {
        printf("write current volt value failed\n");
        break;
      }
      dac_set_volt((float)curr_volt / 100.0);
      lock_idx = display_bcd(0,read_current_volt(),true,100);
      cf_display_lock(0,lock_idx,true);
      break;

    
    case CTRL_KEY:
      break;

    /*Calibration logic*/
    /* 1.Long press FN_KEY + CTRL_KEY*/
    /* 2. press 7 times CTRL_KEY*/
    case FN_KEY|CTRL_KEY:
      fn_ctrl_cnt++;
      if (fn_ctrl_cnt >= CALI_LONG_PRESS_CNT) {
        /*count 7 times CTRL_KEY*/
        printf("CTRL_FN\n");
        lock_idx = cf_display_buffer(1,"CALI",1);
        cf_display_lock(1,lock_idx,true);
        while (1) {
          mq_receive(mq_key,&key,1,NULL);
          if (key == CTRL_KEY) {
            printf("CTRL++\n");
            ctrl_cnt++;
            /*continue count CTRL_KEY, do not do anything else*/
            if (ctrl_cnt < CALI_CTRL_PRESS_CNT) {
              continue; 
            }
          } else if (key == (CTRL_KEY|FN_KEY) | key == FN_KEY) {
            continue;
          } else {
            /* Any key other CTRL_KEY would break out calibration*/
            fn_ctrl_cnt = 0;
            ctrl_cnt = 0;
            printf("break out calibration\n");
            break;
          }
          if (ctrl_cnt >= CALI_CTRL_PRESS_CNT) {
            /*do calibration*/
            /* Display 'cali' on disp0 to tell user enter calibration mode*/
            uint16_t dac_output = 0;
            printf("Enter cali\n");
            cf_display_buffer(0,"CALI",200);
            /*calibrate 0.1V*/
            lock_idx = cf_display(1,0x00018000,1);
            cf_display_lock(1,lock_idx,true);
            RL2V(true);
            RL4V(false);
            RL8V(false);
            RL16V(false);
            stm32_gpiowrite(GPIO_OUT_EN,0); //enable output
            while (1) {
              mq_receive(mq_key,&key,1,NULL);
              if (key == CTRL_KEY) {
                /*DAC output value +1*/
                if ((dac_output+1) <= 0xFFF) {
                  dac_output++;
                  printf("+1\n");
                  putreg16((dac_output)&0xFFF,STM32_DAC_DHR12R1);
                  lock_idx = display_bcd(0,dac_output,false,100);
                  cf_display_lock(0,lock_idx,true);
                }
              }
              if (key == (FN_KEY|CTRL_KEY)) {
                /*DAC output value -1*/
                if ( dac_output != 0 ) {
                  dac_output --;
                  printf("-1\n");
                  putreg16((dac_output)&0xFFF,STM32_DAC_DHR12R1);
                  lock_idx = display_bcd(0,dac_output,false,100);
                  cf_display_lock(0,lock_idx,true);
                }
              }
              if (key == (OUTPUT_KEY|UP_KEY)) {
                /*break out, calibrate 30V*/
                g_calibrate_volt.min = dac_output;
                break;
              }
            }

            //calibrate 30V
            RL2V(true);
            RL4V(true);
            RL8V(true);
            RL16V(true);
            lock_idx = cf_display(0,0x00008003,1);
            cf_display_lock(0,lock_idx,true);
            while (1) {
              mq_receive(mq_key,&key,1,NULL);
              if (key == CTRL_KEY) {
                /*DAC output value +1*/
                if ((dac_output+1) <= 0xFFF) {
                  dac_output++;
                  printf("+1\n");
                  putreg16((dac_output)&0xFFF,STM32_DAC_DHR12R1);
                  lock_idx = display_bcd(0,dac_output,false,100);
                  cf_display_lock(0,lock_idx,true);
                }
                
              }
              if (key == (FN_KEY|CTRL_KEY)) {
                /*DAC output value -1*/
                if ( dac_output != 0 ) {
                  dac_output --;
                  putreg16((dac_output)&0xFFF,STM32_DAC_DHR12R1);
                  lock_idx = display_bcd(0,dac_output,false,100);
                  cf_display_lock(0,lock_idx,true);
                }
                
              }
              if (key == (OUTPUT_KEY|UP_KEY)) {
                /*break out, calibrate 30V*/
                g_calibrate_volt.max = dac_output;
                break;
              }
            }
          }
          printf("finish cali\n");
          break;
        }
      }
      break;

    default:
      break;
    }
  }

}

void calibrate()
{

}

static cf_disp_key_task(int argc, char **argv)
{
   while(1) {
    usleep(1000*10);
    cf_display_fresh();
    cf_key_get();
  }
}

 int cf_main(int argc, char **argv)
{
  sem_init(&g_currnt_volt.sem,0,1);
  task_create("key",SCHED_PRIORITY_DEFAULT,1024,cf_user_key_task,NULL);
  task_create("CATA",SCHED_PRIORITY_DEFAULT,1024,cf_temp_current_task,NULL);
  task_create("timer",SCHED_PRIORITY_DEFAULT,1024,cf_disp_key_task,NULL);
  //init_timer();
  
  // bring up ficl
  ficl_main(argc,argv);
}
