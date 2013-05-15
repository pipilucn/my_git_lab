/*
 *  Header file for mygpio driver 
 *  Written by Leo April 2, 2013 in BBT college 
 */

#include "myioctl.h"
//#define MYGPIO_IOC_MAGIC   'k'
//#define NEWKIT_SET    _IOW(MYGPIO_IOC_MAGIC, 0, int)
//#define NEWKIT_GET    _IOR(MYGPIO_IOC_MAGIC, 1, int)
//#define MYGPIO_IOC_MAXNR 4 /* max 4 cmds here */

MODULE_LICENSE("Dual BSD/GPL");

//#define MYGPIO_POLLING /* default interrupt mode, for using polling mode*/
#define MYGPIO_DEBUG

#define GPIOC_DIRECTION_REG 0x10000094 /* input, output mode set register for port C */
//#define GPIOC_DIRECTION_REG 0x1000008c /* input, output mode set register for port B */

#define RINGBUFFER_SIZE 8 /* size of buffer */  

#define INT6_VEC_NUM 91 /* interrupt vector number of INT 6*/
#define ICR4_INT6_ADDR (0x10000000 + 0x2c + 1) /*address of ICR4 for INT 6*/
//#define INT1_VEC_NUM 68 /* interrupt vector number of INT 4*/
//#define ICR1_INT4_ADDR (0x10000000 + 0x20 + 1) /*address of ICR1 for INT 4*/

const int GPIODRV_MAJOR = 238;
const int GPIODRV_MINOR = 0;
const unsigned int int6_vec_num = INT6_VEC_NUM;
int KIT = 5; /* key interval time, check input key every 0.02s */


typedef volatile struct 
{
    u16 gpioc_direction_reg;
    u16 gpioc_data_reg;
} gpio;

#define GPIOC_BASE GPIOC_DIRECTION_REG
#define GPIOC_SIZE sizeof(my_gpio)

struct GPIO_DEV 
{
    /* ring buffer */ 
    u8 header;
    u8 tail;
    u8 ringbuffer[RINGBUFFER_SIZE];
    u8 count;
    struct cdev char_dev;/* char device to be registered to kernel*/     
    wait_queue_head_t inq_wait;/* synchronization */
    struct semaphore mutex;
    struct timer_list timer; /* for polling mode */
    /*struct class *dev_class;
    struct device *mydev;*/
} mygpio_dev;

