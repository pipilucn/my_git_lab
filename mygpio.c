/*
 *  A simple driver of GPIO on M5272 board, as input/output device
 *  with major (238), that takes PC12-15 as input, 8-9 as output
 *  
 *  Written by Leo April 2, 2013 in BBT college 
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/uio.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <asm/system.h>
/*#include <linux/workqueue.h>*/
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <linux/poll.h>

#include "mygpio.h"

#ifdef MYGPIO_DEBUG

/*
 * The proc filesystem: function to read /proc/mygpioregs 
 */
int mygpio_read_procmem(char *buf, char **start, off_t offset,
                   int count, int *eof, void *data)
{
    gpio *pgpio = NULL;
    int len = 0;
    volatile char icr4;
    volatile char ddr;

    pgpio = (gpio*)GPIOC_BASE;        

    if (down_interruptible(&mygpio_dev.mutex))
        return -ERESTARTSYS;

    icr4 = *((char*)ICR4_INT6_ADDR);
    ddr  = inb(&pgpio->gpioc_direction_reg);/**((char*))(0x10000094);*/;
    len += sprintf(buf+len,"ICR4:0x%x,  Data Direction Register: 0x%x\n",icr4,ddr);

    up(&mygpio_dev.mutex);

    *eof = 1;

    return len;
}

/*
 * Actually create (and remove) the /proc file(s).
 */

static void mygpio_create_proc(void)
{
    struct proc_dir_entry *entry;
    printk(KERN_INFO "I'm creating /proc..\n");    
    entry = create_proc_read_entry("mygpioregs", 0 /* default mode */,
                    NULL /* parent dir */, mygpio_read_procmem,
                    NULL /* client data */);
    printk(KERN_INFO "done\n");    
}

static void mygpio_remove_proc(void)
{
        /* no problem if it was not registered */
    remove_proc_entry("mygpioregs", NULL /* parent dir */);
}


#endif
static void print_ringbuffer(void)
{
    u8 i;

    for (i=0; i<RINGBUFFER_SIZE; i++){
       printk(KERN_INFO "\nringbuffer[%i] = 0x%x", i,mygpio_dev.ringbuffer[i]);   
       if (i == mygpio_dev.tail) {
            printk(KERN_INFO "   <-- tail ");
       } else if ( i == mygpio_dev.header) {
            printk(KERN_INFO "  <-- header ");
       }
    }

    printk(KERN_INFO"\ncount= %i\n", mygpio_dev.count);
    printk(KERN_INFO "----------------------\n");
}

/* When the ring buffer is full, we just discard the 
   new coming data
 */
static void write_ringbuf(u8 key)
{
    /* We don't need mutex here, because this is in the ISR context */
    if (mygpio_dev.count < RINGBUFFER_SIZE) {
        mygpio_dev.ringbuffer[mygpio_dev.tail++] = key;
        mygpio_dev.count++; 
        mygpio_dev.tail %= RINGBUFFER_SIZE;
    } else {
        printk(KERN_NOTICE"ring buffer is full, discard new key now");
   }
}

static u8 read_key(void)
{
    gpio *pgpio;
    u8 ch;

    /* we don't need mutex here so far, in ISR context*/
    pgpio = (gpio*)GPIOC_BASE;        
    ch = inb(&pgpio->gpioc_data_reg);
    return ch;    
}

static void save_key(u8 key)
{
    if (key != 0xff) { /* There is data, we need to turn on relavant led to show it*/
       write_ringbuf(key);
       print_ringbuffer();

       wake_up_interruptible(&mygpio_dev.inq_wait);
    } else {
       printk(KERN_INFO "press key [0-3], and do it ASAP after press IRQ3\n");
    } 
}

static void  mygpio_bf_work_handler(struct work_struct *dummy)
{
    /*save_key(key);*/
}

static DECLARE_WORK(mygpio_bf_work, mygpio_bf_work_handler);

/* 
   reset INT6  
   format: 23 | 22 21 20 | <- bits
           ^  <----------> <- priority level: 111, the highest level  
           |     
       pending bit 1: reset (clear) interrupt happened 
   INT 6: ICR4 (0x1000002d)bit 23-20 the highest priority 
 */
static void reset_init(void)
{
    /* reset external interrupt 6, we don't need mutex because of ISR */    
    //volatile char c1 = inb(ICR4_INT6_ADDR);
    volatile char c1 = *((char*)ICR4_INT6_ADDR);
    char c2 = 0xf0 | c1; // set higher 4 bits 1111, keep low 4 bits as before      
    outb(c2, ICR4_INT6_ADDR); /* reset interrupt, i.e. clear pending bits*/
}

static irqreturn_t mygpio_isr(int irq, void *dev_id/*, struct pt_regs *regs*/)
{
    u8 key;

    printk(KERN_INFO "Enterring in mygpio_irs\n");
    local_irq_disable(); /* fast interrupt */
    reset_init();

    /* read data and save */
    key = read_key();
    save_key(key);
   
    local_irq_enable();
    /*shedule_work(&mygpio_bf_work);*/ 
    
    wake_up_interruptible(&mygpio_dev.inq_wait);
    printk(KERN_INFO "Leaving mygpio_irs\n");

    return IRQ_HANDLED;  
}

static int mygpio_start(void)
{ 
    gpio *pgpio = (gpio*)GPIOC_BASE;        
    static bool first = true;
    
    printk(KERN_INFO "All IRQs are disabled for reset init\n");
    local_irq_disable(); /* turn off all interrupts*/
    reset_init();

    if (first) {
        if (request_irq(int6_vec_num, mygpio_isr,IRQF_DISABLED, "mygpio_int_dev_name", 
                       (void*)&mygpio_dev)) {
            printk(KERN_ALERT "unable to register interrupt handler: %d\n", int6_vec_num);
            return -1;
        }

        /* mutex with read from /proc/mygpioregs */
        if (down_interruptible(&mygpio_dev.mutex))
           return -ERESTARTSYS;

        outb(0xf0,&pgpio->gpioc_direction_reg); /* Set PC0-4 input, PC5-8 output*/
        outb(0xff,&pgpio->gpioc_data_reg); /*turn off all leds, important, don't remove*/
        // *((char*)0x10000094) = 0xf0;    /* same as above, verified */
        // *((char*)0x10000096) = 0xff;    

        up(&mygpio_dev.mutex);

        first = false;
    }
    
    local_irq_enable();
    printk("All IRQs are enabled again\n");

    return 0;  
}

/* inode represents a file in system internally,
   flip represents an open file.  
   The difference lies in that mutiple open files point to
   a signle inode.
 */
static int mygpio_open(struct inode *inode, struct file *flip)
{
    struct GPIO_DEV *dev;
   
    /* i_cdev is the real guy, its type is char_dev, the type of container
       is struct GPIO_DEV, the purpose is to get the address of container 
       so that the buffer can be operated as well.
     */ 
    dev = container_of(inode->i_cdev, struct GPIO_DEV, char_dev); 
    flip->private_data = (void*)dev; /* save it to flip*/    
   
    /*do we really need this? */ 
    if (down_interruptible(&dev->mutex))
        return -ERESTARTSYS;

    dev->header = 0;
    dev->tail = 0;
    dev->count = 0;
    init_waitqueue_head(&dev->inq_wait);   

    up(&dev->mutex); 
#if !defined(MYGPIO_POLLING)
    mygpio_start(); /* working mode: interrupt */
#else
    mygpio_polling(); 
#endif
    
    printk(KERN_INFO "Open ok\n");
    return 0;
}

static int mygpio_release(struct inode *inode, struct file *flip)
{
    printk(KERN_NOTICE "I'm closed\n"); 
    return 0;
}

static ssize_t mygpio_read(struct file *flip, char __user *buf, size_t count, loff_t *f_pos)
{
    u8 total = 0;
    // u8 i = 0;
    struct GPIO_DEV *dev = NULL;
    dev  = (struct GPIO_DEV *)flip->private_data;    
    
    printk(KERN_NOTICE "mygpio_read starts \n");

    if (dev->count == 0) { // empty buffer
        printk(KERN_NOTICE "empty buffer, reader is sleeping.. \n");
        
        if(flip->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
 
        wait_event_interruptible(dev->inq_wait,dev->count);// there is data
        printk(KERN_NOTICE "reader is waken up\n");        
    }

    total = (count < dev->count) ? count : dev->count;    
    printk(KERN_NOTICE "total keys in buffer: %d\n", total);

    /*
    for (i = 0 ;i < total; i++) {
        copy_to_user(buf+i, (void*)&dev->ringbuffer[dev->header],1);
        //put_user(dev->ringbuffer[dev->header], buf+i); // bettter, faster
    
        dev->header = (dev->header+1) % RINGBUFFER_SIZE;
        dev->count--;       
    }
   */

    printk(KERN_NOTICE "start copy to user..\n");
    //copy_to_user(buf, (void*)&dev->ringbuffer[dev->header],1); // copy only one byte here
    put_user(dev->ringbuffer[dev->header],buf);
 
    if (down_interruptible(&dev->mutex))
        return -ERESTARTSYS;

    dev->ringbuffer[dev->header] = 0; // delete the key transferred
    dev->header = (++dev->header) % RINGBUFFER_SIZE;   
    dev->count--;

    up(&dev->mutex);
    printk(KERN_NOTICE "done\n");
    return 1; // 1 byte, must math the exact data copied, or repeated calling
    //return total;
}

/*
 * Different from mygpio_read, we write the data to the gpio directly 
 * instead of save it in ring buffer 
*/
static ssize_t mygpio_write(struct file *flip, const char __user *buff, size_t count, loff_t *f_pos)
{
    char chr;
    gpio *pgpio = (gpio*)GPIOC_BASE;

    struct GPIO_DEV *dev = (struct GPIO_DEV *)flip->private_data;

    printk(KERN_NOTICE "I'm in mygpio_write..\n");
    printk(KERN_NOTICE "start copy from user\n");
    //copy_from_user((void*)&chr, buff, 1);
    get_user(chr,buff);
    printk(KERN_NOTICE "done\n");

    if (down_interruptible(&dev->mutex))
        return -ERESTARTSYS;

    outb(chr,&pgpio->gpioc_data_reg); /* write directly without in ring buffer*/  

    up(&dev->mutex);
    return 1; // 1 bytes, consistent with real result, or repeating call mygpio_write
}

/*
 * for how to pass and return parameters, refer to scull/main.c
 */

static int mygpio_ioctl(struct inode *inode, struct file *flip, unsigned int cmd, unsigned long arg) 
/*static long mygpio_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)*/
{
    int err = 0;
    long retval = 0;

    printk(KERN_INFO "in mygpio_ioctl now\n");
    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (_IOC_TYPE(cmd) != MYGPIO_IOC_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd) > MYGPIO_IOC_MAXNR) return -ENOTTY;

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
           err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
           err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (err) return -EFAULT;

    switch (cmd) {
    case NEWKIT_SET : // set new KIT, we don't consider real argument so far
         if (! capable (CAP_SYS_ADMIN))
                return -EPERM;
         retval = __get_user(KIT, (int __user *)arg);
         printk(KERN_INFO "in mygpio_ioctl: new polling period is: %d\n", KIT);
         break;

    case NEWKIT_GET : // get new KIT, a dummy here
         retval = __put_user(KIT, (int __user *)arg);
         printk(KERN_INFO "in mygpio_ioctl: get new KIT\n");
         break;
    default:
         break;
    }

    return 0;
}

/* How much space is free? */
static int spacefree(struct GPIO_DEV *dev)
{
    if (dev->header == dev->tail)
         return RINGBUFFER_SIZE - 1;
    return ((dev->tail + RINGBUFFER_SIZE - dev->header) % RINGBUFFER_SIZE) - 1;
}

static unsigned int mygpio_poll(struct file *filp, poll_table *wait)
{
    struct GPIO_DEV *dev = filp->private_data;
    unsigned int mask = 0;

    printk(KERN_INFO "in mygpio_poll now\n");
    /*
     * The buffer is circular; it is considered full
     * if "tail" is right behind "header" and empty if the
     * two are equal.
     */
    down_interruptible(&dev->mutex);
    poll_wait(filp, &dev->inq_wait, wait);  /* add inq_wait to kernel poll table*/
    if (dev->header != dev->tail)
            mask |= POLLIN | POLLRDNORM;    /* readable */
    if (spacefree(dev))
            mask |= POLLOUT | POLLWRNORM;   /* writable */
    up(&dev->mutex);
    return mask;
}

static struct file_operations mygpio_fops = 
{
    .open    =  mygpio_open,
    .release =  mygpio_release,
    .read    =  mygpio_read,
    .write   =  mygpio_write,
    /*.unlocked_ioctl   =  mygpio_unlocked_ioctl,*/ /* 3.2? */
    .ioctl   =  mygpio_ioctl, /* 2.6*/
    .poll    =  mygpio_poll,
    .owner   =  THIS_MODULE, 
};

static int mygpio_init(void)
{
    dev_t devno; /* used to hold device numbers */
    int err;

    printk(KERN_INFO "hello, mygpio_init\n");
    
    devno = MKDEV(GPIODRV_MAJOR, GPIODRV_MINOR);
    cdev_init(&mygpio_dev.char_dev, &mygpio_fops); /* allocate,initialize cdev: char_dev */   
    mygpio_dev.char_dev.owner = THIS_MODULE;

    /* To get real device number, devno is the first number desired, 
       we don't need more than 1 device number for our gpio char device
       the last is name of the device or driver
       If you are sure the major is not used, below is -NOT necessary 
     */
    err = register_chrdev_region(devno,1,"gpio"); /* To get major */      
    if (err) {
        printk(KERN_NOTICE "Can't get the major number %d as requested in gpio\n", GPIODRV_MAJOR);
        goto fail_get_major;
    }    
   
    err = cdev_add(&mygpio_dev.char_dev, devno, 1);
    if (err) {
        printk(KERN_NOTICE "Can't register gpio with major number %d to the system\n", GPIODRV_MAJOR);
        goto fail_register;
    }    

#ifdef MYGPIO_DEBUG
   init_MUTEX(&mygpio_dev.mutex);
   mygpio_create_proc(); 
#endif
 
    /* write_test(); */ 
    printk(KERN_INFO "gpio init ok\n");
    return 0;

    fail_register:
       unregister_chrdev_region(devno,1); 

    fail_get_major:
       /* nothing special need to do here, you believe it */
  
    return err;
}	

static void mygpio_exit(void)
{
    cdev_del(&mygpio_dev.char_dev);
    unregister_chrdev_region(MKDEV(GPIODRV_MAJOR,GPIODRV_MINOR),1); 
    
#if defined MYGPIO_POLLING
    del_timer_sync(&mygpio_dev.timer);
#else
    free_irq(int6_vec_num,&mygpio_dev); 
#endif

#ifdef MYGPIO_DEBUG 
    mygpio_remove_proc();
#endif

    printk(KERN_INFO "bye, mygpio_exit\n");
}

module_init(mygpio_init);
module_exit(mygpio_exit);
