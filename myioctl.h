// support ioctl in driver

#define MYGPIO_IOC_MAGIC   'k'
#define NEWKIT_SET    _IOW(MYGPIO_IOC_MAGIC, 1, int)
#define NEWKIT_GET    _IOR(MYGPIO_IOC_MAGIC, 2, int)
#define MYGPIO_IOC_MAXNR 4 /* max 4 cmds here */

