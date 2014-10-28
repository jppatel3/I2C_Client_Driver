/* ----------------------------------------------- DRIVER i2c_flash ---------------------------------------------*/
//---------------------------------------------------------------------------------------------------------------
/* Included Libraries */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/delay.h>
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>
#include <linux/delay.h>
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* GPIO Definations */

#define OnBoardLED 	3	// LED On Galileo
#define OnBoardIO8 	26	// Read/Write Busy LED
#define I2CMUX	 	29 	// GPIO for selection of SCLK and SDA
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
MODULE_LICENSE("GPL");
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
#define PAGE_LENGTH 	64		// Number of bytes in one page of memory
#define DEVICE		"I2C_Flash"  	// device input queue to be created and registered
#define MAJOR_NUMBER 	333		// Defining The Major Number
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* IOCTL Definations */
#define IOCTL_ERASE_FLASH  _IO(MAJOR_NUMBER, 0)
#define IOCTL_FLASHGETRP   _IOR(MAJOR_NUMBER, 1, long)
#define IOCTL_FLASHSETRP   _IOW(MAJOR_NUMBER, 2, long)
#define IOCTL_FLASHGETWP   _IOR(MAJOR_NUMBER, 3, long)
#define IOCTL_FLASHSETWP   _IOW(MAJOR_NUMBER, 4, long)
#define IOCTL_FLASHGETRS   _IOR(MAJOR_NUMBER, 5, long)

//---------------------------------------------------------------------------------------------------------------
extern int errno;
int errno = EPERM;
//---------------------------------------------------------------------------------------------------------------
/* Driver structure defination */
struct i2c_flash_dev{
	struct i2c_adapter *Device_adapter;	// Adapter to which client is connected
	struct cdev cdev;			// Device structure
	u8 Client_Address;			// Address of Client
	int Current_Page_Write;			// EEPROM current page write pointer
	int Current_Page_Read;			// EEPROM current page read pointer
	int EEPROM_BUSY;			// Status of Busy/Free EEPROM
} *i2c_flash_devp;

int DATA_READY;

struct class *i2c_flash_dev_class;          	// Tie with the device model 
static struct device *i2c_flash_dev_device;
static dev_t i2c_flash_dev_number;

/* Message structure */
struct epprom_msg{
	u8 Address[2];
	u8 user_message[64];
};
/* Work Queue */
static struct workqueue_struct *my_wq;
typedef struct {
	struct work_struct my_work;
	int count;
	struct i2c_flash_dev *i2c_flash_devp;
	u8 *Kernel_Message_Pointer;
} my_work_t;
my_work_t *work, *work2;
//---------------------------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------------------------
/* Function for writing Page to EEPROM */
static int Write_Page_EEPROM(struct epprom_msg *epprom_msgp, u8 client_addr,struct i2c_adapter *Used_Adapter, u16 Page_Number)
{
	int ret = 0;
	struct i2c_msg *msg;

	epprom_msgp->Address[0] = (Page_Number>>2);
	epprom_msgp->Address[1] = (Page_Number<<6);

	//printk(KERN_ALERT"Address Byte 0 : %x",epprom_msgp->Address[0]);
	//printk(KERN_ALERT"Address Byte 1 : %x",epprom_msgp->Address[1]);

	/* Sending Write Address and Data */	
	msg = kmalloc(sizeof(struct i2c_msg),GFP_KERNEL);
	msg ->addr 	= client_addr;
	msg ->flags 	= 0;
	msg ->len 	= PAGE_LENGTH + 2; //2 extra for Address bytes
	msg ->buf 	= (u8 *)epprom_msgp;

	ret = i2c_transfer(Used_Adapter,msg,1);
	if(ret < 0)
	{
		//printk(KERN_ALERT"Transfer Failed\n");
		kfree(msg);
		return 1;
	}
	kfree(msg);
	return 0;	
}
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* Function for Reading Page from EEPROM */
static int Read_Page_EEPROM(struct epprom_msg *epprom_msgp, u8 *buf, u8 client_addr,struct i2c_adapter *Used_Adapter, u16 Page_Number)
{
	int ret = 0;
	struct i2c_msg *msg;

	epprom_msgp->Address[0] = (Page_Number>>2);
	epprom_msgp->Address[1] = (Page_Number<<6);

	//printk(KERN_ALERT"Address Byte 0 : %x",epprom_msgp->Address[0]);
	//printk(KERN_ALERT"Address Byte 1 : %x",epprom_msgp->Address[1]);


	/* Sending Read Address */
	msg = kmalloc(sizeof(struct i2c_msg),GFP_KERNEL);
	msg ->addr 	= client_addr;
	msg ->flags 	= 0;
	msg ->len 	= 2;
	msg ->buf 	= (u8 *)epprom_msgp;
	ret = i2c_transfer (Used_Adapter,msg,1);
	if(ret < 0)
	{
		//printk(KERN_ALERT"Transfer Failed\n");
		return -1;
	}
	kfree(msg);

	/* Recieving Data */
	msg = kmalloc(sizeof(struct i2c_msg),GFP_KERNEL);
	msg->addr = client_addr;
	msg->flags= 1;
	msg-> len = PAGE_LENGTH;
	msg-> buf = buf;
	ret = i2c_transfer(Used_Adapter,msg,1);
	if(ret < 0)
	{
		//printk(KERN_ALERT"Transfer Failed\n");
		return 1;
	}
	kfree(msg);
	
	return 0;
}
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* Function for Erasing Page to EEPROM */
static int Erase_Page_EEPROM(u8 client_addr, struct i2c_adapter *Used_Adapter, u16 Page_Number)
{
	int ret = 0;
	struct i2c_msg *msg = kmalloc(sizeof(struct i2c_msg),GFP_KERNEL);
	struct epprom_msg *epprom_msgp = kmalloc(sizeof(struct epprom_msg),GFP_KERNEL);

	epprom_msgp->Address[0] = (Page_Number>>2);
	epprom_msgp->Address[1] = (Page_Number<<6);
	memset(epprom_msgp->user_message, 0xFF, sizeof(epprom_msgp->user_message));

	/* Sending Write Address and Data */	
	msg ->addr = client_addr;
	msg ->flags = 0;
	msg ->len = PAGE_LENGTH + 2; //2 extra for Address bytes
	msg ->buf = (u8 *)epprom_msgp;
	ret = i2c_transfer(Used_Adapter,msg,1);
	if(ret < 0)
	{
		//printk(KERN_ALERT"Transfer Failed\n");
		kfree(epprom_msgp);
		kfree(msg);
		return 1;
	}
	kfree(epprom_msgp);
	kfree(msg);
	return 0;	
}
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* Open */
int i2c_flash_open(struct inode *inode, struct file *file)
{
	struct i2c_flash_dev *i2c_flash_devp;

	i2c_flash_devp = container_of(inode->i_cdev, struct i2c_flash_dev, cdev);
	file->private_data =i2c_flash_devp;

	/* Attaching Present I2C Adapter to my Driver */ 	
	i2c_flash_devp->Device_adapter = i2c_get_adapter(0);

	/* Set Client Address */
	i2c_flash_devp->Client_Address = 0x54;

	/* Currrent Page for Read and Write */

	i2c_flash_devp->Current_Page_Write = 0;
	i2c_flash_devp->Current_Page_Read  = 0;
	DATA_READY  = 0;
	i2c_flash_devp->EEPROM_BUSY =0;


	printk(KERN_ALERT"Get Set Go\n");
	return 0;
}
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
/* Close */
int i2c_flash_release(struct inode *inode, struct file *file)
{
	printk("File release does nothing that important for now!!! \n");
	return 0;
}
//---------------------------------------------------------------------------------------------------------------
// Write EEPROM 
static void Write_EEPROM(struct work_struct *work){

	struct epprom_msg *send_message = kmalloc(sizeof(struct epprom_msg),GFP_KERNEL);
	int i =0;
	int res = 0;
	int curr_page;

	my_work_t *my_work = (my_work_t *)work;
	
	curr_page = my_work->i2c_flash_devp->Current_Page_Write;

	gpio_set_value_cansleep(OnBoardIO8,1);	//Setting IO8 to 1
	my_work->i2c_flash_devp->EEPROM_BUSY = 1;	//Setting Status to Bus

	for(i = 0;i< my_work->count;i++)
	{
		memcpy(send_message->user_message,my_work->Kernel_Message_Pointer+(i*PAGE_LENGTH), PAGE_LENGTH);
		/*EEPROM Write Function*/
		res = 1;
		while(res == 1){
			res = Write_Page_EEPROM(send_message,my_work->i2c_flash_devp->Client_Address,my_work->i2c_flash_devp->Device_adapter,(u16)curr_page);
		}
		/* Incrementing current write pointer */
		if(curr_page < 511){
			curr_page = curr_page + 1;
		}
		else{
			curr_page = 0;
		}
	}
	my_work->i2c_flash_devp->Current_Page_Write = curr_page;

	my_work->i2c_flash_devp->EEPROM_BUSY = 0;	//Setting Status to Bus
	gpio_set_value_cansleep(OnBoardIO8,0); 	//Setting IO8 to 0

	kfree(send_message);
	kfree(my_work->Kernel_Message_Pointer);

}
//---------------------------------------------------------------------------------------------------------------
/* Write */
ssize_t i2c_flash_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{

	struct i2c_flash_dev *i2c_flash_devp = file->private_data;

	u8 *Kernel_Message_Pointer = kmalloc(64*count,GFP_KERNEL);	// Message pointer in kernel
	u8 *User_Data_Pointer;						// Message pointer in userspace
	u8 *Data_Pointer;

	int res;
	
	User_Data_Pointer = (u8*)buf;
	Data_Pointer = Kernel_Message_Pointer;

	/* Copying data from user space to kernel space */
	res = copy_from_user(Kernel_Message_Pointer,User_Data_Pointer,PAGE_LENGTH*count);

	if(res < 0){
		printk(KERN_ALERT"Write to Kernel Failed");
		errno = -EAGAIN;
		return -1;
	}

	//Write_EEPROM(count,i2c_flash_devp,Kernel_Message_Pointer);
	if(i2c_flash_devp->EEPROM_BUSY == 0){
		INIT_WORK((struct work_struct *)work,Write_EEPROM);
		work->count 			= count;
		work->i2c_flash_devp 		= i2c_flash_devp;
		work->Kernel_Message_Pointer 	= Kernel_Message_Pointer;
	
		res = queue_work( my_wq, (struct work_struct *)work );
		return 0;
	}

	errno = -EBUSY;
	return -1;
}
//---------------------------------------------------------------------------------------------------------------
// Read EEPROM
static void Read_EEPROM(struct work_struct *work2){

	struct epprom_msg *send_message = kmalloc(sizeof(struct epprom_msg),GFP_KERNEL);
	int i ;
	int curr_page;
	u8 *Kernel_Pointer;

	my_work_t *my_work = (my_work_t *)work2;
	Kernel_Pointer = my_work->Kernel_Message_Pointer;
	curr_page = my_work->i2c_flash_devp->Current_Page_Read;

	my_work->i2c_flash_devp->EEPROM_BUSY = 1;	//Setting Status to Bus
	gpio_set_value_cansleep(OnBoardIO8,1);

	for(i = 0;i< my_work->count;i++)
	{
		Read_Page_EEPROM(send_message,send_message->user_message,my_work->i2c_flash_devp->Client_Address,my_work->i2c_flash_devp->Device_adapter, (u16)curr_page);
		memcpy(Kernel_Pointer+(i*PAGE_LENGTH),send_message->user_message, PAGE_LENGTH);
		/* Incrementing current write pointer */
		if(curr_page < 511){
			curr_page = curr_page + 1;
		}
		else{
			curr_page = 0;
		}
	
	}

	
	DATA_READY = 1;	
	my_work->i2c_flash_devp->Current_Page_Write = curr_page;
	my_work->i2c_flash_devp->EEPROM_BUSY = 0;	//Setting Status to Bus
	gpio_set_value_cansleep(OnBoardIO8,0);

	printk(KERN_ALERT"Read Complete");
	kfree(send_message);
}
//---------------------------------------------------------------------------------------------------------------
/* Read */
ssize_t i2c_flash_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct i2c_flash_dev *i2c_flash_devp = file->private_data;
	u8 *User_Data_Pointer;
	u8 *Kernel_Message_Pointer;	    						// Message pointer in userspace
	int res;

	User_Data_Pointer 	= (u8*)buf;

	if(DATA_READY == 1){
		res = copy_to_user(User_Data_Pointer,work2->Kernel_Message_Pointer,count*PAGE_LENGTH);
		DATA_READY = 0;
		if(res < 0){
			printk("Read to User Failed");
			gpio_set_value_cansleep(OnBoardIO8,0); //Setting IO8 to 1
			errno = -EAGAIN;
			return -1;
		}
		kfree(work2->Kernel_Message_Pointer);
		return 0;
	}

	if(i2c_flash_devp->EEPROM_BUSY == 0){
		Kernel_Message_Pointer = kmalloc(PAGE_LENGTH*count,GFP_KERNEL);	// Message pointer in kernel
		INIT_WORK((struct work_struct *)work2,Read_EEPROM);
		work2->count 			= count;
		work2->i2c_flash_devp 		= i2c_flash_devp;
		work2->Kernel_Message_Pointer 	= Kernel_Message_Pointer;
		res = queue_work( my_wq, (struct work_struct *)work2);	
		errno = -EAGAIN;
		return -1;
	}
	errno = -EBUSY;
	return -1;
}
//---------------------------------------------------------------------------------------------------------------

static long EEPROM_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_flash_dev *i2c_flash_devp = file->private_data;
	int i 	= 0;
	int ret = 0;

	printk(KERN_ALERT"Entered IOCTL\n");

	switch(cmd){
	case IOCTL_ERASE_FLASH: // Function for Erase
		printk(KERN_ALERT"Starting Erase\n");

		i2c_flash_devp->EEPROM_BUSY = 1;	//Setting Status to Busy
		gpio_set_value_cansleep(OnBoardIO8,1);	//Setting IO8 to 1

		for(i = 0; i<512; i++)
		{
			ret = 1;
			while (ret == 1){
				ret = Erase_Page_EEPROM(i2c_flash_devp->Client_Address,i2c_flash_devp->Device_adapter,i);
			}
		}

		printk(KERN_ALERT"ERASE Complete \n");

		i2c_flash_devp->EEPROM_BUSY = 0;	//Setting status to free
		gpio_set_value_cansleep(OnBoardIO8,0);	//Setting IO8 to 1

		return 0;

	case IOCTL_FLASHGETRP: // Returns Current Read Page Number
		return i2c_flash_devp->Current_Page_Read;

	case IOCTL_FLASHSETRP: // Sets Current Read Page Number
		i2c_flash_devp->Current_Page_Read = arg;
		return 0;

	case IOCTL_FLASHSETWP: // Sets Current Write Page Number
		return i2c_flash_devp->Current_Page_Write;

	case IOCTL_FLASHGETWP: // Returns Current Write Page Number
		i2c_flash_devp->Current_Page_Write = arg;
		return 0;
	case IOCTL_FLASHGETRS: // Returns Status Busy/Free(1/0)
		return i2c_flash_devp->EEPROM_BUSY;

	default:
		return 0;
	}
}

//---------------------------------------------------------------------------------------------------------------
/* INIT */

static struct file_operations eep_fops = {
	.owner 	= THIS_MODULE,
	.read  	= i2c_flash_read,
	.unlocked_ioctl  = EEPROM_ioctl,
	.open  	= i2c_flash_open,
	.release= i2c_flash_release,
	.write 	= i2c_flash_write,
};

int __init i2c_flash_device_init(void)
{
	int ret;
//---------------------------------------------------------------------------------------------------------------
	/* Creating GPIOs in sys folders */
	ret = gpio_request(OnBoardLED, "GPIO3");
	gpio_export(OnBoardLED, true);
	if (ret < 0 ){
		printk(KERN_ALERT"On board LED not created\n");
	}
	
	ret = gpio_request(OnBoardIO8, "GPIO26");
	gpio_export(OnBoardIO8, true);
	if (ret < 0 ){
		printk(KERN_ALERT"On board IO8 not created\n");
	}

	ret = gpio_request(I2CMUX, "GPIO29");
	gpio_export(I2CMUX, true);
	if (ret < 0 ){
		printk(KERN_ALERT"On I2CMUX not created\n");
	}

	/* Set Value of I2CMUX to zero i.e select I2C clock and SDA line */
	gpio_direction_output(OnBoardLED,1);
	gpio_direction_output(OnBoardIO8,1);
	gpio_direction_output(I2CMUX,0);

	gpio_set_value_cansleep(OnBoardLED,1);
	gpio_set_value_cansleep(I2CMUX,0);

	printk(KERN_ALERT"All GPIOs Set\n");
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------
	/* Creating Device File*/

	i2c_flash_dev_number = MKDEV(MAJOR_NUMBER, 0);

	if (register_chrdev_region(i2c_flash_dev_number, 0,DEVICE) < 0) {
		printk(KERN_DEBUG "Can't register device\n");
		return -1;
	}
	/* Allocating Memory */
	i2c_flash_devp = kmalloc(sizeof(struct i2c_flash_dev),GFP_KERNEL);
	i2c_flash_dev_class = class_create(THIS_MODULE, DEVICE);
	cdev_init(&i2c_flash_devp->cdev, &eep_fops);
	
	ret = cdev_add(&i2c_flash_devp->cdev, MKDEV(MAJOR(i2c_flash_dev_number), 0), 1);	
	if (ret) {
		printk("Bad cdev\n");
		return ret;
	}

	i2c_flash_dev_device =  device_create(i2c_flash_dev_class, NULL, MKDEV(MAJOR(i2c_flash_dev_number), 0), NULL, DEVICE);

	printk(KERN_ALERT"I2C Flash client created\n");

	/* Creatin Work Queue */
	my_wq = create_workqueue("write_queue");
	if (my_wq) {
		work = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
		if (!work) 
		{
			printk(KERN_ALERT"Work Queue Creation Failed \n");
			return -1;
		}
		work2 = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
		if (!work2) 
		{
			printk(KERN_ALERT"Work Queue Creation Failed \n");
			return -1;
		}
	}
	printk(KERN_ALERT"Work Queue Created \n");

//---------------------------------------------------------------------------------------------------------------

	
	return 0;
}

//---------------------------------------------------------------------------------------------------------------
// Opening the Device

void __exit i2c_flash_device_exit(void)
{
	flush_workqueue(my_wq);
	destroy_workqueue(my_wq);
	kfree(work);
	kfree(work2);

	/* Removing Device */
	unregister_chrdev_region(i2c_flash_dev_number,1);
	device_destroy(i2c_flash_dev_class, i2c_flash_dev_number);
	cdev_del(&i2c_flash_devp->cdev);
	kfree(i2c_flash_devp);
	class_destroy(i2c_flash_dev_class);

	/* Removing GPIOs */
	gpio_set_value_cansleep(OnBoardLED,0);
	gpio_set_value_cansleep(I2CMUX,1);
	gpio_free(OnBoardLED);
	gpio_free(OnBoardIO8);
	gpio_free(I2CMUX);
	
	printk(KERN_ALERT "Have a Good Day :)\n");

}

module_init(i2c_flash_device_init);
module_exit(i2c_flash_device_exit);
