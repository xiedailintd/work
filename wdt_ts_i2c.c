/* 
 * drivers/input/touchscreen/wdt8913.c
 *
 * Weida Hi-Tech wdt8913 TouchScreen driver. 
 *
 * Copyright (c) 2015 Weida Hi-Tech
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/irq.h>
#include <linux/ioc4.h>
#include <linux/io.h>
#include <linux/module.h> 
#include <linux/proc_fs.h>
#include <asm/unaligned.h>
#include <linux/input/mt.h>
#include <linux/version.h>
#include <linux/of_gpio.h>

#define		I2C_MASTER_CLK		400 * 1000

#define WDT8913_NAME			"wdt8913"
#define WDT8913_DRV_VER			"0.9.1"

#define WDT8913_MAX_POINTS		10
#define WDT8913_MAX_X			32768
#define WDT8913_MAX_Y			32768

#define	C_FW_CMD_SET_DEVICE_MODE	0xBB

/* for read and write memory, auto-increment */
#define C_ISP_CMD_SET_MEM_ADDRESS	0xC0
#define C_ISP_CMD_READ_WORDS		0xC1
#define C_ISP_CMD_READ_HALFWORDS	0xC2
#define C_ISP_CMD_READ_BYTES		0xC3
#define C_ISP_CMD_WRITE_WORDS		0xC4
#define C_ISP_CMD_WRITE_HALFWORDS	0xC5
#define C_ISP_CMD_WRITE_BYTES		0xC6

/* Support in vA IC only. Use this commmand to identify the protocol version */
#define C_ISP_CMD_READ_GP_REGISTER	0xC7
#define C_ISP_CMD_READ_DEVICE_INFO	0xC8

/* for read and write, auto-increment */
#define C_ISP_CMD_SET_FLASH_ADDRESS	0xD0
#define C_ISP_CMD_READ_FLASH		0xD1
#define C_ISP_CMD_WRITE_FLASH		0xD2
#define C_ISP_CMD_ERASE_FLASH		0xD3

/* write protection on/off */
#define C_ISP_CMD_LOCK_FLASH		0xD4

/* enable flash interface */
#define C_ISP_CMD_ENABLE_FLASH		0xD5
#define C_ISP_CMD_CHECKSUM_FLASH	0xD6

#define C_ISP_CMD_RESET_CONNECTION	0xDA
#define C_ISP_CMD_CALL_FUNCTION		0xDB
#define C_ISP_CMD_RUN_PROGRAM		0xDC
#define C_ISP_CMD_PING			0xDD

/* success */
#define	C_ISP_RSP_OK			0x80	

/* Invalid address */
#define	C_ISP_RSP_INVALID_ADRESS	0xF0	

/* Invalid count */
#define	C_ISP_RSP_INVALID_COUNT		0xF1

/* Timeout (not enough bytes received)	*/
#define	C_ISP_RSP_TIMEOUT		0xF2

/* Invalid command */
#define	C_ISP_RSP_INVALID_COMMAND	0xF3

/* Invalid flash region */
#define	C_ISP_RSP_INVALID_REGION	0xF4

/* Invalid key */
#define	C_ISP_RSP_INVALID_KEY		0xF5

/* Invalid function  call address */
#define	C_ISP_RSP_INVALID_FUNC_ADDRESS	0xF6

/* Flash busy (callback function) */
#define	C_ISP_RSP_BUSY			0xFE

/* mp test pass */
#define C_MP_RSP_PASS			0x81

/* mp test fail */
#define C_MP_RSP_FAIL			0x8F

#define	FLASH_BATCH_READ_SIZE		30
#define	FLASH_BATCH_WRITE_SIZE		31
#define	MEMORY_BYTE_BATCH_READ_SIZE	30
#define	MEMORY_BYTE_BATCH_WRITE_SIZE 	30

#define ENABLE_TP_WAKEUP 0

static const unsigned short weida_i2c[2] = {0x2C, I2C_CLIENT_END};

#if ENABLE_TP_WAKEUP
extern volatile u32 lcdc_off_flag;
#endif

/*
 * BOOTLOADER only exists during ROM and fastboot. It cannot be set as the
 * target mode. SENSING is Normal sensing. DOZE is that device sleeps most time
 * and does sensing in a period. SLEEP is that both device and sensor are in
 * sleep. FACTORY is for MP testing. The device cannot switch to other modes
 * unless reboot it. When entering COMMAND mode, device is waiting for commands.
 * Do flash/memory access commands mostly.
 */
typedef enum {
	BOOTLOADER = 0,
	SENSING = 1,
	DOZE = 2,
	SLEEP = 3,
	FACTORY = 0x80, 
	COMMAND = 0x90,
} device_mode_t;

struct wdt8913 {
	struct i2c_client *client;
	struct input_dev *input;
	bool isp_mode;
	bool tip_flags[10];
	bool exiting;
};

static int wdt8913_wait_complete_and_return_data(struct i2c_client * client,
						 void * result, int result_size,
						 int polling_interval_us)
{
	u8 read_buf[32];
	read_buf[0] = C_ISP_RSP_BUSY;
	while (1) {
		int rc;

		
		if (polling_interval_us > 0)
			udelay(polling_interval_us);

		
		if ((rc = i2c_master_recv(client, read_buf, result_size+1)) > 0) {
			if (read_buf[0] == C_ISP_RSP_OK) {
				if (result_size > 0)
					memcpy(result, read_buf+1, result_size);
				return rc;
			}
			else if (read_buf[0] == C_ISP_RSP_BUSY)
				continue;
			else if (read_buf[0] == C_MP_RSP_PASS)
				return C_MP_RSP_PASS;
			else if (read_buf[0] == C_MP_RSP_FAIL)
				return C_MP_RSP_FAIL;
			else
				return -1;	
		}
	}	
}

static int wdt8913_write_command_and_return_data(struct i2c_client * client,
						 const u8 cmd[], int cmd_size,
						 void * result, int result_size,
						 int polling_interval_us)
{
	if (i2c_master_send(client, cmd, cmd_size) >= 0)
		return wdt8913_wait_complete_and_return_data(client, result,
				result_size, polling_interval_us);
	else
		return -1;
}

static int wdt8913_write_command(struct i2c_client * client, const u8 cmd[],
				 int size, int polling_interval_us)
{
	if (i2c_master_send(client, cmd, size) >= 0)
		return wdt8913_wait_complete_and_return_data(client, NULL, 0,
							     polling_interval_us);

	
	return -1;	
}

static int wdt8913_set_memory_address(struct i2c_client * client, u32 mem_address)
{
	u8 cmd[] = { C_ISP_CMD_SET_MEM_ADDRESS, (u8)mem_address,
		     (u8)(mem_address>>8), (u8)(mem_address>>16),
		     (u8)(mem_address>>24) };

	return wdt8913_write_command(client, cmd, sizeof(cmd), 0);
}

/* size shall be a positive number less or equal to MEMORY_BYTE_BATCH_READ_SIZE */
static int wdt8913_read_memory_data(struct i2c_client * client, u32 mem_cmd,
				    void * buf, int start, int item_size, int item_count)
{
	u8 cmd[] = { mem_cmd, item_count };

	
	/*
	 * we need a little delay to avoid read in the middle of write
	 * (device write to I2C FIFO)
	 */
	return wdt8913_write_command_and_return_data(client, cmd, sizeof(cmd),
						     buf + start,
						     item_size * item_count, 0);
}

/* size shall be a positive number less or equal to MEMORY_BYTE_BATCH_WRITE_SIZE */
static int wdt8913_write_memory_data(struct i2c_client * client, u32 mem_cmd,
				     const void * buf, int start, int size)
{
	u8 cmd[1+MEMORY_BYTE_BATCH_WRITE_SIZE];
	if (size > MEMORY_BYTE_BATCH_WRITE_SIZE)
		return -1;
	cmd[0] = mem_cmd;
	memcpy(&(cmd[1]), buf+start, size);

	
	return wdt8913_write_command(client, cmd, 1+size, 0);
}

static int wdt8913_read_memory_words(struct i2c_client * client, u32 buf[], int word_count)
{
	int word_batch_count = MEMORY_BYTE_BATCH_READ_SIZE / sizeof(u32);
	int offset = 0;
	while (word_count >= word_batch_count) {
		if (wdt8913_read_memory_data(client, C_ISP_CMD_READ_WORDS, buf,
					     offset, sizeof(u32), word_batch_count) < 0)
			return -1;
		offset += word_batch_count*sizeof(u32);
		word_count -= word_batch_count;
	}
	if (word_count > 0) {
		if (wdt8913_read_memory_data(client, C_ISP_CMD_READ_WORDS, buf,
					     offset, sizeof(u32), word_count) < 0)
			return -1;
	}
	return 0;
}

static int wdt8913_write_memory_words(struct i2c_client * client,
				      const u32 buf[], int word_count)
{
	int word_batch_count = MEMORY_BYTE_BATCH_WRITE_SIZE/sizeof(u32);
	int offset = 0;
	while (word_count >= word_batch_count) {
		if (wdt8913_write_memory_data(client, C_ISP_CMD_WRITE_WORDS, buf,
					      offset, word_batch_count*sizeof(u32)) < 0)
			return -1;
		offset += word_batch_count*sizeof(u32);
		word_count -= word_batch_count;
	}

	if (word_count > 0) {
		if (wdt8913_write_memory_data(client, C_ISP_CMD_WRITE_WORDS, buf,
					      offset, word_count*sizeof(u32)) < 0)
			return -1;
	}
	return 0;
}

static int wdt8913_set_device_mode(struct i2c_client * client, device_mode_t mode)
{
	if (mode == COMMAND) {
		const u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, COMMAND, 0x73, 0x50 };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	} else if (mode == FACTORY) {
		const u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, FACTORY, 0xad, 0xb1 };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	} else	{
		u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, mode };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	}

	return 0;
}

static int wdt8913_rom_workaround(struct i2c_client * client)
{
	u32 fw_version;
	u8  cmd[] = { C_ISP_CMD_READ_DEVICE_INFO, sizeof(fw_version), 4, 0 };

	if (wdt8913_write_command_and_return_data(client, cmd, sizeof(cmd),
					&fw_version, sizeof(fw_version), 1000)) {
		struct wdt8913 *tsdata = i2c_get_clientdata(client);

		dev_info(&client->dev, "fw_ver=0x%08x\n", fw_version);

		if (fw_version < 0x01000000) {
			u32 register_value;

			/* ROM code bug workaround */
			wdt8913_set_memory_address(client, 0x9d160020);
			register_value = 0;
			if (wdt8913_read_memory_words(client, &register_value, 1) >= 0
			    && (register_value & 2)) {
				/* i2cs.irq_dis = 0xfffe */
				wdt8913_set_memory_address(client, 0x9d160028);	
				register_value = 0xfffe;
				wdt8913_write_memory_words(client, &register_value, 1);

				/* i2cs.irq_en = 0x0001	*/
				wdt8913_set_memory_address(client, 0x9d160024);	
				register_value = 0x0001;
				wdt8913_write_memory_words(client, &register_value, 1);
			}
			tsdata->isp_mode = true;

			/* return on ISP mode */
			return 0;	
		} else {
			tsdata->isp_mode = false;

			/* return on FW mode */
			return 1;	
		}
	} else {
		dev_err(&client->dev, "Unable to read device info.\n");
		return -ENODEV;
	}
}

static int wdt8913_report_touch_info(struct wdt8913 * tsdata)
{
	struct i2c_client * client = tsdata->client;
	u8 cmd[] = { 0x90 };
	u8 data[3];
	int n_touches=0;
	int retry;
	if (i2c_master_send(client, cmd, sizeof(cmd)) == sizeof(cmd))
	{
		retry = 5;
		while (retry-- > 0) {
			udelay(100);
			if (i2c_master_recv(client, data, 3) == 3)
				break;
		}

		if (retry == 0)
			return -1;	

		/*
		 * data[0] is the finger nuber 
		 */
		n_touches = data[0]; 
	}

	return n_touches;
}

static int wdt8913_report_touch(struct wdt8913 * tsdata, int n_touches)
{
	struct i2c_client * client = tsdata->client;
	u8 cmd[] = {0x91};
	u8 data[500];
	int i;
	int retry;

/*	if (n_touches > WDT8913_MAX_POINTS) {
		printk("%s: max fingers exceed 10!\n", __func__);
		return -EINVAL;
   }
	*/
	if (i2c_master_send(client, cmd, sizeof(cmd)) == sizeof(cmd)) {
		retry = 5;
		while (retry-- > 0) {
			udelay(100);
			if (i2c_master_recv(client, data, n_touches*5) ==
			    (n_touches*5))
				break;
		}

		if (retry == 0)
			return -1;	
		for (i=0; i<n_touches; i++) {
			int x, y, id, flag;
			input_mt_slot(tsdata->input, i);
			flag = data[5*i];

			id = (flag >> 3) - 1;	

			flag = flag & 1;
			if (id < 0 || id >= WDT8913_MAX_POINTS)
				continue;

			x = get_unaligned_le16(&(data[ 5*i + 1]));
			y = get_unaligned_le16(&(data[ 5*i + 3]));

			if (flag == 0 && tsdata->tip_flags[id] != false) {
				input_mt_slot(tsdata->input, id);
				input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, false);
				tsdata->tip_flags[id] = false;
			} else {
				input_mt_slot(tsdata->input, id);
				input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, true);
				input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
				input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);	
				tsdata->tip_flags[id] = true;
			}
		}
		
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)
		input_mt_report_pointer_emulation(tsdata->input, 0);
#else
		input_mt_sync_frame(tsdata->input);
#endif
		/* to mark the end of this transfer */
		input_sync(tsdata->input);		
	
		#if ENABLE_TP_WAKEUP
		//printk("lcdc_off_flag is %d\n",lcdc_off_flag);
		if(lcdc_off_flag)
		{
			input_event(tsdata->input, EV_KEY, KEY_WAKEUP, 1);
			input_event(tsdata->input, EV_KEY, KEY_WAKEUP, 0);
			input_sync(tsdata->input);
		}
		#endif
	
		return n_touches;
	}
	return -1;	
}

static irqreturn_t wdt8913_isr(int irq, void *dev_id)
{
	struct wdt8913 *tsdata = dev_id;
	int n_touches = 0;	
	
	n_touches = wdt8913_report_touch_info(tsdata);

	wdt8913_report_touch(tsdata,n_touches);
	
	return IRQ_HANDLED;
}

static int wdt8913_handle_of(struct i2c_client *client)
{
	int error;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags rst_flags, irq_flags;
	int reset_pin;

	client->irq = of_get_named_gpio_flags(np, "irq_gpio_number", 0, &irq_flags);
	reset_pin = of_get_named_gpio_flags(np, "rst_gpio_number", 0, &rst_flags);
					    
	error = gpio_request(reset_pin, "reset GPIO");	
	if (error < 0) {
		dev_err(&client->dev, "%s %d: request gpio fail (%d)\n", __func__, __LINE__, error);
		return -EINVAL;
	} else {
		gpio_direction_output(reset_pin, 1);
	}

	dev_info(&client->dev, "irq gpio num: (%d)\n", client->irq);

	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}

	dev_info(&client->dev, "client irq num: (%d)\n", client->irq);

	return error;
}

static int wdt8913_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(dev, "suspend\n");

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	
	wdt8913_set_device_mode(client, SLEEP);

	return 0;
}

static int wdt8913_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(dev, "resume\n");

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	wdt8913_set_device_mode(client, SENSING);

	return 0;
}

static int wdt8913_setup_irq(struct i2c_client *client)
{
	return wdt8913_handle_of(client);
}

static int wdt8913_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct wdt8913 *tsdata;
	struct input_dev *input;
	int error;

	printk("%s\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "No I2C!\n");
		return -ENODEV;
	}
	
	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	memset(&(tsdata->tip_flags), 0, sizeof(tsdata->tip_flags));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input = input_allocate_device();
#else
	input = devm_input_allocate_device(dev);
#endif
	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	
	if (!tsdata || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	tsdata->client = client;
	tsdata->input = input;
	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	if (wdt8913_rom_workaround(client) < 0) {
		error = -ENODEV;
		goto err_free_mem;
	}

	dev_info(dev, "input dev iniit\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_mt_init_slots(input, WDT8913_MAX_POINTS);
#else
	input_mt_init_slots(input, WDT8913_MAX_POINTS,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif

	set_bit(ABS_MT_POSITION_X, input->absbit);
	set_bit(ABS_MT_POSITION_Y, input->absbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, WDT8913_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, WDT8913_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, WDT8913_MAX_POINTS, 0, 0);

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN,	input->evbit);
	
	#if ENABLE_TP_WAKEUP
	input_set_capability(input, EV_KEY, KEY_POWER);
	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	device_init_wakeup(dev, 1);	
	#endif	
	
	error = wdt8913_setup_irq(client);
	if (error)
		return error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
	set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#else
	irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#endif

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
//					  wdt8913_isr, IRQF_TRIGGER_FALLING,
					  wdt8913_isr, IRQF_ONESHOT,
					  client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}

	dev_info(dev, "register input dev\n");
	error = input_register_device(input);
	if (error) 
		goto err_free_irq;

	dev_info(dev, "probe done\n");
	return 0;
err_free_irq:
	//free_irq(client->irq, tsdata);
err_free_mem:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input_free_device(input);

#endif	
	return error;
}

static int wdt8913_remove(struct i2c_client *client)
{
	struct wdt8913 *tsdata = i2c_get_clientdata(client);

	dev_info(&client->dev, "remove\n");

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input_unregister_device(tsdata->input);

#endif
	return 0;
}

static struct of_device_id weida_ts_match[] = {                                                                               
    {.compatible = "weida,wdt89xx", .data = NULL},
    {},
};
static const struct i2c_device_id wdt8913_id[] = {
	{ WDT8913_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, wdt8913_id);

static SIMPLE_DEV_PM_OPS(wdt8913_pm_ops, wdt8913_suspend, wdt8913_resume);

static struct i2c_driver wdt8913_driver = {
	.probe		= wdt8913_probe,
	.remove		= wdt8913_remove,
	.id_table	= wdt8913_id,
	.driver	= {
		.name	= WDT8913_NAME,
		.owner	= THIS_MODULE,
		.pm	= &wdt8913_pm_ops,
		.of_match_table = of_match_ptr(weida_ts_match), 
	},
};

static int __init wdt8913_init(void)
{
	int ret = 0;
	
	ret = i2c_add_driver(&wdt8913_driver);
	printk(KERN_INFO"ret = %d\n",ret);

	return ret;
}

static void __exit wdt8913_exit(void)
{
	printk(KERN_INFO"%s exit\n", WDT8913_NAME);
	i2c_del_driver(&wdt8913_driver);
}

late_initcall(wdt8913_init);
module_exit(wdt8913_exit);

MODULE_AUTHOR("<victor.chang@weidahitech.com>");
MODULE_DESCRIPTION("Weida Hi-Tech WDT8913 touch driver");
MODULE_VERSION(WDT8913_DRV_VER);
MODULE_LICENSE("GPL");

