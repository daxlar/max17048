#define DT_DRV_COMPAT maxim_max17048a

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "max17048a.h"

LOG_MODULE_REGISTER(MAX17048A, CONFIG_SENSOR_LOG_LEVEL);

struct max17048a_data {
    uint8_t channel_states;
};

struct max17048a_dev_config {
    struct i2c_dt_spec i2c;
};

static int max17048a_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan) {
	
    return 0;
}

static int max17048a_channel_get(const struct device *dev, 
                               enum sensor_channel chan, 
                               struct sensor_value *val) {
	
	const struct max17048a_dev_config *config = dev->config;

	switch(chan) {
		case MAX17048A_GET_SOC : {
			uint8_t soc_data[2];
			int i2c_operation_error = i2c_burst_read_dt(&config->i2c, MAX17048A_SOC_REG, soc_data, 2);
			if(i2c_operation_error) {
				printk("trouble reading MAX17048A soc register \r\n");
				return -EIO;
			}
			uint16_t soc_value = soc_data[0];
			soc_value = soc_value << 8;
			soc_value |= soc_data[1];
			val->val1 = soc_value / 256;
			break;
		}
		case MAX17048A_GET_CHARGE_RATE : {
			uint8_t crate_data[2];
			int i2c_operation_error = i2c_burst_read_dt(&config->i2c, MAX17048A_CRATE_REG, crate_data, 2);
			if(i2c_operation_error) {
				printk("trouble reading MAX17048A crate register \r\n");
				return -EIO;
			}
			int16_t crate_value = crate_data[0]; 
			crate_value = crate_value << 8;
			crate_value |= crate_data[1];
			val->val1 = (float)crate_value * 0.208;
			break;
		}
		case MAX17048A_GET_STATUS : {
			uint8_t status_data[2];
			int i2c_operation_error = i2c_burst_read_dt(&config->i2c, MAX17048A_STATUS_REG, status_data, 2);
			if(i2c_operation_error) {
				printk("trouble reading MAX17048A status register \r\n");
				return -EIO;
			}
			uint8_t status_value = status_data[0]; 
			val->val1 = status_value;
			break;
		}
	}

    return 0;
}


static int max17048a_attr_set(const struct device *dev, 
                            enum sensor_channel chan, 
                            enum sensor_attribute attr, 
                            const struct sensor_value *val) {

	const struct max17048a_dev_config *config = dev->config;

	switch(chan) {
		case MAX17048A_INITIALIZE: {
			// set up low soc interrupts here
			// 1). clear all status flags
			// 	a). clear STATUS.RI bit to clear the initial reset alert
			// 	b). clear STATUS.EnVR bit to disable voltage reset alert
			// 2). clear HIBRT to disable hibernate (only hibernates based on CRATE or OCV)
			// 3). set VALRT.MAX to correspond to 3.7V for voltage alert
			// 	a). set VALRT.MIN to correspond to 0V for voltage alert
			// 4). clear CONFIG.ALRT bit to deassert int pin
			// 	a).  clear CONFIG.ALSC bit to disable 1% alerting
			//  b).  set CONFIG.ATHD to correspond to 20% soc to enable low soc alert

			uint8_t status_data[2] = {0x00, 0x00};
			int i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_STATUS_REG, status_data, 2);
			if(i2c_operation_error) {
				printk("trouble clearing MAX17048A status register \r\n");
				return -EIO;
			}

			uint8_t hibrt_data[2] = {0x00, 0x00};
			i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_HIBRT_REG, hibrt_data, 2);
			if(i2c_operation_error) {
				printk("trouble clearing MAX17048A hibrt register \r\n");
				return -EIO;
			}

			uint8_t valrt_data[2] = {0x00, 0b10111110};
			i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_VALRT_REG, valrt_data, 2);
			if(i2c_operation_error) {
				printk("trouble setting MAX17048A valrt register \r\n");
				return -EIO;
			}

			uint8_t config_data[2] = {0x97, 0x00};
			config_data[1] = 0b00001111;
			i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_CONFIG_REG, config_data, 2);
			if(i2c_operation_error) {
				printk("trouble setting MAX17048A config register \r\n");
				return -EIO;
			}

			break;
		}
		case MAX17048A_CLEAR_ALERT: {
			// clear the low soc interrupt alert
			// 1). clear CONFIG.ALRT to service and deassert ALRT pin
			// 2). clear any STATUS.X bit that's high

			printk("at clear alert \r\n");

			uint8_t status_data[2] = {0x00, 0x00};
			int i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_STATUS_REG, status_data, 2);
			if(i2c_operation_error) {
				printk("trouble clearing MAX17048A status register at clear alert\r\n");
				return -EIO;
			}

			uint8_t config_data[2] = {0x97, 0x00};
			config_data[1] = 0b00001111;
			i2c_operation_error = i2c_burst_write_dt(&config->i2c, MAX17048A_CONFIG_REG, config_data, 2);
			if(i2c_operation_error) {
				printk("trouble setting MAX17048A config register at clear alert\r\n");
				return -EIO;
			}
			
			printk("cleared alert \r\n");

			break;
		}
	}

	return 0;
}

int max17048a_init(const struct device *dev) {

    const struct max17048a_dev_config *config = dev->config;
    
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("failure with i2c bus \r\n");
		return -ENODEV;
	}

	return 0;
}

static const struct sensor_driver_api max17048a_api_funcs = {
	.sample_fetch = max17048a_sample_fetch,
	.channel_get = max17048a_channel_get,
	.attr_set = max17048a_attr_set,
};

#define MAX17048A_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      max17048a_init,				        \
			      NULL,					            \
			      &max17048a_data_##inst,			    \
			      &max17048a_config_##inst,			\
			      POST_KERNEL,				        \
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &max17048a_api_funcs);

#define MAX17048A_CONFIG(inst)								\
    {                                                       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
    }                                                       \

#define MAX17048A_DEFINE(inst)                                        \
    static struct max17048a_data max17048a_data_##inst;                 \
    static const struct max17048a_dev_config max17048a_config_##inst =	\
        MAX17048A_CONFIG(inst);				                        \
    MAX17048A_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(MAX17048A_DEFINE)
