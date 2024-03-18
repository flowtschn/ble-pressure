/*
 *  lis2dh12tr.c
 *
 *  Created on: Dec 18, 2023
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
//#include "lis2dh12_reg.h"
//#include "lis2dh12tr.h"

LOG_MODULE_REGISTER(lis2dh12tr, 3);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

//#define LIS2DH12TR_ADDRESS 0x18

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/* I2C bus */
//static const struct i2c_dt_spec lis2dh12tr_bus = {
//    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c0_inst)),
//    .addr = LIS2DH12TR_ADDRESS
//};

/* Interrupt to wake up */
//static const struct gpio_dt_spec accel_int1 = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(accelint1), gpios, 0);
//static struct gpio_callback accel_int_cb;

const struct device *const accel_dev = DEVICE_DT_GET_ANY(st_lis2dh12);

/* Interrupt queue */
K_MSGQ_DEFINE(ext_queue, sizeof(uint8_t), 1, 4);

//static lis2dh12_ctx_t dev_ctx;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/*!
 * @brief  Initialize external interrupt from INT1 pin
 */
//static void ext_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//    uint8_t pin = pins;
//    k_msgq_put(&ext_queue, &pin, K_NO_WAIT);
//}

static void fetch_and_display(const struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel[3];
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_Z,
					accel);
	}
	if (rc < 0) {
		LOG_ERR("ERROR: Update failed: %d", rc);
	} else {
        uint8_t pin = 2;
        k_msgq_put(&ext_queue, &pin, K_NO_WAIT);
	}
}

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	fetch_and_display(dev);
}

/*!
 * @brief  Write generic device register
 */
//static int lis2dh12tr_write(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
//    return i2c_burst_write_dt(&lis2dh12tr_bus, reg, buf, len);
//}

/*!
 * @brief  Read generic device register
 */
//static int lis2dh12tr_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
//    return i2c_burst_read_dt(&lis2dh12tr_bus, reg, buf, len);
//}

/*!
 * @brief  Configure sensor in interrupt mode
 */
int start_interrupt_mode(void) {
    //int ret;
    /* Default settings */
    //ret = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_100Hz);
    //if (ret != 0) LOG_ERR("lis2dh12_data_rate_set failed, error = %d", ret);
    //ret = lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_8g);
    //if (ret != 0) LOG_ERR("lis2dh12_full_scale_set failed, error = %d", ret);
    //ret = lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);    /* Disable temperature measurement */
    //if (ret != 0) LOG_ERR("lis2dh12_temperature_meas_set failed, error = %d", ret);
    //ret = lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
    //if (ret != 0) LOG_ERR("lis2dh12_operating_mode_set failed, error = %d", ret);

    /* Configure interrupt */
    //ret = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);
    //if (ret != 0) LOG_ERR("lis2dh12_data_rate_set failed, error = %d", ret);

    /* Disable interrupt */
    //lis2dh12_int1_cfg_t val;
    //memset(&val, 0, sizeof(val));
    //ret = lis2dh12_int1_gen_conf_set(&dev_ctx, &val);
    //if (ret != 0) LOG_ERR("lis2dh12_int1_gen_conf_set failed, error = %d", ret);

#if 0
    //lis2dh12_ctrl_reg6_t val6;
    //lis2dh12_pin_int2_config_get(&dev_ctx, &val6);
    //val6.int_polarity = 1;    /* Set INT_POLARITY bit for active low */
    //lis2dh12_pin_int2_config_set(&dev_ctx, &val6);
#endif

    //lis2dh12_ctrl_reg3_t val3;
    //lis2dh12_pin_int1_config_get(&dev_ctx, &val3);
    //val3.i1_ia1 = 1;          /* Enable IA1 on INT1 */
    //ret = lis2dh12_pin_int1_config_set(&dev_ctx, &val3);
    //if (ret != 0) LOG_ERR("lis2dh12_pin_int1_config_set failed, error = %d", ret);

    /* Set threshold INT1_THS = 500mg / 16mb = 31
     * 62 = 90 degree tilt before interrupt
     * 31 = 45 degree tilt before interrupt
     */
    //ret = lis2dh12_int1_gen_threshold_set(&dev_ctx, 8);
    //if (ret != 0) LOG_ERR("lis2dh12_int1_gen_threshold_set failed, error = %d", ret);

    /* Set duration < 1 / ODR */
    //ret = lis2dh12_int1_gen_duration_set(&dev_ctx, 2);
    //if (ret != 0) LOG_ERR("lis2dh12_int1_gen_duration_set failed, error = %d", ret);

    /* Latch interrupt 1, CTRL_REG5, LIR_INT1 */
    //lis2dh12_ctrl_reg5_t ctrl_reg5;
    //lis2dh12_read_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
    //ctrl_reg5.lir_int1 = 0;
    //ret = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t *)&ctrl_reg5, 1);
    //if (ret != 0) LOG_ERR("lis2dh12_write_reg failed, error = %d", ret);

    /* Clear the interrupt */
    //lis2dh12_int1_src_t val1;
    //int timeout = 0;
    //do {
        /* Reading int will clear it */
    //    lis2dh12_int1_gen_source_get(&dev_ctx, &val1);
    //    k_msleep(10);
    //    if (timeout++ > 10) {
    //        break;
    //    }
    //} while (val1.ia);

    /* Set data rate */
    //ret = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_100Hz);    /* For low power */
    //if (ret != 0) LOG_ERR("lis2dh12_data_rate_set failed, error = %d", ret);

    /* Enable interrupt */
    //memset(&val, 0, sizeof(val));
    // val.yhie = 1;    /* Enable only X or Y as interrupt sources */
    //val.xhie = 1;    /* Leave out Z otherwise it will always trigger when sitting on table */
    // val.zhie = 1;
    //ret = lis2dh12_int1_gen_conf_set(&dev_ctx, &val);
    //if (ret != 0) LOG_ERR("lis2dh12_int1_gen_conf_set failed, error = %d", ret);

    struct sensor_trigger accel_trig;
    int rc = 0;

    accel_trig.type = SENSOR_TRIG_MOTION;
    accel_trig.chan = SENSOR_CHAN_ACCEL_Z;

    struct sensor_value lower_thresh = {
        .val1 = 45,
    };

    struct sensor_value upper_thresh = {
        .val1 = 90,
    };

    struct sensor_value full_scale = {
        .val1 = 8,
    };

    struct sensor_value odr = {
        .val1 = 100,
    };

    rc = sensor_attr_set(accel_dev, accel_trig.chan,
                    SENSOR_ATTR_SAMPLING_FREQUENCY,
                    &odr);
    if (rc != 0) {
        LOG_ERR("Failed to set odr: %d", rc);
        return rc;
    }

    rc = sensor_attr_set(accel_dev, accel_trig.chan,
                    SENSOR_ATTR_FULL_SCALE,
                    &full_scale);
    if (rc != 0) {
        LOG_ERR("Failed to set full_scale attribute: %d", rc);
        return rc;
    }

    rc = sensor_attr_set(accel_dev, accel_trig.chan,
                    SENSOR_ATTR_LOWER_THRESH,
                    &lower_thresh);
    if (rc != 0) {
        LOG_ERR("Failed to set lower_thresh attribute: %d", rc);
        return rc;
    }

    rc = sensor_attr_set(accel_dev, accel_trig.chan,
                    SENSOR_ATTR_UPPER_THRESH,
                    &upper_thresh);
    if (rc != 0) {
        LOG_ERR("Failed to set upper_thresh attribute: %d", rc);
        return rc;
    }
         
    rc = sensor_trigger_set(accel_dev, &accel_trig, trigger_handler);
    if (rc != 0) {
        LOG_ERR("Failed to set trigger: %d\n", rc);
        return rc;
    }

    LOG_INF("Begin Interrupt Scanning");

    return rc;
}

/******************************************************************************/

/*!
 * @brief  Waiting for interrupt from sensor
 */
int lis2dh12tr_wait_interrupt(k_timeout_t timeout) {
    uint8_t pin;
    return k_msgq_get(&ext_queue, &pin, timeout);
}

/*!
 * @brief  Power down.
 */
int lis2dh12tr_power_down(void)
{
    //int ret = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_POWER_DOWN);
    //if (ret != 0) LOG_ERR("lis2dh12_data_rate_set failed, error = %d", ret);
    return 0;
}

/*!
 * @brief  Initialize LIS2DH12TR
 */
int lis2dh12tr_init(void) {
    //int err;
    //dev_ctx.write_reg = lis2dh12tr_write;
    //dev_ctx.read_reg = lis2dh12tr_read;
    //dev_ctx.handle = NULL;

    /* Check I2C bus is ready */
    //if (!device_is_ready(lis2dh12tr_bus.bus)) {
    //    LOG_ERR("I2C0 bus device not ready");
    //    return -1;
    //}

    /* Check sensor is available */
    //uint8_t whoamI;
    //lis2dh12_device_id_get(&dev_ctx, &whoamI);
    //if (whoamI != LIS2DH12_ID) {
    //    LOG_ERR("Couldn't get LIS2DH12 ID %02X != %02X", LIS2DH12_ID, whoamI);
    //    return -2;
    //}

    /* Check interrupt is ready */
    //if (!gpio_is_ready_dt(&accel_int1)) {
    //    LOG_ERR("Interrupt device not ready");
    //    return -3;
    //}

    //err = gpio_pin_configure_dt(&accel_int1, GPIO_INPUT);
    //if (err) {
    //    LOG_ERR("Failed to configure intput pin (err %d)", err);
    //    return -4;
    //}

    //err = gpio_pin_interrupt_configure_dt(&accel_int1, GPIO_INT_EDGE_BOTH);
    //if (err) {
    //    LOG_ERR("Failed to configure intput interrupt (err %d)", err);
    //    return -5;
    //}

    /* Attach interrupt functions */
    //gpio_init_callback(&accel_int_cb, ext_interrupt_callback, BIT(accel_int1.pin));
    //gpio_add_callback(accel_int1.port, &accel_int_cb);
    
    if (accel_dev == NULL) {
		LOG_ERR("No accel found");
		return 0;
	}
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("Device %s is not ready", accel_dev->name);
		return 0;
	}
    uint8_t pin = 2;
    k_msgq_put(&ext_queue, &pin, K_NO_WAIT);    /* Push the first time to adv after power up */

    /* Start sensor in the interrupt mode */
    start_interrupt_mode();
    return 0;
}

