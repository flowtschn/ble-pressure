/*
 *  main.c
 *
 *  Created on: Dec 17, 2023
 */

/******************************************************************************/

/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/gpio.h>
#include <helpers/nrfx_reset_reason.h>
#include "tk2700/tk2700.h"
#include "lis2dh12tr/lis2dh12tr.h"

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_INF);

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define ADV_INTERVAL_S          20    /* Advertising upto ADV_INTERVAL_S seconds if no motion detected */
#define MEASURE_INTERVAL_S      5     /* Measure and update adv data every MEASURE_INTERVAL_S seconds */

#define MAX_PRESSURE            1000    /* Max is 10 bar so max value = 10 * 100 = 1000 kPA */
#define DEVICE_NAME_LENGTH      (sizeof(CONFIG_BT_DEVICE_NAME) + 5)    /* Prefix name + '-XXXX' */

#define SERVICE_UUID            0x181A      /* Custom service UUID */
#define IDX_TEMPL               4           /* Index of lo byte of temp in service data*/
#define IDX_TEMPH               5           /* Index of hi byte of temp in service data*/

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, \
                  BT_GAP_ADV_SLOW_INT_MIN, \
                  BT_GAP_ADV_SLOW_INT_MAX, NULL)

/* Index in the service data */
#define PRES_INDEX              4
#define TEMP_INDEX              6

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

static uint8_t service_data[] = {
    BT_UUID_16_ENCODE(SERVICE_UUID),
    0x07,    /* Length */
    0x01,    /* Pressure type */
    0x00,    /* Value in 0.1 bar */
    0x02,    /* Temperature type */
    0x00,    /* Value in C */
};

static char device_name[DEVICE_NAME_LENGTH];
static struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name) - 1),
    BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data))
};

static int16_t pressure = 0, temperature = 0;
static int sleep_counter = 0;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/



/******************************************************************************/
/*                                FUNCTIONS                                   */
/******************************************************************************/



/******************************************************************************/

/*!
 * @brief  This callback is called after BT ready
 */
static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    /* Update device name included mac address */
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    sprintf(device_name, "%s-%02X%02X", CONFIG_BT_DEVICE_NAME, addr.a.val[4], addr.a.val[5]);
    LOG_INF("Device name %s", device_name);
}

int main(void) {
    int err;
    LOG_INF("Power up! Firmware version %s", "1.0.0");
    k_msleep(100);    /* 1 seconds */

    /* Need to initialize lis2dh12tr to pull up INT1 pin */
    if (lis2dh12tr_init() != 0) {
        LOG_INF("Failed to initialize lis2dh12tr. Restart system");
        k_msleep(100);
        sys_reboot(SYS_REBOOT_WARM);
    }

    /* Configure to generate PORT event (wakeup) on buttons press. */
    nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_HIGH);

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    /* the sensors */
    tk2700_init();

    /* Loop */
    while (1) {
        /* Start advertising */
        LOG_INF("Start advertising!!!");
        err = bt_le_adv_start(ADV_PARAM, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
        if (err) {
            LOG_ERR("Advertising failed to start (err %d)", err);
            sys_reboot(SYS_REBOOT_WARM);
        }

        sleep_counter = 0;
        while (sleep_counter <= ADV_INTERVAL_S) {
            /* Measure and update adv data */
            if ((sleep_counter % MEASURE_INTERVAL_S) == 0) {
                if (tk2700_read_results(&pressure, &temperature) == 0) {
                    if (pressure > MAX_PRESSURE) {
                        pressure = MAX_PRESSURE;
                    }
                    service_data[PRES_INDEX] = pressure / 10;                  /* kPa to 0.1bar */
                    service_data[TEMP_INDEX] = (int8_t) (temperature / 10);    /* C to 0.1C */
                    LOG_INF("Temperature %d, pressure %d", temperature, pressure);

                    /* Update adv */
                    err = bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
                    if (err) {
                        LOG_ERR("Failed to update advertising data (err %d)", err);
                    }
                }
            }

            sleep_counter++;
            k_msleep(1000);    /* 1 seconds */

            /* If have interrupt, we reset timeout counter */
            if (lis2dh12tr_wait_interrupt(K_NO_WAIT) == 0) {
                sleep_counter = 0;
                LOG_INF("Sleep after %d seconds", ADV_INTERVAL_S);
            }
        }

        LOG_INF("Go to sleep...");
        k_msleep(100);    /* 100ms before sleeping */
        sys_poweroff();
    }
}