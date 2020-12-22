/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "bma4_common.h"

#if 0
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

#if 1
#define ERROR_PRINTF(...) printf(__VA_ARGS__)
#else
#define ERROR_PRINTF(...)
#endif


/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of I2C bus.
 */
int8_t user_i2c_init(void)
{
    DEBUG_PRINTF("user_i2c_init: ENTER\r\n");

    /* Implement I2C bus initialization according to the target machine. */
    i2c_set_freq(100000); // 100KHz

    DEBUG_PRINTF("user_i2c_init: EXIT\r\n");
    return 0;
}

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through SPI bus.
 */
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI read routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through I2C bus.
 * returns 0 if success else -1
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    DEBUG_PRINTF("user_i2c_read: ENTER\r\n");
    DEBUG_PRINTF("user_i2c_read: reg_addr=0x%x, len=%d\r\n", reg_addr, length);


    /* Read from registers using I2C. Return 0 for a successful execution. */

    int ret = BMA4_OK;

    const uint8_t i2c_addr = *(uint8_t *)intf_ptr;
    DEBUG_PRINTF("user_i2c_read: i2c_addr=0x%x\r\n", i2c_addr);

    ret = i2c_wr(i2c_addr << 1, (const char *)&reg_addr, 1, FALSE);
    if (ret != BMA4_OK) // Failed writing
    {
        ERROR_PRINTF("user_i2c_read: Read Failed writing address, returned %i\r\n", ret);
    }

    ret = i2c_rd(i2c_addr << 1, (char *)reg_data, length, FALSE);
    if (ret != BMA4_OK) // Failed reading
    {
        ERROR_PRINTF("user_i2c_read: Read Failed reading , returned %i\r\n", ret);
    }

    return ret;
}

/*!
 * @brief Function for writing the sensor's registers through SPI bus.
 */
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI write routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    DEBUG_PRINTF("user_i2c_write: ENTER\r\n");
    DEBUG_PRINTF("user_i2c_write: reg_addr=0x%x, reg_data=0x%x, len=%d\r\n", reg_addr, *reg_data, length);

	uint8_t buffer[length + 1];

	buffer[0] = reg_addr;
	memcpy(&buffer[1], reg_data, length);

    int ret = BMA4_OK;

    if (NULL != intf_ptr)
    {
        const uint8_t device_addr = *(uint8_t *)intf_ptr;
        DEBUG_PRINTF("user_i2c_write: device_addr=0x%x\r\n", device_addr);

        // Write to I2C
        // Write register address
        // ret = i2c_wr(device_addr << 1, &reg_addr, 1, TRUE);
        // Write data
        ret = i2c_wr(device_addr << 1, buffer, length + 1, FALSE);
    }

    if (ret != BMA4_OK) // Failed writing
    {
        ERROR_PRINTF("user_i2c_write: Read Failed reading , returned %i\r\n", ret);
    }

    /* Implement the I2C write routine according to the target machine. */
    DEBUG_PRINTF("user_i2c_write: EXIT, ret=%d\r\n", ret);
    return ret;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay(uint32_t period_us, void *intf_ptr)
{
    delay_us(period_us);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bma4_interface_selection(struct bma4_dev *bma, uint8_t variant)
{
    DEBUG_PRINTF("bma4_interface_selection: ENTER\r\n");

    int8_t rslt = BMA4_OK;

    if (bma != NULL)
    {
        /* Select the interface for execution
         * For I2C : BMA4_I2C_INTF
         * For SPI : BMA4_SPI_INTF
         */
        bma->intf = BMA4_I2C_INTF; // Default to I2C

        /* Bus configuration : I2C */
        if (bma->intf == BMA4_I2C_INTF)
        {
            DEBUG_PRINTF("I2C Interface \n\r");

            /* To initialize the user I2C function */
            user_i2c_init();
            dev_addr = BMA4_I2C_ADDR_PRIMARY;
            bma->bus_read = user_i2c_read;
            bma->bus_write = user_i2c_write;
        }

        /* Bus configuration : SPI */
        else if (bma->intf == BMA4_SPI_INTF)
        {
            DEBUG_PRINTF("SPI Interface \n\r");

            /* To initialize the user SPI function */
            user_spi_init();
            dev_addr = 0;
            bma->bus_read = user_spi_read;
            bma->bus_write = user_spi_write;
        }

        /* Assign variant parameter */
        bma->variant = variant;

        /* Assign device address to interface pointer */
        bma->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma->delay_us = user_delay;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bma->read_write_len = 8;
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    DEBUG_PRINTF("bma4_interface_selection: EXIT\r\n");
    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma4_error_codes_print_result(const char api_name[], uint16_t rslt)
{
    if (rslt != BMA4_OK)
    {
        printf("%s\t", api_name);
        if (rslt & BMA4_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt & BMA4_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Invalid configuration stream\r\n", rslt);
        }
        else if (rslt & BMA4_E_SELF_TEST_FAIL)
        {
            printf("Error [%d] : Self test failed\r\n", rslt);
        }
        else if (rslt & BMA4_E_INVALID_SENSOR)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
