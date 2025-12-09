/*
 * $QNXLicenseC:
 * Copyright 2020,2025 QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "proto.h"

bcm2711_dev_t dev_st = { 0 };
#ifdef GPIO_CONFIG

#define GPIO_FSEL0_OFF      0x00
#define GPIO_PULL0_OFF      0xe4
#define GPIO_PULL_UP        0x65  /*101b for 2 ports*/

enum gpio_fsel {
    FSEL_INPUT  = 0,
    FSEL_OUTPUT = 1,
    FSEL_FUN0   = 4,
    FSEL_FUN1   = 5,
    FSEL_FUN2   = 6,
    FSEL_FUN3   = 7,
    FSEL_FUN4   = 3,
    FSEL_FUN5   = 2,
};

typedef struct _gpio_info {
    uint32_t i2c_addr;
    uint8_t gpio_sda;
    uint32_t fsel;
} gpio_info_t;


static int gpio_init(bcm2711_dev_t *dev)
{
    gpio_info_t gpio_tbl[] = {
        {BCM2711_I2C0_BASE,  0, FSEL_FUN0 },
        {BCM2711_I2C1_BASE,  2, FSEL_FUN0 },
        {BCM2711_I2C3_BASE,  2, FSEL_FUN5 },
        {BCM2711_I2C3_BASE,  4, FSEL_FUN5 },
        {BCM2711_I2C4_BASE,  6, FSEL_FUN5 },
        {BCM2711_I2C4_BASE,  8, FSEL_FUN5 },
        {BCM2711_I2C5_BASE, 10, FSEL_FUN5 },
        {BCM2711_I2C5_BASE, 12, FSEL_FUN5 },
        {BCM2711_I2C6_BASE,  0, FSEL_FUN5 },
        {BCM2711_I2C6_BASE, 22, FSEL_FUN5 }
    };

    volatile uintptr_t gpio_base;
    uint32_t reg_val = 0;
    uint8_t i = 0, num_items = 0;
    volatile uintptr_t reg_base;
    uint32_t mask, shift;

    gpio_base = mmap_device_io(BCM2711_GPIO_SIZE, BCM2711_GPIO_BASE);
    if (gpio_base == (uintptr_t)MAP_FAILED) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR,
            "%s : GPIO mmap_device_io failed", __FUNCTION__);
        return EXIT_FAILURE;
    }

    num_items = sizeof(gpio_tbl)/sizeof(gpio_info_t);
    for (i = 0; i < num_items; i++) {
        if ((dev->physbase == gpio_tbl[i].i2c_addr) &&
            (dev->gpio == gpio_tbl[i].gpio_sda)) {
            break;
        }
    }

    if (i >= num_items) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR,
            "%s : Failed to find I2C port: phy addr 0x%x, GPIO SDA %d",
             __FUNCTION__, dev->physbase, dev->gpio);
        munmap_device_io(gpio_base, BCM2711_GPIO_SIZE);
        return EXIT_FAILURE;
    }

    /* Set alternative functions*/
    shift = 3 * (dev->gpio % 10);
    mask = 0x3f << shift;
    reg_base = gpio_base + GPIO_FSEL0_OFF + 4 * ((uint8_t)(dev->gpio / 10));
    reg_val = (uint32_t)in32(reg_base);
    reg_val &= ~mask;
    reg_val |= (gpio_tbl[i].fsel << shift) | (gpio_tbl[i].fsel << (shift + 3));
    out32(reg_base, reg_val);

    /* pull both SDA and SCL up*/
    shift = 2 * (dev->gpio % 16);
    mask = 0x0f << shift;
    reg_base = gpio_base + GPIO_PULL0_OFF + 4 * ((uint8_t)(dev->gpio / 16));
    reg_val = (uint32_t)in32(reg_base);
    reg_val &= ~mask;
    reg_val |= GPIO_PULL_UP << shift;
    out32(reg_base, reg_val);

    if (munmap_device_io(gpio_base, BCM2711_GPIO_SIZE) < 0) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR,"%s : munmap_device_io failed %s",
                                        __FUNCTION__, strerror(errno));
        return EXIT_FAILURE;
    }

    return EOK;
}
#endif

void *bcm2711_init(int argc, char *argv[])
{
    bcm2711_dev_t *dev = &dev_st;
    pthread_condattr_t condattr;
    int32_t err = EOK;

    if (-1 == bcm2711_options(dev, argc, argv)) {
        goto fail0;
    }

    if (-1 == ThreadCtl(_NTO_TCTL_IO, 0)) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR, "ThreadCtl failed: %s", strerror(errno));
        goto fail0;
    }


#ifdef GPIO_CONFIG
    if (gpio_init(dev) != EOK) {
        goto fail0;
    }
#endif

    dev->regbase = mmap_device_io(dev->reglen, dev->physbase);
    if (dev->regbase == (uintptr_t)MAP_FAILED) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR, "mmap_device_io failed!");
        goto fail0;
    }

    i2c_reset(dev);

    (void)pthread_mutex_init(&dev->mutex, NULL);
    (void)pthread_condattr_init(&condattr);
    (void)pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC);
    err = pthread_cond_init(&dev->condvar, &condattr);
    if (err != EOK) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR, "failed to init condvar");
        (void)pthread_mutex_destroy(&dev->mutex);
        (void)pthread_condattr_destroy(&condattr);
        goto fail1;
    }

    (void)pthread_condattr_destroy(&condattr);

    err = pthread_create(NULL, NULL, &bcm2711_wait_thread, NULL);
    if (err != EOK) {
        bcm2711_i2c_slogf(dev, _SLOG_ERROR, "failed to create IST thread for irq %d (%s)",
                dev->intr, strerror(err));
        goto fail1;
    }
    return dev;

fail1:
    munmap_device_io(dev->regbase, dev->reglen);
fail0:
    bcm2711_i2c_slogf(dev, _SLOG_ERROR, "init failed!");
    return NULL;
}

void i2c_reset(bcm2711_dev_t *dev)
{
    disable_interrupt(dev);

    out32(dev->regbase + BCM2711_I2C_CTRREG_OFF, CTRREG_I2CEN | CTRREG_CLEAR);
    out32(dev->regbase + BCM2711_I2C_STATREG_OFF, STATREG_ERR | STATREG_CLKT | STATREG_DONE);
}

void disable_interrupt(bcm2711_dev_t *dev)
{
    out32(dev->regbase + BCM2711_I2C_CTRREG_OFF,
        in32(dev->regbase + BCM2711_I2C_CTRREG_OFF) & ~(CTRREG_INTR | CTRREG_INTT | CTRREG_INTD));
}
