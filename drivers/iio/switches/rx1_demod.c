/*
 * RX1 demodulator control
 *
 * Copyright 2015 GR UPM
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/io.h>
#define NUM_CHANNELS 1

struct rx1_demod_platform_data {
    unsigned long *base_addr;
    struct resource *res;
    unsigned long remap_size;
};

struct rx1_demod_state {
    unsigned int frequency;
    unsigned int bandwidth;
    unsigned int modulation;
    struct rx1_demod_platform_data* pdata;
};

static ssize_t rx1_demod_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_demod_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1200000000) ret = -EINVAL;
            else {
                st->frequency = readin;
                regval = st->frequency;
            }
            break;
        case 1:
            if (readin < 0 || readin > 4)ret = -EINVAL;
            else {
                st->bandwidth = readin;
                regval = st->bandwidth;
            }
            break;
        case 2:
            if (readin < 0 || readin > 3)ret = -EINVAL;
            else {
                st->modulation = readin;
                regval = st->modulation;
            }
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
        iowrite32(regval, (st->pdata->base_addr) + (u32) private);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx1_demod_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_demod_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            ret = sprintf(buf, "%d\n", st->frequency);
            break;
        case 1:
            ret = sprintf(buf, "%d\n", st->bandwidth);
            break;
        case 2:
            ret = sprintf(buf, "%d\n", st->modulation);
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

#define _RX1_EXT_INFO(_name, _ident, _write, _read) { \
	.name = _name, \
	.read = _read, \
	.write = _write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info rx1_demod_ext_info[] = {
    _RX1_EXT_INFO("frequency", 0, rx1_demod_write, rx1_demod_read),
    _RX1_EXT_INFO("bandwidth", 1, rx1_demod_write, rx1_demod_read),
    _RX1_EXT_INFO("modulation", 2, rx1_demod_write, rx1_demod_read), {},
};

static const struct iio_chan_spec rx1_demod_chan[NUM_CHANNELS] = {
    {.type = IIO_ALTVOLTAGE,
        .channel = 0,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_demod_ext_info,},
};

static const struct iio_info rx1_demod_info = {
    .driver_module = THIS_MODULE,
};

static int rx1_demod_probe(struct platform_device *pdev) {
    struct rx1_demod_platform_data *pdata;
    struct iio_dev *indio_dev;
    struct rx1_demod_state *st;
    int ret = 0;

    pdata = devm_kzalloc(&pdev->dev, sizeof (*pdata), GFP_KERNEL);
    if (!pdata) {
        dev_err(&pdev->dev, "could not allocate memory for platform data\n");
        return -EINVAL;
    }
    pdata->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!pdata->res) {
        dev_err(&pdev->dev, "No memory resource\n");
        return -ENODEV;
    }

    pdata->remap_size = pdata->res->end - pdata->res->start + 1;
    if (!request_mem_region(pdata->res->start, pdata->remap_size, pdev->name)) {
        dev_err(&pdev->dev, "Cannot request IO\n");
        return -ENXIO;
    }

    pdata->base_addr = ioremap(pdata->res->start, pdata->remap_size);
    if (pdata->base_addr == NULL) {
        dev_err(&pdev->dev, "Couldn't ioremap memory at 0x%08lx\n",
                (unsigned long) pdata->res->start);
        ret = -ENOMEM;
        goto err_release_region;
    }

    indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof (*st));
    if (indio_dev == NULL) {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "failed to allocate indio_dev\n");
        goto err_alloc_indio;
    }
    st = iio_priv(indio_dev);
    platform_set_drvdata(pdev, indio_dev);
    st->pdata = pdata;
    indio_dev->dev.parent = NULL;
    indio_dev->name = "rx1_demod";
    indio_dev->info = &rx1_demod_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = rx1_demod_chan;
    indio_dev->num_channels = NUM_CHANNELS;
    ret = iio_device_register(indio_dev);
    st->frequency = 200000000;
    st->bandwidth = 0;
    st->modulation = 0;
    return 0;
err_alloc_indio:
    iounmap(pdata->base_addr);
err_release_region:
    release_mem_region(pdata->res->start, pdata->remap_size);
    return ret;
}

static int rx1_demod_remove(struct platform_device *pdev) {
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct rx1_demod_state *st = iio_priv(indio_dev);
    iounmap(st->pdata->base_addr);
    release_mem_region(st->pdata->res->start, st->pdata->remap_size);
    iio_device_unregister(indio_dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rx1_demod_dt_match[] = {
    { .compatible = "gr,rx1_demod_config"},
    {},
};
MODULE_DEVICE_TABLE(of, rx1_demod_dt_match);
#endif

static struct platform_driver rx1_demod_driver = {
    .driver =
    {
        .name = "rx1_demod_config",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rx1_demod_dt_match),
    },
    .probe = rx1_demod_probe,
    .remove = rx1_demod_remove,
};



module_platform_driver(rx1_demod_driver);

MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_DESCRIPTION("RX1 DEMOD DRIVER ");
MODULE_LICENSE("GPL v2");

