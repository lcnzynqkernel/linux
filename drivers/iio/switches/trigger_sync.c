/*
 * rx2_rfio RF switch
 *
 * Copyright 2016 GR UPM
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
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/div64.h>
#include <asm/io.h>

#define NUM_CHANNELS 4
#define MAX_CAPTURE_SIZE 1000000

struct trigger_sync_platform_data {
    unsigned long *base_addr;
    struct resource *res;
    unsigned long remap_size;
};

struct trigger_sync_state {
	unsigned int start_capture;
    unsigned int capture_delay;
	unsigned int enable_trigger;
	unsigned int pci_rst;
	unsigned int ms_config;
	unsigned int capture_samples;
	unsigned int lmk04281_ld1;
	unsigned int lmk04281_ld2;
	unsigned int mainol_ld;
	unsigned int lfol_ld;
	unsigned int scol_ld;
    struct trigger_sync_platform_data * pdata;
};

static ssize_t capture_sync_status_write(struct iio_dev *indio_dev,
    
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {
    unsigned long long readin;
    struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else st->start_capture = readin;
            break;
	    case 1:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else st->pci_rst = readin;
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
		regval |= st->start_capture;
		regval |= st->pci_rst << 7;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t capture_sync_status_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			ret = sprintf(buf, "%d\n", st->start_capture);
			break;
		case 1:
			ret = sprintf(buf, "%d\n", st->pci_rst);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t capture_sync_cfg_write(struct iio_dev *indio_dev,
    
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {
    unsigned long long readin;
    struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else st->ms_config = readin;
            break;
        case 1:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->enable_trigger = readin;
            break;
        case 2:
            if (readin < 0 || readin > 15)ret = -EINVAL;
            else st->capture_delay = readin;            
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
		regval |= st->capture_delay;
		regval |= st->enable_trigger << 4;
		regval |= st->ms_config << 7;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t capture_sync_cfg_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			ret = sprintf(buf, "%s\n", st->ms_config ? "SLAVE" : "MASTER");
			break;
		case 1:
			ret = sprintf(buf, "%s\n", st->enable_trigger ? "ENABLE" : "DISABLE");
			break;
		case 2:
			ret = sprintf(buf, "%d\n", st->capture_delay);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t capture_sync_samples_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
		unsigned long long readin;
    struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > MAX_CAPTURE_SIZE)
        ret = -EINVAL;
    else {
        regval = (u32) readin;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
        st->capture_samples = readin;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t capture_sync_samples_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
  struct trigger_sync_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    ret = sprintf(buf, "%d\n", st->capture_samples);
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t trigger_sync_lock_detect_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
		struct trigger_sync_state *st = iio_priv(indio_dev);
		unsigned int regval = 0;
		int ret = 0;

		mutex_lock(&indio_dev->mlock);
		regval = (u32) ioread32((st->pdata->base_addr) + chan->channel);

		switch ((u32) private) {
			case 0:
				st->lmk04281_ld1 = regval & 0x1;
				ret = sprintf(buf, "%s\n", st->lmk04281_ld1 ? "Locked" : "Not locked");
				break;
			case 1:
				st->lmk04281_ld2 = (regval & 0x2)>>1;	
				ret = sprintf(buf, "%s\n", st->lmk04281_ld2 ? "Locked" : "Not locked");
				break;
			case 2:
				st->mainol_ld = (regval & 0x4)>>2;	
				ret = sprintf(buf, "%s\n", st->mainol_ld ? "Locked" : "Not locked");
				break;
			case 3:
				st->lfol_ld = (regval & 0x8)>>3;	
				ret = sprintf(buf, "%s\n", st->lfol_ld ? "Locked" : "Not locked");
				break;
			case 4:
				st->scol_ld = (regval & 0x16)>>4;	
				ret = sprintf(buf, "%s\n", st->scol_ld ? "Locked" : "Not locked");
				break;
			default:
				ret = -EINVAL;
		}
		mutex_unlock(&indio_dev->mlock);
		return ret;
}

#define _TRIGGER_SYNC_EXT_INFO(_name, _ident, _write, _read) { \
	.name = _name, \
	.read = _read, \
	.write = _write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info trigger_sync_capture_status_ext_info[] = {
    _TRIGGER_SYNC_EXT_INFO("status", 0, capture_sync_status_write, capture_sync_status_read),  	//0
 	_TRIGGER_SYNC_EXT_INFO("pci_rst", 1, capture_sync_samples_write, capture_sync_samples_read), 
    {},
};

static const struct iio_chan_spec_ext_info trigger_sync_capture_cfg_ext_info[] = {
	_TRIGGER_SYNC_EXT_INFO("ms_mode", 0, capture_sync_cfg_write, capture_sync_cfg_read), //7
	_TRIGGER_SYNC_EXT_INFO("enable", 1, capture_sync_cfg_write, capture_sync_cfg_read),  //4
    _TRIGGER_SYNC_EXT_INFO("delay", 2, capture_sync_cfg_write, capture_sync_cfg_read),   //3-0
	{},
};

static const struct iio_chan_spec_ext_info trigger_sync_capture_samples_ext_info[] = {
    _TRIGGER_SYNC_EXT_INFO("capture_samples", 0, capture_sync_samples_write, capture_sync_samples_read), 
	{}, //32 
};

static const struct iio_chan_spec_ext_info trigger_sync_lock_detect_ext_info[] = {
    _TRIGGER_SYNC_EXT_INFO("lmk04281_ld1", 0, NULL, trigger_sync_lock_detect_read),  //0
	_TRIGGER_SYNC_EXT_INFO("lmk04281_ld2", 1, NULL, trigger_sync_lock_detect_read), 	//1
    _TRIGGER_SYNC_EXT_INFO("mainol_ld", 2, NULL, trigger_sync_lock_detect_read),     //2
    _TRIGGER_SYNC_EXT_INFO("lfol_ld", 3, NULL, trigger_sync_lock_detect_read), 		//3
    _TRIGGER_SYNC_EXT_INFO("scol_ld", 4, NULL, trigger_sync_lock_detect_read), 		//74
    {},
};

static const struct iio_chan_spec trigger_sync_chan[NUM_CHANNELS] = {
    {.type = IIO_ALTVOLTAGE,
        .channel = 0,
        .indexed = 1,
        .output = 1,
        .ext_info = trigger_sync_capture_cfg_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 1,
        .indexed = 1,
        .output = 1,
        .ext_info = trigger_sync_capture_status_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 2,
        .indexed = 1,
        .output = 1,
        .ext_info = trigger_sync_capture_samples_ext_info,},
 	{.type = IIO_ALTVOLTAGE,
        .channel = 3,
        .indexed = 1,
        .output = 0,
        .ext_info = trigger_sync_lock_detect_ext_info,},
};


static const struct iio_info  trigger_sync_info = {
    .read_raw = NULL,
    .write_raw = NULL,
    .driver_module = THIS_MODULE,
};

static int trigger_sync_probe(struct platform_device *pdev) {
    struct trigger_sync_platform_data *pdata;
    struct iio_dev *indio_dev;
    struct trigger_sync_state *st;
    int ret = 0;
	unsigned char regval = 0;

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
    indio_dev->name = "TRIGGER_SYNC";
    indio_dev->info = &trigger_sync_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = trigger_sync_chan;
    indio_dev->num_channels = NUM_CHANNELS;
    ret = iio_device_register(indio_dev);

	st->start_capture = 0;
	st->capture_delay = 0;
	st->enable_trigger = 0;
	st->capture_samples = 0;
	st->pci_rst = 0;

	//regval = 0;
    //iowrite32(regval, (st->pdata->base_addr));

    return 0;
err_alloc_indio:
    iounmap(pdata->base_addr);
err_release_region:
    release_mem_region(pdata->res->start, pdata->remap_size);
    return ret;
}

static int trigger_sync_remove(struct platform_device *pdev) {
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct trigger_sync_state *st = iio_priv(indio_dev);
    iounmap(st->pdata->base_addr);
    release_mem_region(st->pdata->res->start, st->pdata->remap_size);
    iio_device_unregister(indio_dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id trigger_sync_dt_match[] = {
    { .compatible = "gr,trigger_sync"},
    {},
};
MODULE_DEVICE_TABLE(of, trigger_sync_dt_match);
#endif

static struct platform_driver trigger_sync_driver = {
    .driver =
    {
        .name = "trigger_sync",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(trigger_sync_dt_match),
    },
    .probe = trigger_sync_probe,
    .remove = trigger_sync_remove,
};

module_platform_driver(trigger_sync_driver);

MODULE_AUTHOR("Luis Cuéllar  <luiscn@gr.ssr.upm.es>");
MODULE_DESCRIPTION("TRIGGER SYNC DRIVER ");
MODULE_LICENSE("GPL v2");

