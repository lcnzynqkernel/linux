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
#define NUM_CHANNELS 8
#define GAIN_STEPS 37

#define FMC_PLL_STATUS		1
#define FMC_OL_LD		(1 << 1)
#define FMC_RF_ALERT		(1 << 2)
#define FMC_TEMP_ALERT		(1 << 3)
#define FMC_PWR_VALID		(1 << 4)
#define FMC_PWR_ALERT		(1 << 5)

#define ADC_CLK				24576000000UL
//#define DDS_DDC_ACC			32
//#define DDS_DUC_ACC			28
#define MAX_FREQ_DDS_DDC	960000
#define MAX_FREQ_DDS_DUC	50000000

const unsigned int gaincodes[37] = {
    1024, 1149, 1289, 1446, 1623, 1821,
    2043, 2292, 2572, 2886, 3238, 3633,
    4077, 4574, 5132, 5758, 6461, 7249,
    8134, 9126, 10240, 11489, 12891, 14464,
    16229, 18210, 20431, 22925, 25722, 28860,
    32382, 36333, 40766, 45740, 51322, 57584,
    64610
};

const unsigned int gaincodes_out[73] = { 
 0, 4,  4,   5,   5,  6,  6,  7,  8,  9, 10, 11, 13, 14, 16,
 18,  20,  23,  26, 29, 32, 36, 41, 46, 51, 58, 65, 72, 81,
 91, 102, 115, 129,145,162,182,204,229,257,289,324,363,408,
457,513,576,646,725,813,913,1024,1149,1289,1446,1623,1821,2043,
2292,2572,2886,3238,3633,4077,4574,5132,5758,6461,7249,8134,9126,10240,
11489, 12891
};

struct rx2_platform_data {
    unsigned long *base_addr;
    struct resource *res;
    unsigned long remap_size;
};

struct rx2_state {
	unsigned int ddcbypass;
    unsigned int stage2mode;
	unsigned int mixerbyp;
    unsigned int mixerpol;
	unsigned int ducenable;
    unsigned int dcfilter;
    unsigned int ddcfrequency;
    unsigned int ifingain;
    unsigned int ifingaindb;
    unsigned int ifoutgain;
    unsigned int ifoutgaindb;
    unsigned int iffrequency;
    unsigned int ifPeakAmplitude;
	unsigned int lmk04281_ld1;
	unsigned int lmk04281_ld2;
	unsigned int mainol_ld;
	unsigned int lfol_ld;
	unsigned int scol_ld;
    struct rx2_platform_data* pdata;
};

static ssize_t rx2_ddc_write(struct iio_dev *indio_dev,
    
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {
    unsigned long long readin;
    struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else st->ddcbypass = readin;
            break;
        case 1:
            if (readin < 0 || readin > 4)ret = -EINVAL;
            else st->stage2mode = readin;
            break;
        case 2:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->mixerbyp = readin;            
            break;
        case 3:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->mixerpol = readin;            
            break;
        case 4:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->ducenable = readin;
            break;
        case 5:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->dcfilter = readin;
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
		regval |= st->ddcbypass;
        regval |= (st->stage2mode < 5 ? st->stage2mode : 4) << 1;
        regval |= st->mixerbyp << 5;
		regval |= st->mixerpol << 6;
		regval |= st->ducenable << 7;
		regval |= st->dcfilter << 8;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx2_ddc_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			ret = sprintf(buf, "%s\n", st->ddcbypass & 0x01 ? "Bypass" : "Enabled");
			break;
        case 1:
            switch (st->stage2mode) {
                case 0:
                    ret = sprintf(buf, "%s\n", "DDC2bypass");
                    break;
                case 1:
                    ret = sprintf(buf, "%s\n", "340kHz");
                    break;
                case 2:
                    ret = sprintf(buf, "%s\n", "240kHz");
                    break;
                case 3:
                    ret = sprintf(buf, "%s\n", "100kHz");
                    break;
                case 4:
                    ret = sprintf(buf, "%s\n", "30kHz");
                    break;
      			case 5:
                    ret = sprintf(buf, "%s\n", "15kHz");
                    break;
            }
            break;
        case 2:
            ret = sprintf(buf, "%s\n", st->mixerbyp ? "Bypass" : "Enabled");
            break;
        case 3:
            ret = sprintf(buf, "%s\n", st->mixerpol ? "Positive" : "Negative");
            break;
        case 4:
            ret = sprintf(buf, "%s\n", st->ducenable ? "Enabled" : "Disabled");
            break;
        case 5:
            ret = sprintf(buf, "%s\n", st->dcfilter ? "Enabled" : "Disabled");
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx2_ddcfreq_write(struct iio_dev *indio_dev,
    uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {
    unsigned long long readin;
    struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > MAX_FREQ_DDS_DDC)
        ret = -EINVAL;
    else {
        regval = ((u32) readin * 4096) / 1875;
        st->ddcfrequency = ( unsigned int ) (regval * 1875) / 4096;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx2_ddcfreq_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
		struct rx2_state *st = iio_priv(indio_dev);
		int ret = 0;
		mutex_lock(&indio_dev->mlock);
		ret = sprintf(buf, "%d\n", st->ddcfrequency);
		mutex_unlock(&indio_dev->mlock);
		return ret;
}


static ssize_t rx2_lock_detect_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
		struct rx2_state *st = iio_priv(indio_dev);
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

static ssize_t rx2_if_out_freq_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
		unsigned long long readin;
    struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > MAX_FREQ_DDS_DUC)
        ret = -EINVAL;
    else {
        regval = ((u32) readin / 1875);
        regval *= 2048;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
       // regval /= 2048;
        st->iffrequency = readin; //regval * 1875;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx2_if_out_freq_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
  struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    ret = sprintf(buf, "%d\n", st->iffrequency);
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx2_ifout_gain_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len){

	unsigned long long readin;
	unsigned int code;
    struct rx2_state *st = iio_priv(indio_dev);
    int ret = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
	if (readin < 0 || readin > 73)
		    ret = -EINVAL;
	else{
	    st->ifoutgaindb = readin;
	    code = gaincodes_out[(u8) readin]; //integer part of gain	
		st->ifoutgain = code;
		iowrite32(st->ifoutgain, (st->pdata->base_addr) + chan->channel);
	}   
	mutex_unlock(&indio_dev->mlock);
	return ret ? ret : len;
}
   
static ssize_t rx2_ifout_gain_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
		struct rx2_state *st = iio_priv(indio_dev);
		int ret = 0;

		mutex_lock(&indio_dev->mlock);
		ret = sprintf(buf, "%d\n", st->ifoutgaindb);
		mutex_unlock(&indio_dev->mlock);

		return ret;
}

static ssize_t rx2_ifamp_measure_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
		struct rx2_state *st = iio_priv(indio_dev);
		unsigned int regval = 0;
		int ret = 0;

		mutex_lock(&indio_dev->mlock);
		regval = (u32) ioread32((st->pdata->base_addr) + chan->channel);
		st->ifPeakAmplitude = regval & 0x00FFFFFF;
		ret = sprintf(buf, "%d\n", st->ifPeakAmplitude);
		mutex_unlock(&indio_dev->mlock);
		return ret;
}


static int rx2_ifgain_write_raw
		(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan,
        int val,
        int val2,
		long mask) {

    struct rx2_state *st = iio_priv(indio_dev);
    unsigned int code;
    int ret = 0;
    if (val < 0 || val > 36 || val2 < 0)
        return -EINVAL;
    code = gaincodes[(u8) val]; //integer part of gain
    mutex_lock(&indio_dev->mlock);
    switch (mask) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
            st-> ifingain = code;
            st->ifingaindb = val;
            iowrite32(st->ifingain, (st->pdata->base_addr) + chan->channel);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static int rx2_ifgain_read_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan,
        int *val,
        int *val2,
        long m) {
	struct rx2_state *st = iio_priv(indio_dev);
    int ret;
    mutex_lock(&indio_dev->mlock);
    switch (m) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
            *val = st->ifingaindb;
            *val2 = 0;
            ret = IIO_VAL_INT_PLUS_MICRO_DB;
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);

    return ret;
};



#define _RX2_EXT_INFO(_name, _ident, _write, _read) { \
	.name = _name, \
	.read = _read, \
	.write = _write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info rx2_ddc_ext_info[] = {
    _RX2_EXT_INFO("ddcbypass", 0, rx2_ddc_write, rx2_ddc_read),  	//0
	_RX2_EXT_INFO("stage2mode", 1, rx2_ddc_write, rx2_ddc_read), 	//1-4
    _RX2_EXT_INFO("mixerbyp", 2, rx2_ddc_write, rx2_ddc_read), 		//5
    _RX2_EXT_INFO("mixerpol", 3, rx2_ddc_write, rx2_ddc_read), 		//6
    _RX2_EXT_INFO("ducenable", 4, rx2_ddc_write, rx2_ddc_read), 	//7
	_RX2_EXT_INFO("dcfilter", 5, rx2_ddc_write, rx2_ddc_read), 		//8
    {},
};

static const struct iio_chan_spec_ext_info rx2_ddcfreq_ext_info[] = {
    _RX2_EXT_INFO("ddcfrequency", 0, rx2_ddcfreq_write, rx2_ddcfreq_read), {}, //24
};

static const struct iio_chan_spec_ext_info rx2_if_freq_ext_info[] = {
    _RX2_EXT_INFO("if_output_frequency", 0, rx2_if_out_freq_write, rx2_if_out_freq_read), {}, //24 
};

static const struct iio_chan_spec_ext_info rx2_ifamp_measure_ext_info[] = {
    _RX2_EXT_INFO("ddc_st1_amp_peak", 0, NULL, rx2_ifamp_measure_read), {}, //24
};


static const struct iio_chan_spec_ext_info rx2_ifout_gain_ext_info[] = {
    _RX2_EXT_INFO("if_output_gain", 0, rx2_ifout_gain_write, rx2_ifout_gain_read), {}, 
};

static const struct iio_chan_spec_ext_info rx2_lock_detect_ext_info[] = {
    _RX2_EXT_INFO("lmk04281_ld1", 0, NULL, rx2_lock_detect_read),  	//0
	_RX2_EXT_INFO("lmk04281_ld2", 1, NULL, rx2_lock_detect_read), 	//1
    _RX2_EXT_INFO("mainol_ld", 2, NULL, rx2_lock_detect_read), 		//2
    _RX2_EXT_INFO("lfol_ld", 3, NULL, rx2_lock_detect_read), 		//3
    _RX2_EXT_INFO("scol_ld", 4, NULL, rx2_lock_detect_read), 		//74
    {},
};

static const struct iio_chan_spec rx2_chan[NUM_CHANNELS] = {
    {.type = IIO_ALTVOLTAGE,
        .channel = 0,
        .indexed = 1,
        .output = 1,
        .ext_info = rx2_ddc_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 1,
        .indexed = 1,
        .output = 1,
        .ext_info = rx2_ddcfreq_ext_info,},
    {.type = IIO_VOLTAGE,
        .channel = 2,
        .indexed = 1,
        .output = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),},
    {.type = IIO_ALTVOLTAGE,
        .channel = 3,
        .indexed = 1,
        .output = 1,
        .ext_info = rx2_if_freq_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 4,
        .indexed = 1,
        .output = 1,
        .ext_info = rx2_ifout_gain_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 5,
        .indexed = 1,
        .output = 0,
        .ext_info = rx2_ifamp_measure_ext_info,},
 	{.type = IIO_ALTVOLTAGE,
        .channel = 7,
        .indexed = 1,
        .output = 0,
        .ext_info = rx2_lock_detect_ext_info,},
};

static const struct iio_info rx2_info = {
    .read_raw = &rx2_ifgain_read_raw,
    .write_raw = &rx2_ifgain_write_raw,
    .driver_module = THIS_MODULE,
};

static int rx2_probe(struct platform_device *pdev) {
    struct rx2_platform_data *pdata;
    struct iio_dev *indio_dev;
    struct rx2_state *st;
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
    indio_dev->name = "RX2_CONFIG";
    indio_dev->info = &rx2_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = rx2_chan;
    indio_dev->num_channels = NUM_CHANNELS;
    ret = iio_device_register(indio_dev);

	st->ddcbypass = 1;
	st->stage2mode = 0;
	st->mixerbyp = 1;
	st->mixerpol = 0;
	st->ducenable  = 0;
	st->dcfilter = 0;

	regval = 0;
	regval |= st->ddcbypass;
    regval |= (st->stage2mode < 5 ? st->stage2mode : 4) << 1;
    regval |= st->mixerbyp << 5;
	regval |= st->mixerpol << 6;
	regval |= st->ducenable << 7;
	regval |= st->dcfilter << 8;
    iowrite32(regval, (st->pdata->base_addr));

	st->ddcfrequency=480000;
	st->iffrequency=10700000;
	st->ifoutgaindb = 36;
	st->ifoutgain = gaincodes[st->ifoutgaindb];
	//regval = st->ifoutgain;
    //iowrite32(regval, (st->pdata->base_addr) + 4);

    return 0;
err_alloc_indio:
    iounmap(pdata->base_addr);
err_release_region:
    release_mem_region(pdata->res->start, pdata->remap_size);
    return ret;
}

static int rx2_remove(struct platform_device *pdev) {
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct rx2_state *st = iio_priv(indio_dev);
    iounmap(st->pdata->base_addr);
    release_mem_region(st->pdata->res->start, st->pdata->remap_size);
    iio_device_unregister(indio_dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rx2_dt_match[] = {
    { .compatible = "gr,rx2_config"},
    {},
};
MODULE_DEVICE_TABLE(of, rx2_dt_match);
#endif

static struct platform_driver rx2_driver = {
    .driver =
    {
        .name = "rx2_config",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rx2_dt_match),
    },
    .probe = rx2_probe,
    .remove = rx2_remove,
};



module_platform_driver(rx2_driver);

MODULE_AUTHOR("Luis Cuéllar  <luiscn@gr.ssr.upm.es>");
MODULE_DESCRIPTION("RX2 CONFIG DRIVER ");
MODULE_LICENSE("GPL v2");

