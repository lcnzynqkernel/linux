/*
 * rx1_rfio RF switch
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
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/io.h>
#define NUM_CHANNELS 9
#define GAIN_STEPS 37

#define FMC_PLL_STATUS		1
#define FMC_OL_LD		(1 << 1)
#define FMC_RF_ALERT		(1 << 2)
#define FMC_TEMP_ALERT		(1 << 3)
#define FMC_PWR_VALID		(1 << 4)
#define FMC_PWR_ALERT		(1 << 5)

const unsigned int gaincodes[37] = {
    1024, 1149, 1289, 1446, 1623, 1821,
    2043, 2292, 2572, 2886, 3238, 3633,
    4077, 4574, 5132, 5758, 6461, 7249,
    8134, 9126, 10240, 11489, 12891, 14464,
    16229, 18210, 20431, 22925, 25722, 28860,
    32382, 36333, 40766, 45740, 51322, 57584,
    64610
};
char *colors[8] = {"off", "green", "red", "yellow", "blue", "cyan", "purple", "white"};

struct rx1_platform_data {
    unsigned long *base_addr;
    struct resource *res;
    unsigned long remap_size;
};

struct rx1_state {
    unsigned int selected;
    unsigned int rfen;
    unsigned int picrst;
    unsigned int rfshutdown;
    unsigned int pllfunction;
    unsigned int stage2mode;
    unsigned int stage1mode;
    unsigned int mixerpol;
    unsigned int mixerbyp;
    unsigned int ddcfrequency;
    unsigned int ifgain;
    unsigned int ifgaindb;
    unsigned int ledcolor;
    unsigned int pwrMeanCh0;
    unsigned int pwrMeanCh1;
    unsigned int pwrPeakCh0;
    unsigned int pwrPeakCh1;
    unsigned int PLLstatus;
    unsigned int OLlockDetect;
    unsigned int RFalert;
    unsigned int TempAlert;
    unsigned int PWRvalid;
    unsigned int PWRalert;
    unsigned int iffrequency;
    unsigned int ifPeakAmplitude;
    struct rx1_platform_data* pdata;
};

static ssize_t rx1_rfio_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 4) ret = -EINVAL;
            else st->selected = readin;
            break;
        case 1:
            st->rfen = readin ? 0 : 1;
            break;
        case 2:
            st->picrst = readin ? 1 : 0;
            break;
        case 3:
            st->rfshutdown = readin ? 1 : 0;
            break;
        case 4:
            st->pllfunction = readin ? 1 : 0;
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
        regval |= st->rfen;
        regval |= st->selected ? (st->selected) << 1 : 0xE;
        regval |= st->rfshutdown << 4;
        regval |= st->pllfunction << 5;
        regval |= st->picrst << 7;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx1_ddc_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval = 0;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 5) ret = -EINVAL;
            else {
                if (st->stage1mode & 0x01) { //Bypass enabled
                    st->stage2mode = 0;
                } else {
                    st->stage2mode = readin;
                }
            }
            break;
        case 1:
            if (readin < 0 || readin > 3)ret = -EINVAL;
            else {
                st->stage1mode = readin;
                if (st->stage1mode & 0x01) { //Bypass enabled
                    st->stage2mode = 0; //Bypass stage2
                    st->mixerbyp = 1; //Bypass mixer
                }
            }
            break;
        case 2:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else st->mixerpol = readin;
            break;
        case 3:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else {
                if (st->stage1mode & 0x01) { //Bypass enabled
                    st->mixerbyp = 1;
                } else {
                    st->mixerbyp = readin;
                }
            }
            break;
        default:
            ret = -EINVAL;
    }
    if (!ret) {
        regval |= st->stage2mode < 5 ? st->stage2mode : 4;
        regval |= st->stage1mode << 4;
        regval |= st->mixerpol << 6;
        regval |= st->mixerbyp << 7;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx1_ddcfreq_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > 960000)
        ret = -EINVAL;
    else {
        regval = ((u32) readin * 4096) / 1875;
        st->ddcfrequency = (regval * 1875) / 4096;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx1_led_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > 7)
        ret = -EINVAL;
    else {
        st->ledcolor = readin;
        iowrite32(readin, (st->pdata->base_addr) + chan->channel);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static ssize_t rx1_rfio_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            switch (st->selected) {
                case 0:
                    ret = sprintf(buf, "%s\n", "Disconnected");
                    break;
                case 1:
                    ret = sprintf(buf, "%s\n", "Test");
                    break;
                case 2:
                    ret = sprintf(buf, "%s\n", "1.2GHz-3GHz");
                    break;
                case 3:
                    ret = sprintf(buf, "%s\n", "50ohm load");
                    break;
                case 4:
                    ret = sprintf(buf, "%s\n", "20MHz-1.2GHz");
                    break;
            }
            break;
        case 1:
            ret = sprintf(buf, "%s\n", st->rfen ? "RF disabled" : "RF enabled");
            break;
        case 2:
            ret = sprintf(buf, "%s\n", st->picrst ? "PIC held in reset" : "PIC running");
            break;
        case 3:
            ret = sprintf(buf, "%s\n", st->rfshutdown ? "FMC power off" : "FMC power on");
            break;
        case 4:
            ret = sprintf(buf, "%s\n", st->pllfunction ? "1" : "0");
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_ddc_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
        case 0:
            switch (st->stage2mode) {
                case 0:
                    ret = sprintf(buf, "%s\n", "Bypass");
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
        case 1:
            ret = sprintf(buf, "%s %s\n", st->stage1mode & 0x01 ? "Bypass" : "",
                    st->stage1mode & 0x02 ? "12dB" : "0dB");
            break;
        case 2:
            ret = sprintf(buf, "%s\n", st->mixerpol ? "Positive" : "Negative");
            break;
        case 3:
            ret = sprintf(buf, "%s\n", st->mixerbyp ? "Bypass" : "Normal");
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_ddcfreq_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    ret = sprintf(buf, "%d\n", st->ddcfrequency);
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_if_out_freq_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    ret = sprintf(buf, "%d\n", st->iffrequency);
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_led_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    if (st->ledcolor < 0 || st->ledcolor > 7) {
        ret = sprintf(buf, "error\n");
    } else {
        ret = sprintf(buf, "%s\n", colors[st->ledcolor]);
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_fmc_status_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    unsigned int regval = 0;
    int ret = 0;

    mutex_lock(&indio_dev->mlock);
    regval = (u32) ioread32((st->pdata->base_addr) + chan->channel);

    switch ((u32) private) {
        case 0:
            st->PLLstatus = ((regval & FMC_PLL_STATUS) == FMC_PLL_STATUS) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->PLLstatus);
            break;
        case 1:
            st->OLlockDetect = ((regval & FMC_OL_LD) == FMC_OL_LD) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->OLlockDetect);
            break;
        case 2:
            st->RFalert = ((regval & FMC_RF_ALERT) == FMC_RF_ALERT) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->RFalert);
            break;
        case 3:
            st->TempAlert = ((regval & FMC_TEMP_ALERT) == FMC_TEMP_ALERT) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->TempAlert);
            break;
        case 4:
            st->PWRvalid = ((regval & FMC_PWR_VALID) == FMC_PWR_VALID) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->PWRvalid);
            break;
        case 5:
            st->PWRalert = ((regval & FMC_PWR_ALERT) == FMC_PWR_ALERT) ? 1 : 0;
            ret = sprintf(buf, "%d\n", st->PWRalert);
            break;
        case 6:
            ret = sprintf(buf, "%d\n", regval);
            break;
        default:
            ret = -EINVAL;
    }

    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_pwr_measure_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    unsigned int regval = 0;
    int ret = 0;
    mutex_lock(&indio_dev->mlock);
    regval = (u32) ioread32((st->pdata->base_addr) + chan->channel);
    st->pwrMeanCh0 = (regval & 0x000000FF);
    regval >>= 8;
    st->pwrMeanCh1 = (regval & 0x000000FF);
    regval >>= 8;
    st->pwrPeakCh0 = (regval & 0x000000FF);
    regval >>= 8;
    st->pwrPeakCh1 = (regval & 0x000000FF);
    switch ((u32) private) {
        case 0:
            ret = sprintf(buf, "%d\n", st->pwrMeanCh0);
            break;
        case 1:
            ret = sprintf(buf, "%d\n", st->pwrMeanCh1);
            break;
        case 2:
            ret = sprintf(buf, "%d\n", st->pwrPeakCh0);
            break;
        case 3:
            ret = sprintf(buf, "%d\n", st->pwrPeakCh1);
            break;
        default:
            ret = -EINVAL;
    }

    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_ifamp_measure_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
    struct rx1_state *st = iio_priv(indio_dev);
    unsigned int regval = 0;
    int ret = 0;

    mutex_lock(&indio_dev->mlock);
    regval = (u32) ioread32((st->pdata->base_addr) + chan->channel);
    st->ifPeakAmplitude = regval & 0x00FFFFFF;
    ret = sprintf(buf, "%d\n", st->ifPeakAmplitude);
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static int rx1_ifgain_write_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan,
        int val,
        int val2,
        long mask) {
    struct rx1_state *st = iio_priv(indio_dev);
    unsigned int code;
    int ret = 0;
    if (val < 0 || val > 36 || val2 < 0)
        return -EINVAL;
    code = gaincodes[(u8) val]; //integer part of gain
    mutex_lock(&indio_dev->mlock);
    switch (mask) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
            st-> ifgain = code;
            st->ifgaindb = val;
            iowrite32(st->ifgain, (st->pdata->base_addr) + chan->channel);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t rx1_if_out_freq_write(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        const char *buf, size_t len) {
    unsigned long long readin;
    struct rx1_state *st = iio_priv(indio_dev);
    int ret = 0;
    unsigned int regval;
    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);
    if (readin < 0 || readin > 60000000)
        ret = -EINVAL;
    else {
        regval = ((u32) readin / 1875);
        regval *= 2048;
        iowrite32(regval, (st->pdata->base_addr) + chan->channel);
        regval /= 2048;
        st->iffrequency = regval *1875;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret ? ret : len;
}

static int rx1_ifgain_read_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan,
        int *val,
        int *val2,
        long m) {
    struct rx1_state *st = iio_priv(indio_dev);
    int ret;
    mutex_lock(&indio_dev->mlock);
    switch (m) {
        case IIO_CHAN_INFO_HARDWAREGAIN:
            *val = st->ifgaindb;
            *val2 = 0;
            ret = IIO_VAL_INT_PLUS_MICRO_DB;
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);

    return ret;
};


#define _RX1_EXT_INFO(_name, _ident, _write, _read) { \
	.name = _name, \
	.read = _read, \
	.write = _write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info rx1_rfio_ext_info[] = {
    _RX1_EXT_INFO("branch", 0, rx1_rfio_write, rx1_rfio_read),
    _RX1_EXT_INFO("rfen", 1, rx1_rfio_write, rx1_rfio_read),
    _RX1_EXT_INFO("picrst", 2, rx1_rfio_write, rx1_rfio_read),
    _RX1_EXT_INFO("rfshutdown", 3, rx1_rfio_write, rx1_rfio_read),
    _RX1_EXT_INFO("pllfunction", 4, rx1_rfio_write, rx1_rfio_read), {},
};

static const struct iio_chan_spec_ext_info rx1_ddc_ext_info[] = {
    _RX1_EXT_INFO("stage2mode", 0, rx1_ddc_write, rx1_ddc_read), //0-3
    _RX1_EXT_INFO("stage1mode", 1, rx1_ddc_write, rx1_ddc_read), //4-5
    _RX1_EXT_INFO("mixerpol", 2, rx1_ddc_write, rx1_ddc_read), //6
    _RX1_EXT_INFO("mixerbyp", 3, rx1_ddc_write, rx1_ddc_read), //7
    {},
};

static const struct iio_chan_spec_ext_info rx1_ddcfreq_ext_info[] = {
    _RX1_EXT_INFO("ddcfrequency", 0, rx1_ddcfreq_write, rx1_ddcfreq_read), {}, //24
};

static const struct iio_chan_spec_ext_info rx1_led_ext_info[] = {
    _RX1_EXT_INFO("ledcolor", 0, rx1_led_write, rx1_led_read), {}, //0-3
};

static const struct iio_chan_spec_ext_info rx1_fmc_status_info[] = {
    _RX1_EXT_INFO("PLLstatus", 0, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("OLlockDetect", 1, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("RFalert", 2, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("TempAlert", 3, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("PWRvalid", 4, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("PWRalert", 5, NULL, rx1_fmc_status_read),
    _RX1_EXT_INFO("STATUS", 6, NULL, rx1_fmc_status_read), {},
};

static const struct iio_chan_spec_ext_info rx1_pwr_measure_ext_info[] = {
    _RX1_EXT_INFO("pwrMeanCh0", 0, NULL, rx1_pwr_measure_read),
    _RX1_EXT_INFO("pwrMeanCh1", 1, NULL, rx1_pwr_measure_read),
    _RX1_EXT_INFO("pwrPeakCh0", 2, NULL, rx1_pwr_measure_read),
    _RX1_EXT_INFO("pwrPeakCh1", 3, NULL, rx1_pwr_measure_read), {},
};

static const struct iio_chan_spec_ext_info rx1_if_freq_ext_info[] = {
    _RX1_EXT_INFO("if_output_frequency", 0, rx1_if_out_freq_write, rx1_if_out_freq_read), {}, //24 
};

static const struct iio_chan_spec_ext_info rx1_ifamp_measure_ext_info[] = {
    _RX1_EXT_INFO("ddc_st1_amp_peak", 0, NULL, rx1_ifamp_measure_read), {}, //24
};

static const struct iio_chan_spec rx1_chan[NUM_CHANNELS] = {
    {.type = IIO_ALTVOLTAGE,
        .channel = 0,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_rfio_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 1,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_ddc_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 2,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_ddcfreq_ext_info,},
    {.type = IIO_VOLTAGE,
        .channel = 3,
        .indexed = 1,
        .output = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),},
    {.type = IIO_ALTVOLTAGE,
        .channel = 4,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_led_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 5,
        .indexed = 1,
        .output = 0,
        .ext_info = rx1_fmc_status_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 6,
        .indexed = 1,
        .output = 0,
        .ext_info = rx1_pwr_measure_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 7,
        .indexed = 1,
        .output = 1,
        .ext_info = rx1_if_freq_ext_info,},
    {.type = IIO_ALTVOLTAGE,
        .channel = 8,
        .indexed = 1,
        .output = 0,
        .ext_info = rx1_ifamp_measure_ext_info,},
};

static const struct iio_info rx1_info = {
    .read_raw = &rx1_ifgain_read_raw,
    .write_raw = &rx1_ifgain_write_raw,
    .driver_module = THIS_MODULE,
};

static int rx1_probe(struct platform_device *pdev) {
    struct rx1_platform_data *pdata;
    struct iio_dev *indio_dev;
    struct rx1_state *st;
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
    indio_dev->name = "RX1_CONFIG";
    indio_dev->info = &rx1_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = rx1_chan;
    indio_dev->num_channels = NUM_CHANNELS;
    ret = iio_device_register(indio_dev);
    iowrite32(0x8F, pdata->base_addr);
    udelay(1000);
    iowrite32(0x0F, pdata->base_addr);
    st->ledcolor = 2;
    iowrite32(2, (pdata->base_addr) + 4); //LED starts RED
    return 0;
err_alloc_indio:
    iounmap(pdata->base_addr);
err_release_region:
    release_mem_region(pdata->res->start, pdata->remap_size);
    return ret;
}

static int rx1_remove(struct platform_device *pdev) {
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct rx1_state *st = iio_priv(indio_dev);
    iounmap(st->pdata->base_addr);
    release_mem_region(st->pdata->res->start, st->pdata->remap_size);
    iio_device_unregister(indio_dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rx1_dt_match[] = {
    { .compatible = "gr,rx1_config"},
    {},
};
MODULE_DEVICE_TABLE(of, rx1_dt_match);
#endif

static struct platform_driver rx1_driver = {
    .driver =
    {
        .name = "rx1_config",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(rx1_dt_match),
    },
    .probe = rx1_probe,
    .remove = rx1_remove,
};



module_platform_driver(rx1_driver);

MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_DESCRIPTION("RX1 CONFIG DRIVER ");
MODULE_LICENSE("GPL v2");

