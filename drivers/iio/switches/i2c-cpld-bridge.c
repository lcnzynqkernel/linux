/*
 * i2c-cpld-bridge
 *
 * Copyright 2016 GR UPM
 *
 * Licensed under the GPL-2.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/io.h>


#define NUM_CHANNELS 8

#define REG_FIELD_MASK(offset, size) (((1<<(size))-1) << (offset))
#define REG_FIELD_WRITE(value, offset, size) (((value) & ((1<<(size))-1)) << (offset))
#define REG_FIELD_READ(reg, offset, size) (((reg) >> (offset)) & ((1<<(size))-1))
#define REG_SIGN_EXTEND(value, bits) (((value) & (1<<bits) ? ~((1<<(bits))-1): 0 ) | (value))

/* definitions for register: RF control */
#define GPIO_RF_ADDRESS						  0
/* definitions for field: RF_SWOL in reg: RF control */
#define GPIO_RF_SWOL_SHIFT                    0
/* definitions for field: RF_SWBAND in reg: RF control */
#define GPIO_RF_SWBAND_SHIFT                  1
/* definitions for field: RF_SWSC_CTRL in reg: RF control */
#define GPIO_RF_SWSC_SHIFT                    3
/* definitions for field: RF_SWIN_CTRL in reg: RF control */
#define GPIO_RF_SWIN_SHIFT					  4
/* definitions for field: RF_SWC_CTRL in reg: RF control */
#define GPIO_RF_SWC_SHIFT					  5	
/* definitions for field: IQ_LO_BSEL in reg: RF control */
#define GPIO_RF_IQLO_BSEL_SHIFT               6

/* definitions for register: POWER control */
#define GPIO_PWR_ADDRESS					  1
/* definitions for field: RF_LNA2_EN in reg: POWER control */
#define GPIO_PWR_LNA2_EN_SHIFT                0
/* definitions for field: LF_OL_CE in reg: POWER control */
#define GPIO_PWR_LFOL_CE_SHIFT                1
/* definitions for field: SF_OL_CE in reg: POWER control */
#define GPIO_PWR_SCOL_CE_SHIFT                2
/* definitions for field: Int.REF enable  in reg: POWER control */
#define GPIO_PWR_INTREF_DIS_SHIFT             3
/* definitions for field: RFHF power enable in reg: POWER control */
#define GPIO_PWR_RFHF_PWREN_SHIFT             4
/* definitions for field: RFLF power enable in reg: POWER control */
#define GPIO_PWR_RFLF_PWREN_SHIFT             5
/* definitions for field: ADC_PDWN_1V8 in reg: POWER control */
#define GPIO_PWR_ADC_PDWN_SHIFT               6

/* definitions for register: SYNC control */
#define GPIO_SYNC_ADDRESS					  2
/* definitions for field: PLL_RESET in reg: SYNC control */
#define GPIO_SYNC_PLL_RESET_SHIFT             0
/* definitions for field: PLL_SYNC in reg: SYNC control */
#define GPIO_SYNC_PLL_SYNC_SHIFT              1
/* definitions for field: ADC_SYNC in reg: SYNC control */
#define GPIO_SYNC_ADC_SYNC_SHIFT              2

/* definitions for register: External IO */
#define GPIO_EXTIO_ADDRESS					  3
/* definitions for field: External IO in reg: External IO */
#define GPIO_EXTIO_EXTIO_SHIFT                0

/* definitions for register: Geographical Address REG */
#define GPIO_GA_ADDRESS					  	  4
/* definitions for field: GAddress in reg: Geographical Address REG */
#define GPIO_GA_GA_SHIFT                      0

/* definitions for register: LMK04281 PLL SEL */
#define GPIO_PLL_SEL_ADDRESS				  5
/* definitions for field: PLL SEL in reg: LMK04281 PLL SEL */
#define GPIO_PLL_SEL_PLLSEL_SHIFT             0

/* definitions for register: RF_PWR_Ch0_MSB */
#define GPIO_RF_PWR_Ch0_MSB_ADDRESS			  6
/* definitions for register: RF_PWR_Ch0_LSB */
#define GPIO_RF_PWR_Ch0_LSB_ADDRESS			  7
/* definitions for register: RF_PWR_Ch1_MSB */
#define GPIO_RF_PWR_Ch1_MSB_ADDRESS			  8
/* definitions for register: RF_PWR_Ch1_LSB */
#define GPIO_RF_PWR_Ch1_LSB_ADDRESS			  9

struct i2c_cpld_bridge_data  {
	struct i2c_client *client;
	unsigned long devid;
	struct mutex lock;
	//struct i2c_cpld_bridge_state* st;
	int int_time;
	unsigned int rf_swol;
	unsigned int rf_swband;
	unsigned int rf_swsc;
	unsigned int rf_swin;
	unsigned int rf_swc;
	unsigned int rf_iqlo_bsel;
	unsigned int pwr_lna2_en;
	unsigned int pwr_lfol_ce;
	unsigned int pwr_scol_ce;
	unsigned int pwr_intref_dis;
	unsigned int pwr_rfhf_pwren;
	unsigned int pwr_rflf_pwren;
	unsigned int pwr_adc_pdwn;
	unsigned int sync_pll_reset;
	unsigned int sync_pll_sync;
	unsigned int sync_adc_sync;
	unsigned int extio;
	unsigned int ga_address;
	unsigned int lmk04281_pll_sel;
	unsigned int pwrMeanCh0;
	unsigned int pwrMeanCh1;
	unsigned int pwrPeakCh0;
	unsigned int pwrPeakCh1;
};


static ssize_t rf_control_write(struct iio_dev *indio_dev,
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {

    unsigned long long readin;
    struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);

	switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else data->rf_swol = readin;
            break;
        case 1:
            if (readin < 1 || readin > 3)ret = -EINVAL;
            else data->rf_swband = readin;
            break;
        case 2:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->rf_swsc = readin;           
            break;
        case 3:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->rf_swin = readin;            
            break;
        case 4:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->rf_swc = readin;  
            break;
        case 5:
            if (readin < 1 || readin > 3)ret = -EINVAL;
            else data->rf_iqlo_bsel = readin;
            break;
        default:
            ret = -EINVAL;
    }

	if (!ret) {
		regval |= data->rf_swol & 0x1;
        regval |= REG_FIELD_WRITE(data->rf_swband,GPIO_RF_SWBAND_SHIFT,2);
        regval |= REG_FIELD_WRITE(data->rf_swsc,GPIO_RF_SWSC_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->rf_swin,GPIO_RF_SWIN_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->rf_swc,GPIO_RF_SWC_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->rf_iqlo_bsel,GPIO_RF_IQLO_BSEL_SHIFT,2);

		i2c_smbus_write_byte_data(data->client,GPIO_RF_ADDRESS,regval);


	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}


static ssize_t rf_control_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
	char data_read;
	i2c_smbus_write_byte(data->client,GPIO_RF_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			data->rf_swol = data_read & 0x1;
			ret = sprintf(buf, "%s\n", data->rf_swol ? "SC Low" : "SC High");
			break;
        case 1:
			data->rf_swband = REG_FIELD_READ(data_read, GPIO_RF_SWBAND_SHIFT , 2); 
            switch (data->rf_swband) {
                case 1:
                    ret = sprintf(buf, "%s\n", "Low band");
                    break;
                case 2:
                    ret = sprintf(buf, "%s\n", "50 Ohm Load");
                    break;
                case 3:
                    ret = sprintf(buf, "%s\n", "High band");
                    break;
            }
            break;
        case 2:
			data->rf_swsc = REG_FIELD_READ(data_read, GPIO_RF_SWSC_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->rf_swsc ? "SC Cal.out" :"SC RF chain");
            break;
        case 3:
			data->rf_swin = REG_FIELD_READ(data_read, GPIO_RF_SWIN_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->rf_swin ? "Cal.In" : "RF In" );
            break;
        case 4:
			data->rf_swc = REG_FIELD_READ(data_read, GPIO_RF_SWC_SHIFT , 1);
            ret = sprintf(buf, "%s\n", data->rf_swc ? "Low band" : "High band" );
            break;
        case 5:
			data->rf_iqlo_bsel = REG_FIELD_READ(data_read, GPIO_RF_IQLO_BSEL_SHIFT , 2);
            switch (data->rf_iqlo_bsel) {
                case 1:
                    ret = sprintf(buf, "%s\n", "DC-830 MHz");
                    break;
                case 2:
                    ret = sprintf(buf, "%s\n", "830-2.25GHz");
                    break;
                case 3:
                    ret = sprintf(buf, "%s\n", "2.25-4GHz");
                    break;
            }
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t pwr_control_write(struct iio_dev *indio_dev,
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {

 	unsigned long long readin;
    struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);

	switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else data->pwr_lna2_en = readin;
            break;
        case 1:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_lfol_ce = readin;
            break;
        case 2:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_scol_ce = readin;           
            break;
        case 3:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_intref_dis = readin;            
            break;
        case 4:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_rfhf_pwren = readin;  
            break;
        case 5:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_rflf_pwren = readin;
            break;
        case 6:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->pwr_adc_pdwn = readin;
            break;
        default:
            ret = -EINVAL;
    }

	if (!ret) {
		regval |= REG_FIELD_WRITE(data->pwr_lna2_en ,GPIO_PWR_LNA2_EN_SHIFT,1);
        regval |= REG_FIELD_WRITE(data->pwr_lfol_ce,GPIO_PWR_LFOL_CE_SHIFT,1);
        regval |= REG_FIELD_WRITE(data->pwr_scol_ce,GPIO_PWR_SCOL_CE_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->pwr_intref_dis,GPIO_PWR_INTREF_DIS_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->pwr_rfhf_pwren,GPIO_PWR_RFHF_PWREN_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->pwr_rflf_pwren,GPIO_PWR_RFLF_PWREN_SHIFT,1);
		regval |= REG_FIELD_WRITE(data->pwr_adc_pdwn,GPIO_PWR_ADC_PDWN_SHIFT,1);

		i2c_smbus_write_byte_data(data->client,GPIO_PWR_ADDRESS,regval);
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;

}

static ssize_t pwr_control_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
	char data_read;
	i2c_smbus_write_byte(data->client,GPIO_PWR_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			data->pwr_lna2_en = REG_FIELD_READ(data_read, GPIO_PWR_LNA2_EN_SHIFT , 1); 
			ret = sprintf(buf, "%s\n", data->pwr_lna2_en ? "Enabled" : "Bypass" );
			break;
        case 1:
			data->pwr_lfol_ce = REG_FIELD_READ(data_read, GPIO_PWR_LFOL_CE_SHIFT , 1); 
			ret = sprintf(buf, "%s\n", data->pwr_lfol_ce ? "Enabled" : "Disabled" );
            break;
        case 2:
			data->pwr_lfol_ce = REG_FIELD_READ(data_read, GPIO_PWR_SCOL_CE_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->pwr_scol_ce ? "Enabled" : "Disabled" );
            break;
        case 3:
			data->pwr_intref_dis = REG_FIELD_READ(data_read, GPIO_PWR_INTREF_DIS_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->pwr_intref_dis ? "Enabled" :"Disabled" );
            break;
        case 4:
			data->pwr_rfhf_pwren = REG_FIELD_READ(data_read, GPIO_PWR_RFHF_PWREN_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->pwr_rfhf_pwren ? "Enabled" :"Disabled" );
            break;
        case 5:
			data->pwr_rflf_pwren = REG_FIELD_READ(data_read, GPIO_PWR_RFLF_PWREN_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->pwr_rflf_pwren ? "Enabled" : "Disabled" );
            break;
        case 6:
			data->pwr_adc_pdwn = REG_FIELD_READ(data_read, GPIO_PWR_ADC_PDWN_SHIFT , 1); 
            ret = sprintf(buf, "%s\n", data->pwr_adc_pdwn ? "Power down" : "Power up");
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;

}

static ssize_t sync_control_write(struct iio_dev *indio_dev,
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {

	unsigned long long readin;
    struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    if (ret)
        return ret;
    mutex_lock(&indio_dev->mlock);

	switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 1) ret = -EINVAL;
            else data->sync_pll_reset = readin;
            break;
        case 1:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->sync_pll_sync = readin;
            break;
        case 2:
            if (readin < 0 || readin > 1)ret = -EINVAL;
            else data->sync_adc_sync = readin;           
            break;
        default:
            ret = -EINVAL;
    }

	if (!ret) {
		regval |= REG_FIELD_WRITE(data->sync_pll_reset,GPIO_SYNC_PLL_RESET_SHIFT,1);
        regval |= REG_FIELD_WRITE(data->sync_pll_sync,GPIO_SYNC_PLL_SYNC_SHIFT,1);
        regval |= REG_FIELD_WRITE(data->sync_adc_sync,GPIO_SYNC_ADC_SYNC_SHIFT,1);

		i2c_smbus_write_byte_data(data->client,GPIO_SYNC_ADDRESS,regval);
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;

}

static ssize_t sync_control_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
	char data_read;
	i2c_smbus_write_byte(data->client,GPIO_SYNC_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			data->sync_pll_reset = REG_FIELD_READ(data_read, GPIO_SYNC_PLL_RESET_SHIFT , 1); 
			ret = sprintf(buf, "%s\n", data->sync_pll_reset ? "Reset" : "PLL on" );
			break;
        case 1:
			data->sync_pll_reset = REG_FIELD_READ(data_read, GPIO_SYNC_PLL_SYNC_SHIFT , 1); 
			ret = sprintf(buf, "%s\n", data->sync_pll_sync ? "Sync" : "Unused" );
            break;
        case 2:
			data->sync_pll_reset = REG_FIELD_READ(data_read, GPIO_SYNC_ADC_SYNC_SHIFT , 1);
            ret = sprintf(buf, "%s\n", data->sync_adc_sync ? "Sync" : "Unused" );
            break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;

}

static ssize_t extio_control_write(struct iio_dev *indio_dev,
	uintptr_t private,
    const struct iio_chan_spec *chan,
    const char *buf, size_t len) {

	unsigned long long readin;
    struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
    int ret = 0;
    unsigned char regval = 0;

    ret = kstrtoull(buf, 10, &readin);
    mutex_lock(&indio_dev->mlock);

	switch ((u32) private) {
        case 0:
            if (readin < 0 || readin > 15) ret = -EINVAL;
            else data->extio= readin;
            break;
        default:
            ret = -EINVAL;
    }


	regval |= REG_FIELD_WRITE(data->extio,GPIO_EXTIO_EXTIO_SHIFT,4);
	i2c_smbus_write_byte_data(data->client,GPIO_EXTIO_ADDRESS,regval);

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;

}

static ssize_t extio_control_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);
	int ret = 0;
	char data_read;
	i2c_smbus_write_byte(data->client,GPIO_EXTIO_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
	data->extio = ret;
    switch ((u32) private) {
		case 0:
			data->extio = data_read &0xFF;
			ret = sprintf(buf, "%d\n", data->extio);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t ga_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);	
	int ret = 0;
	char data_read;
	i2c_smbus_write_byte(data->client,GPIO_GA_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			data->ga_address = data_read &0xF;
			ret = sprintf(buf, "%d\n", data->ga_address);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t pll_sel_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {

	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);	
	char data_read;
	int ret = 0;
	i2c_smbus_write_byte(data->client,GPIO_PLL_SEL_ADDRESS);
	data_read = i2c_smbus_read_byte(data->client);
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			data->lmk04281_pll_sel =  data_read &0x3;
			ret = sprintf(buf, "%d\n", data->lmk04281_pll_sel);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

static ssize_t pwr_measure_read(struct iio_dev *indio_dev,
        uintptr_t private,
        const struct iio_chan_spec *chan,
        char *buf) {
	int ret = 0;
	char data_read;
	struct i2c_cpld_bridge_data *data = iio_priv(indio_dev);	
    mutex_lock(&indio_dev->mlock);
    switch ((u32) private) {
		case 0:
			i2c_smbus_write_byte(data->client,GPIO_RF_PWR_Ch0_LSB_ADDRESS);
			data_read = i2c_smbus_read_byte(data->client);
			data->pwrMeanCh0 = (unsigned int)data_read;
			ret = sprintf(buf, "%d\n", data->pwrMeanCh0);
			break;
		case 1:
			i2c_smbus_write_byte(data->client,GPIO_RF_PWR_Ch1_LSB_ADDRESS);
			data_read = i2c_smbus_read_byte(data->client);
			data->pwrMeanCh1 = (unsigned int)data_read;
			ret = sprintf(buf, "%d\n", data->pwrMeanCh1);
			break;
		case 2:
			i2c_smbus_write_byte(data->client,GPIO_RF_PWR_Ch0_MSB_ADDRESS);
			data_read = i2c_smbus_read_byte(data->client);
			data->pwrPeakCh0 = (unsigned int)data_read;
			ret = sprintf(buf, "%d\n", data->pwrPeakCh0);
			break;
		case 3:
		    i2c_smbus_write_byte(data->client,GPIO_RF_PWR_Ch1_MSB_ADDRESS);
			data_read = i2c_smbus_read_byte(data->client);
			data->pwrPeakCh1 = (unsigned int)data_read;
			ret = sprintf(buf, "%d\n", data->pwrPeakCh1);
			break;
        default:
            ret = -EINVAL;
    }
    mutex_unlock(&indio_dev->mlock);
    return ret;
}

#define _I2C_CPLD_BRIDGE_EXT_INFO(_name, _ident, _write, _read) { \
	.name = _name, \
	.read = _read, \
	.write = _write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info rf_control_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("swol", 0, rf_control_write,  rf_control_read),  		//0
	_I2C_CPLD_BRIDGE_EXT_INFO("swband", 1, rf_control_write, rf_control_read), 		//1-2
    _I2C_CPLD_BRIDGE_EXT_INFO("swsc", 2, rf_control_write, rf_control_read), 		//3
    _I2C_CPLD_BRIDGE_EXT_INFO("swin", 3, rf_control_write, rf_control_read), 		//4
    _I2C_CPLD_BRIDGE_EXT_INFO("swc", 4, rf_control_write, rf_control_read), 		//5
	_I2C_CPLD_BRIDGE_EXT_INFO("iqlo_bsel", 5, rf_control_write, rf_control_read), 	//6-7
    {},
};

static const struct iio_chan_spec_ext_info pwr_control_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("lna2_en", 0, pwr_control_write,  pwr_control_read),  	//0
	_I2C_CPLD_BRIDGE_EXT_INFO("lfol_ce", 1, pwr_control_write, pwr_control_read), 		//1
    _I2C_CPLD_BRIDGE_EXT_INFO("scol_ce", 2, pwr_control_write, pwr_control_read), 		//2
    _I2C_CPLD_BRIDGE_EXT_INFO("intref_dis", 3, pwr_control_write, pwr_control_read), 	//3
    _I2C_CPLD_BRIDGE_EXT_INFO("rfhf_pwren", 4, pwr_control_write, pwr_control_read), 	//4
	_I2C_CPLD_BRIDGE_EXT_INFO("rflf_pwren", 5, pwr_control_write, pwr_control_read), 	//5
	_I2C_CPLD_BRIDGE_EXT_INFO("adc_pdwn", 6, pwr_control_write, pwr_control_read), 		//6
    {},
};

static const struct iio_chan_spec_ext_info sync_control_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("pll_reset", 0, sync_control_write,  sync_control_read),  	//0
	_I2C_CPLD_BRIDGE_EXT_INFO("pll_sync", 1, sync_control_write, sync_control_read), 		//1
    _I2C_CPLD_BRIDGE_EXT_INFO("adc_sync", 2, sync_control_write, sync_control_read), 		//2
    {},
};

static const struct iio_chan_spec_ext_info extio_control_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("ext_io", 0, extio_control_write,  extio_control_read),  	//0
    {},
};


static const struct iio_chan_spec_ext_info ga_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("ga", 0, NULL,  ga_read),  	//0
    {},
};

static const struct iio_chan_spec_ext_info LMK04281_PLL_SEL_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("lmk04281_pll_sel", 0, NULL,  pll_sel_read),  	//0
    {},
};

static const struct iio_chan_spec_ext_info pwr_measure_ext_info[] = {
    _I2C_CPLD_BRIDGE_EXT_INFO("pwr_mean_ch0", 0, NULL,  pwr_measure_read),  	//0
	_I2C_CPLD_BRIDGE_EXT_INFO("pwr_mean_ch1", 1, NULL,  pwr_measure_read),  	//1
    _I2C_CPLD_BRIDGE_EXT_INFO("pwr_peak_ch0", 2, NULL,  pwr_measure_read),  	//2
	_I2C_CPLD_BRIDGE_EXT_INFO("pwr_peak_ch1", 3, NULL,  pwr_measure_read),  	//3
    {},
};

static const struct iio_chan_spec i2c_cpld_bridge_chan[NUM_CHANNELS] = {
    {.type = IIO_ALTVOLTAGE,
        .channel = 0,
        .indexed = 1,
        .output = 1,
        .ext_info = rf_control_ext_info,},
 	{.type = IIO_ALTVOLTAGE,
        .channel = 1,
        .indexed = 1,
        .output = 1,
        .ext_info = pwr_control_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 2,
        .indexed = 1,
        .output = 1,
        .ext_info = sync_control_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 3,
        .indexed = 1,
        .output = 1,
        .ext_info = extio_control_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 4,
        .indexed = 1,
        .output = 1,
        .ext_info = ga_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 5,
        .indexed = 1,
        .output = 1,
        .ext_info = LMK04281_PLL_SEL_ext_info,},
	{.type = IIO_ALTVOLTAGE,
        .channel = 6,
        .indexed = 1,
        .output = 1,
        .ext_info = pwr_measure_ext_info,},
};

static const struct iio_info i2c_cpld_bridge_info = {
	//.read_raw = mlx90614_read_raw,
	.driver_module = THIS_MODULE,
};

static int i2c_cpld_bridge_probe(struct i2c_client *client,
								 const struct i2c_device_id *id) {

	struct iio_dev *indio_dev;
	struct i2c_cpld_bridge_data *data;
	unsigned char regval = 0;
	//struct i2c_cpld_bridge_state *pstate; 

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE |
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->devid = id->driver_data;

	/*Init Defaults*/
	data->rf_swol = 0;
	data->rf_swband = 2;
	data->rf_swsc = 0;
	data->rf_swin = 0;
	data->rf_swc = 1;
	data->rf_iqlo_bsel = 1;

	data->pwr_lna2_en = 0;
	data->pwr_lfol_ce = 1;
	data->pwr_scol_ce = 1;
	data->pwr_intref_dis = 0;
	data->pwr_rfhf_pwren = 0;
	data->pwr_rflf_pwren = 0;
	data->pwr_adc_pdwn = 0;

	data->sync_pll_reset = 0;
	data->sync_pll_sync = 0;
	data->sync_adc_sync = 0;

	data->extio = 3;

	mutex_init(&data->lock);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = client->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &i2c_cpld_bridge_info;

	indio_dev->channels = i2c_cpld_bridge_chan;
	indio_dev->num_channels = ARRAY_SIZE(i2c_cpld_bridge_chan);

	/*Configure Defaults */
	regval |= REG_FIELD_WRITE(data->rf_swol,0,1);
    regval |= REG_FIELD_WRITE(data->rf_swband,GPIO_RF_SWBAND_SHIFT,2);
    regval |= REG_FIELD_WRITE(data->rf_swsc,GPIO_RF_SWSC_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->rf_swin,GPIO_RF_SWIN_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->rf_swc,GPIO_RF_SWC_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->rf_iqlo_bsel,GPIO_RF_IQLO_BSEL_SHIFT,2);
	i2c_smbus_write_byte_data(data->client,GPIO_RF_ADDRESS,regval);

	regval = 0;
	regval |= REG_FIELD_WRITE(data->pwr_lna2_en ,GPIO_PWR_LNA2_EN_SHIFT,1);
    regval |= REG_FIELD_WRITE(data->pwr_lfol_ce,GPIO_PWR_LFOL_CE_SHIFT,1);
    regval |= REG_FIELD_WRITE(data->pwr_scol_ce,GPIO_PWR_SCOL_CE_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->pwr_intref_dis,GPIO_PWR_INTREF_DIS_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->pwr_rfhf_pwren,GPIO_PWR_RFHF_PWREN_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->pwr_rflf_pwren,GPIO_PWR_RFLF_PWREN_SHIFT,1);
	regval |= REG_FIELD_WRITE(data->pwr_adc_pdwn,GPIO_PWR_ADC_PDWN_SHIFT,1);
	i2c_smbus_write_byte_data(data->client,GPIO_PWR_ADDRESS,regval);

	regval = 0;
	regval |= REG_FIELD_WRITE(data->sync_pll_reset,GPIO_SYNC_PLL_RESET_SHIFT,1);
    regval |= REG_FIELD_WRITE(data->sync_pll_sync,GPIO_SYNC_PLL_SYNC_SHIFT,1);
    regval |= REG_FIELD_WRITE(data->sync_adc_sync,GPIO_SYNC_ADC_SYNC_SHIFT,1);
	i2c_smbus_write_byte_data(data->client,GPIO_SYNC_ADDRESS,regval);

	i2c_smbus_write_byte_data(data->client,GPIO_EXTIO_ADDRESS,data->extio);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static int i2c_cpld_bridge_remove(struct i2c_client *client) {
  	
	iio_device_unregister(i2c_get_clientdata(client));

  	return 0;
}


#ifdef CONFIG_OF
static const struct i2c_device_id i2c_cpld_bridge_id[] = {
	{ "i2c_cpld_bridge", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_cpld_bridge_id);
#endif

static struct i2c_driver i2c_cpld_bridge_driver = {
    .driver =
    {
        .name = "i2c_cpld_bridge",
        .owner = THIS_MODULE,
    },
    .probe = i2c_cpld_bridge_probe,
    .remove = i2c_cpld_bridge_remove,
	.id_table = i2c_cpld_bridge_id,
};

module_i2c_driver(i2c_cpld_bridge_driver);

MODULE_AUTHOR("Luis Cuéllar  <luiscn@gr.ssr.upm.es>");
MODULE_DESCRIPTION("I2C TO CPLD GPIO BRIDGE ");
MODULE_LICENSE("GPL v2");
