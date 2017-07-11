/*
 * AXI PL
 */

#ifndef ADI_AXI_PL_H_
#define ADI_AXI_PL_H_

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */

#define PCORE_VERSION(major, minor, letter) ((major << 16) | (minor << 8) | letter)
#define PCORE_VERSION_MAJOR(version) (version >> 16)
#define PCORE_VERSION_MINOR(version) ((version >> 8) & 0xff)
#define PCORE_VERSION_LETTER(version) (version & 0xff)

/* ADC COMMON */

#define ADI_REG_RSTN			0x0040
#define ADI_RSTN				(1 << 0)
#define ADI_MMCM_RSTN 			(1 << 1)

#define ADI_REG_CNTRL			0x0044
#define ADI_R1_MODE			(1 << 2)
#define ADI_DDR_EDGESEL			(1 << 1)
#define ADI_PIN_MODE			(1 << 0)

#define ADI_REG_CLK_FREQ			0x0054
#define ADI_CLK_FREQ(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_FREQ(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_CLK_RATIO		0x0058
#define ADI_CLK_RATIO(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_CLK_RATIO(x)		(((x) >> 0) & 0xFFFFFFFF)


#define ADI_STATUS			(1 << 0)

#define ADI_REG_DRP_CNTRL		0x0070
#define ADI_DRP_SEL			(1 << 29)
#define ADI_DRP_RWN			(1 << 28)
#define ADI_DRP_ADDRESS(x)		(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)		(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS		0x0074
#define ADI_DRP_STATUS			(1 << 16)
#define ADI_DRP_RDATA(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_RDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DMA_CNTRL		0x0080
#define ADI_DMA_STREAM			(1 << 1)
#define ADI_DMA_START			(1 << 0)

#define ADI_REG_DMA_COUNT		0x0084
#define ADI_DMA_COUNT(x)			(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_COUNT(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_DMA_STATUS		0x0088
#define ADI_DMA_OVF			(1 << 2)
#define ADI_DMA_UNF			(1 << 1)
#define ADI_DMA_STATUS			(1 << 0)

#define ADI_REG_DMA_BUSWIDTH		0x008C
#define ADI_DMA_BUSWIDTH(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_DMA_BUSWIDTH(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_USR_CNTRL_1		0x00A0
#define ADI_USR_CHANMAX(x)		(((x) & 0xFF) << 0)
#define ADI_TO_USR_CHANMAX(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_ADC_DP_DISABLE 			0x00C0

/* PCORE Version > 8.00 */
#define ADI_REG_DELAY(l)				(0x0800 + (l) * 0x4)

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1


/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC	0x80000000

#define AXIADC_MAX_CHANNEL		16

#include <linux/spi/spi.h>

enum {
	ID_AD9467,
	ID_AD9649,
};

struct axiadc_chip_info {
	char				*name;
	unsigned			num_channels;
	unsigned 			num_shadow_slave_channels;
	const unsigned long 	*scan_masks;
	const int			(*scale_table)[2];
	int					num_scales;
	int					max_testmode;
	unsigned long			max_rate;
	struct iio_chan_spec		channel[AXIADC_MAX_CHANNEL];
};

struct axiadc_state {
	struct device 			*dev_spi;
	struct iio_info			iio_info;
	void __iomem			*regs;
	void __iomem			*slave_regs;
	unsigned				max_usr_channel;
	unsigned			adc_def_output_mode;
	unsigned			max_count;
	unsigned			id;
	unsigned			pcore_version;
	bool				has_fifo_interface;
	bool				dp_disable;
	unsigned long 		adc_clk;
	bool				streaming_dma;
	unsigned			have_slave_channels;

	struct iio_hw_consumer	*frontend;

	struct iio_chan_spec	channels[AXIADC_MAX_CHANNEL];
};

struct axiadc_converter {
	struct spi_device 	*spi;
	struct clk 		*clk;
	void 			*phy;
	struct gpio_desc		*pwrdown_gpio;
	struct gpio_desc		*reset_gpio;
	unsigned			id;
	unsigned			adc_output_mode;
	unsigned			scratch_reg[AXIADC_MAX_CHANNEL];
	unsigned long 		adc_clk;
	const struct axiadc_chip_info	*chip_info;


	struct iio_chan_spec const	*channels;
	int				num_channels;
	const struct attribute_group	*attrs;
	struct iio_dev 	*indio_dev;

	int (*post_setup)(struct iio_dev *indio_dev);

};




/*
 * IO accessors
 */

int axipl_configure_ring(struct iio_dev *indio_dev, const char *dma_name);
int axipl_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name);
void axipl_unconfigure_ring_stream(struct iio_dev *indio_dev);

#endif /* ADI_AXI_ADC_H_ */
