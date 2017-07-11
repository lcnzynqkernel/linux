/*
 * AXI_ADC ADI PL Interface Module
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include "cf_axi_pl.h"
//#include "cf_axi_adc.h"

#define AXIADC_MAX_CHANNEL		16

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign, _shift, _ev, _nb_ev)	\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type =  BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = NULL,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	  .event_spec = _ev,						\
	  .num_event_specs = _nb_ev,					\
	}

enum {
	ID_PL2CH,
	ID_PL1CH,
};

struct axiadc_core_info {
	bool has_fifo_interface;
	unsigned int version;
};


static const int ad9643_scale_table[][2] = {
	{2087, 0x0F}, {2065, 0x0E}, {2042, 0x0D}, {2020, 0x0C}, {1997, 0x0B},
	{1975, 0x0A}, {1952, 0x09}, {1930, 0x08}, {1907, 0x07}, {1885, 0x06},
	{1862, 0x05}, {1840, 0x04}, {1817, 0x03}, {1795, 0x02}, {1772, 0x01},
	{1750, 0x00}, {1727, 0x1F}, {1704, 0x1E}, {1681, 0x1D}, {1658, 0x1C},
	{1635, 0x1B}, {1612, 0x1A}, {1589, 0x19}, {1567, 0x18}, {1544, 0x17},
	{1521, 0x16}, {1498, 0x15}, {1475, 0x14}, {1452, 0x13}, {1429, 0x12},
	{1406, 0x11}, {1383, 0x10},
};


static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_PL2CH] = {
		       .name = "PL2CH",
  			   .max_rate = 250000000UL,
			   .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
    		   .max_testmode = 0xF,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 'S',0,  NULL, 0),
		       .channel[1] = AIM_CHAN_NOCALIB(1, 1, 16, 'S',0,  NULL, 0),
		       },
	[ID_PL1CH] = {
		       .name = "PL1CH",
  			   .max_rate = 250000000UL,
			   .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
			   .max_testmode = 0xF,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 'S', 0, NULL, 0),
		       },

};

struct axipl_info {
	char						*name;
	unsigned					num_channels;
	unsigned 					num_shadow_slave_channels;
	const unsigned long 		*scan_masks;
	unsigned long				max_rate;
	struct iio_chan_spec		channel[AXIADC_MAX_CHANNEL];
};

struct axipl_state {
	struct iio_info			iio_info;
	void __iomem 			*regs;
	unsigned				max_usr_channel;
	unsigned				adc_def_output_mode;
	unsigned				max_count;
	unsigned				id;
	bool					has_fifo_interface;
	bool					dp_disable;
	unsigned				pcore_version;
	unsigned long 			adc_clk;
	bool					streaming_dma;
	unsigned				have_slave_channels;
	const struct axipl_info *pl_info;
	struct iio_hw_consumer	*frontend;

	struct iio_chan_spec	channels[AXIADC_MAX_CHANNEL];
};


static int axipl_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{

	return 0;
}

static int axipl_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{

	switch (m) {
	case IIO_CHAN_INFO_CALIBPHASE:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBBIAS:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT;
	default:
		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int axipl_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_CALIBPHASE:
		return 0;
	case IIO_CHAN_INFO_CALIBSCALE:
		return 0;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return 0;
	case IIO_CHAN_INFO_CALIBBIAS:
		return 0;
	default:
		return 0;
	}
}

static int axipl_read_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
		return -ENOSYS;
}

static int axipl_write_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
		return -ENOSYS;
}

static int axipl_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
		return -ENOSYS;
}

static int axipl_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
		return -ENOSYS;
}

static int axipl_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	return 0;
}


static int axipl_hw_submit_block(struct iio_dma_buffer_queue *queue,	
	struct iio_dma_buffer_block *block)
{
	//struct iio_dev *indio_dev = queue->driver_data;
	//struct axiadc_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(queue, block, DMA_FROM_DEVICE);

	//axiadc_write(st, ADI_REG_STATUS, ~0);
	//axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

	return 0;
}

static const struct iio_dma_buffer_ops axipl_dma_buffer_ops = {
	.submit = axipl_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,

};

int axipl_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
			&axipl_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

void axipl_unconfigure_ring_stream(struct iio_dev *indio_dev)
{
	iio_dmaengine_buffer_free(indio_dev->buffer);
}


static int axipl_channel_setup(struct iio_dev *indio_dev,
				const struct iio_chan_spec *adc_channels,
				unsigned adc_chan_num)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, cnt;

	//st->max_usr_channel = ADI_USR_CHANMAX(axiadc_read(st, ADI_REG_USR_CNTRL_1));
	st->max_usr_channel = 0; /* FIXME */

	for (i = 0, cnt = 0; i < adc_chan_num; i++)
		st->channels[cnt++] = adc_channels[i];

	for (i = 0; i < st->max_usr_channel; i++) {
		//usr_ctrl = axiadc_read(st, ADI_REG_CHAN_USR_CNTRL_1(cnt));
		st->channels[cnt].type = IIO_VOLTAGE;
		st->channels[cnt].channel = cnt;
		st->channels[cnt].scan_index = cnt;
		st->channels[cnt].info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		st->channels[cnt].extend_name = "user_logic";
		st->channels[cnt].scan_type.sign = 's';
		st->channels[cnt].scan_type.realbits = 0xFF;
		st->channels[cnt].scan_type.storagebits = 0xFF;
		st->channels[cnt].scan_type.shift = 0;
		st->channels[cnt].scan_type.endianness = IIO_BE;
		cnt++;
	}

	indio_dev->channels = st->channels;
	indio_dev->num_channels = cnt;
	indio_dev->masklength = cnt;

	return 0;
}

static const struct iio_info axipl_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &axipl_read_raw,
	.write_raw = &axipl_write_raw,
	.read_event_value = &axipl_read_event_value,
	.write_event_value = &axipl_write_event_value,
	.read_event_config = &axipl_read_event_config,
	.write_event_config = &axipl_write_event_config,
	.debugfs_reg_access = &axipl_reg_access,
	.update_scan_mode = &axipl_update_scan_mode,
};


static const struct axiadc_core_info pl_core_1_00_a_info = {
	.has_fifo_interface = true,
	.version = PCORE_VERSION(1, 0, 'a'),
};


/* Match table for of_platform binding */
static const struct of_device_id axipl_of_match[] = {
	{ .compatible = "xlnx,cf-pl-core-1.00.a", .data = &pl_core_1_00_a_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axipl_of_match);

/**
 * axiadc_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int axipl_probe(struct platform_device *pdev)
{
	const struct axiadc_core_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct axipl_state *st;
	struct resource *mem;
	int ret;
	const struct  axiadc_chip_info *chip_info;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
		 pdev->dev.of_node->name);

	id = of_match_node(axipl_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	info = id->data;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto err_put_converter;
	}

	st = iio_priv(indio_dev);

	/*mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);*/

	platform_set_drvdata(pdev, indio_dev);

	//st->dp_disable = axiadc_read(st, ADI_REG_ADC_DP_DISABLE);


	st->streaming_dma = of_property_read_bool(pdev->dev.of_node,
				"adi,streaming-dma");
	/* FIFO interface only supports streaming DMA */
	if (info)
		st->has_fifo_interface = info->has_fifo_interface;
	else
		st->has_fifo_interface = false;

	if (st->has_fifo_interface)
		st->streaming_dma = true;

	st->pcore_version = info->version;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = 0; // conv->chip_info->scan_masks;

	chip_info = &axiadc_chip_info_tbl[ID_PL2CH];

	axipl_channel_setup(indio_dev, chip_info->channel,
			      chip_info->num_channels);

	st->iio_info = axipl_info;
	//st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	st->dp_disable = false;

	if (!st->dp_disable) {

		//if (st->streaming_dma)
			ret = axipl_configure_ring_stream(indio_dev, NULL);
		//else
			//ret = axiadc_configure_ring(indio_dev, NULL);

		if (ret < 0)
			goto err_put_converter;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	if (indio_dev->buffer && indio_dev->buffer->scan_mask)
		*indio_dev->buffer->scan_mask =
			(1UL << chip_info->num_channels) - 1;

	dev_info(&pdev->dev, "ADI PL AIM (%d.%.2d.%c),"
		 " probed ADC %s as %s\n",
		PCORE_VERSION_MAJOR(st->pcore_version),
		PCORE_VERSION_MINOR(st->pcore_version),
		PCORE_VERSION_LETTER(st->pcore_version),
		"AXI PL",
		"MASTER");


	return 0;

err_unconfigure_ring:
	if (!st->dp_disable) {
		//if (st->streaming_dma)
			axipl_unconfigure_ring_stream(indio_dev);
		//else
			//axiadc_unconfigure_ring(indio_dev);
	}
err_put_converter:
	

	return ret;
}

/**
 * axipol_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int axipl_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!st->dp_disable) {
		//if (st->streaming_dma)
			axipl_unconfigure_ring_stream(indio_dev);
		//else
			//axiadc_unconfigure_ring(indio_dev);
	}

	return 0;
}

static struct platform_driver axipl_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axipl_of_match,
	},
	.probe		= axipl_probe,
	.remove		= axipl_remove,
};

module_platform_driver(axipl_driver);

MODULE_AUTHOR("Luis Cuellar <luiscn@gr.ssr.upm.es>");
MODULE_DESCRIPTION("Axi PL interface");
MODULE_LICENSE("GPL v2");
