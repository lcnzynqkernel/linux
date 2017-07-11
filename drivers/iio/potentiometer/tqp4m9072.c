/*
 * tqp4m9072 SPI Digital Attenuator
 *
 * Copyright 2015 GR UPM
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitrev.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

struct tqp4m9072_state {
	struct spi_device	*spi;
	struct regulator	*reg;
	unsigned char		attenuation;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data ____cacheline_aligned;
};

static int tqp4m9072_write(struct iio_dev *indio_dev,
			unsigned char atten)
{
	struct tqp4m9072_state *st = iio_priv(indio_dev);
	int ret;

	st->attenuation = atten;
	ret = spi_write(st->spi, &st->attenuation, 1);
	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int tqp4m9072_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct tqp4m9072_state *st = iio_priv(indio_dev);
	int ret;
	unsigned code;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		code = st->attenuation;

		/* Values in dB */
		code = 31500 - (code * 500);
		*val = code / 1000;
		*val2 = (code % 1000) * 1000;

		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int tqp4m9072_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct tqp4m9072_state *st = iio_priv(indio_dev);
	unsigned code;
	int ret;

	if (val < 0 || val2 < 0)
		return -EINVAL;

	/* Values in dB */
	code = (((u8)val * 1000) + ((u32)val2 / 1000));

	if (code > 31500 || code < 0)
		return -EINVAL;

	code = (31500 - code) / 500;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		st->attenuation = code;
		ret = tqp4m9072_write(indio_dev, st->attenuation);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info tqp4m9072_info = {
	.read_raw = &tqp4m9072_read_raw,
	.write_raw = &tqp4m9072_write_raw,
	.driver_module = THIS_MODULE,
};

#define tqp4m9072_CHAN(_channel) {				\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = _channel,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),\
}

static const struct iio_chan_spec tqp4m9072_channels[] = {
	tqp4m9072_CHAN(0),
};

static int tqp4m9072_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct tqp4m9072_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;

	/* try to get a unique name */
	if (spi->dev.platform_data)
		indio_dev->name = spi->dev.platform_data;
	else if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

		indio_dev->channels = tqp4m9072_channels;
		indio_dev->num_channels = ARRAY_SIZE(tqp4m9072_channels);

	indio_dev->info = &tqp4m9072_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = tqp4m9072_write(indio_dev, 0);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}


static int tqp4m9072_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct tqp4m9072_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg))
		regulator_disable(reg);

	return 0;
}

static const struct spi_device_id tqp4m9072_id[] = {
	{"tqp4m9072", 0},
	{}
};

static struct spi_driver tqp4m9072_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= tqp4m9072_probe,
	.remove		= tqp4m9072_remove,
	.id_table	= tqp4m9072_id,
};

module_spi_driver(tqp4m9072_driver);

MODULE_AUTHOR("David Marcos <dmarcosgon@gr.ssr.upm.es>");
MODULE_DESCRIPTION("Triquint tqp4m9072 DSA");
MODULE_LICENSE("GPL v2");

