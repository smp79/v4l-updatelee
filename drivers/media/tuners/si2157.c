/*
 * Silicon Labs Si2146/2147/2148/2157/2158 silicon tuner driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include "si2157_priv.h"

static const struct dvb_tuner_ops si2157_ops;

static DEFINE_MUTEX(si2157_list_mutex);
static LIST_HEAD(hybrid_tuner_instance_list);

/* execute firmware command */
static int si2157_cmd_execute(struct i2c_client *client, struct si2157_cmd *cmd)
{
	struct si2157_dev *dev = i2c_get_clientdata(client);
	int ret;
	unsigned long timeout;

//	dprintk("W: %*ph", cmd->wlen, cmd->args);

	mutex_lock(&dev->i2c_mutex);

	if (cmd->wlen) {
		memcpy(cmd->w_args, cmd->args, cmd->wlen);
		/* write cmd and args for firmware */
		ret = i2c_master_send(client, cmd->args, cmd->wlen);
		if (ret < 0) {
			goto w_err_mutex_unlock;
		} else if (ret != cmd->wlen) {
			ret = -EREMOTEIO;
			goto w_err_mutex_unlock;
		}
	}

	if (cmd->rlen) {
		/* wait cmd execution terminate */
		#define TIMEOUT 500
		timeout = jiffies + msecs_to_jiffies(TIMEOUT);
		while (!time_after(jiffies, timeout)) {
			ret = i2c_master_recv(client, cmd->args, cmd->rlen);
			if (ret < 0) {
				goto r_err_mutex_unlock;
			} else if (ret != cmd->rlen) {
				ret = -EREMOTEIO;
				goto r_err_mutex_unlock;
			}

			/* firmware ready? */
			if ((cmd->args[0] >> 7) & 0x01)
				break;
		}

		if (!((cmd->args[0] >> 7) & 0x01)) {
			ret = -ETIMEDOUT;
			goto r_err_mutex_unlock;
		}
	}

	mutex_unlock(&dev->i2c_mutex);

//	dprintk("R: %*ph", cmd->rlen, cmd->args);

	return 0;

w_err_mutex_unlock:
	mutex_unlock(&dev->i2c_mutex);
	dprintk("Write: failed=%d", ret);
	dprintk("W: %*ph", cmd->wlen, cmd->args);
	return ret;
r_err_mutex_unlock:
	mutex_unlock(&dev->i2c_mutex);
	dprintk("Read: failed=%d", ret);
	dprintk("W: %*ph", cmd->wlen, cmd->w_args);
	dprintk("R: %*ph", cmd->rlen, cmd->args);
	return ret;
}

static int si2157_init(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, len, remaining;
	struct si2157_cmd cmd;
	const struct firmware *fw;
	const char *fw_name;
	unsigned int uitmp, chip_id;

	/* Returned IF frequency is garbage when firmware is not running */
	memcpy(cmd.args, "\x15\x00\x06\x07", 4);
	cmd.wlen = 4;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	uitmp = cmd.args[2] << 0 | cmd.args[3] << 8;

	if (uitmp == dev->if_frequency / 1000)
		goto warm;

	/* power up */
	if (dev->chiptype == SI2157_CHIPTYPE_SI2146) {
		memcpy(cmd.args, "\xc0\x05\x01\x00\x00\x0b\x00\x00\x01", 9);
		cmd.wlen = 9;
	} else if (dev->chiptype == SI2157_CHIPTYPE_SI2141) {
		// This might be wrong
		memcpy(cmd.args, "\xc0\x00\x0d\x0e\x00\x01\x01\x01\x01\x03", 10);
		cmd.wlen = 10;
	} else {
		memcpy(cmd.args, "\xc0\x00\x0c\x00\x00\x01\x01\x01\x01\x01\x01\x02\x00\x00\x01", 15);
		cmd.wlen = 15;
	}
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* Si2141 needs a second command before it answers the revision query */
	if (dev->chiptype == SI2157_CHIPTYPE_SI2141) {
		memcpy(cmd.args, "\xc0\x08\x01\x02\x00\x00\x01", 7);
		cmd.wlen = 7;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}

	/* query chip revision */
	memcpy(cmd.args, "\x02", 1);
	cmd.wlen = 1;
	cmd.rlen = 13;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	chip_id = cmd.args[1] << 24 | cmd.args[2] << 16 | cmd.args[3] << 8 |
			cmd.args[4] << 0;

	#define SI2158_A20 ('A' << 24 | 58 << 16 | '2' << 8 | '0' << 0)
	#define SI2148_A20 ('A' << 24 | 48 << 16 | '2' << 8 | '0' << 0)
	#define SI2157_A30 ('A' << 24 | 57 << 16 | '3' << 8 | '0' << 0)
	#define SI2147_A30 ('A' << 24 | 47 << 16 | '3' << 8 | '0' << 0)
	#define SI2146_A10 ('A' << 24 | 46 << 16 | '1' << 8 | '0' << 0)
	#define SI2141_A10 ('A' << 24 | 41 << 16 | '1' << 8 | '0' << 0)

	switch (chip_id) {
	case SI2158_A20:
	case SI2148_A20:
		fw_name = SI2158_A20_FIRMWARE;
		break;
	case SI2141_A10:
		fw_name = SI2141_A10_FIRMWARE;
		break;
	case SI2157_A30:
	case SI2147_A30:
	case SI2146_A10:
		fw_name = NULL;
		break;
	default:
		dprintk("unknown chip version Si21%d-%c%c%c \n\n",
				cmd.args[2], cmd.args[1],
				cmd.args[3], cmd.args[4]);
		ret = -EINVAL;
		goto err;
	}

	dprintk("found a 'Silicon Labs Si21%d-%c%c%c'", cmd.args[2], cmd.args[1], cmd.args[3], cmd.args[4]);

	if (fw_name == NULL)
		goto skip_fw_download;

	/* request the firmware, this will block and timeout */
	ret = request_firmware(&fw, fw_name, &client->dev);
	if (ret) {
		dprintk("firmware file '%s' not found\n",
				fw_name);
		goto err;
	}

	/* firmware should be n chunks of 17 bytes */
	if (fw->size % 17 != 0) {
		dprintk("firmware file '%s' is invalid\n",
				fw_name);
		ret = -EINVAL;
		goto err_release_firmware;
	}

	dprintk("downloading firmware from file '%s'", fw_name);

	for (remaining = fw->size; remaining > 0; remaining -= 17) {
		len = fw->data[fw->size - remaining];
		if (len > SI2157_ARGLEN) {
			dprintk("Bad firmware length\n");
			ret = -EINVAL;
			goto err_release_firmware;
		}
		memcpy(cmd.args, &fw->data[(fw->size - remaining) + 1], len);
		cmd.wlen = len;
		cmd.rlen = 1;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret) {
			dprintk("firmware download failed %d\n",
					ret);
			goto err_release_firmware;
		}
	}

	release_firmware(fw);

skip_fw_download:
	/* reboot the tuner with new firmware? */
	memcpy(cmd.args, "\x01\x01", 2);
	cmd.wlen = 2;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* query firmware version */
	memcpy(cmd.args, "\x11", 1);
	cmd.wlen = 1;
	cmd.rlen = 10;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	dprintk("firmware version: %c.%c.%d\n",
			cmd.args[6], cmd.args[7], cmd.args[8]);

	if (dev->chiptype == SI2157_CHIPTYPE_SI2141) {
		/* set clock */
		memcpy(cmd.args, "\xc0\x00\x0d", 3);
		cmd.wlen = 3;
		cmd.rlen = 1;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;
		/* setup PIN */
		memcpy(cmd.args, "\x12\x80\x80\x85\x00\x81\x00", 7);
		cmd.wlen = 7;
		cmd.rlen = 7;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;
	}

	/* enable tuner status flags, for si2157_tune_wait() */
	memcpy(cmd.args, "\x14\x00\x01\x05\x01\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x01\x06\x01\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	memcpy(cmd.args, "\x14\x00\x01\x07\x01\x00", 6);
	cmd.wlen = 6;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;
warm:
	/* init statistics in order signal app which are supported */
	c->strength.len = 1;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	dev->active = true;
	return 0;
err_release_firmware:
	release_firmware(fw);
err:
	dprintk("failed=%d", ret);
	return ret;
}

static int si2157_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
//	int ret;
//	struct si2157_cmd cmd;

//	dprintk("");

	dev->active = false;

//	/* standby */
//	memcpy(cmd.args, "\x16\x00", 2);
//	cmd.wlen = 2;
//	cmd.rlen = 1;
//	ret = si2157_cmd_execute(client, &cmd);
//	if (ret)
//		goto err;

	return 0;
//err:
//	dprintk("failed=%d", ret);
//	return ret;
}

static int si2157_set_params(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	struct si2157_cmd cmd;
	u8 bandwidth, delivery_system;
	u32 if_frequency = 5000000;

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	if (c->bandwidth_hz <= 6000000)
		bandwidth = 0x06;
	else if (c->bandwidth_hz <= 7000000)
		bandwidth = 0x07;
	else if (c->bandwidth_hz <= 8000000)
		bandwidth = 0x08;
	else
		bandwidth = 0x0f;

	switch (c->delivery_system) {
	case SYS_ATSC:
			delivery_system = 0x00;
			if_frequency = 3250000;
			break;
	case SYS_DVBC_ANNEX_B:
			delivery_system = 0x10;
			if_frequency = 4000000;
			break;
	case SYS_DVBT:
	case SYS_DVBT2: /* it seems DVB-T and DVB-T2 both are 0x20 here */
			delivery_system = 0x20;
			break;
	case SYS_DVBC_ANNEX_A:
			delivery_system = 0x30;
			break;
	case SYS_ISDBT:
			delivery_system = 0x40;
			break;
	case SYS_DTMB:
			delivery_system = 0x60;
			break;
	default:
			ret = -EINVAL;
			goto err;
	}

	memcpy(cmd.args, "\x14\x00\x03\x07\x00\x00", 6);
	cmd.args[4] = delivery_system | bandwidth;
	if (dev->inversion)
		cmd.args[5] = 0x01;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	if (dev->chiptype == SI2157_CHIPTYPE_SI2146)
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x01", 6);
	else
		memcpy(cmd.args, "\x14\x00\x02\x07\x00\x00", 6);
	cmd.args[4] = dev->if_port;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set digital if frequency if needed */
	if (if_frequency != dev->if_frequency) {
		memcpy(cmd.args, "\x14\x00\x06\x07", 4);
		cmd.args[4] = (if_frequency / 1000) & 0xff;
		cmd.args[5] = ((if_frequency / 1000) >> 8) & 0xff;
		cmd.wlen = 6;
		cmd.rlen = 4;
		ret = si2157_cmd_execute(client, &cmd);
		if (ret)
			goto err;

		dev->if_frequency = if_frequency;
	}

	/* set digital frequency */
	memcpy(cmd.args, "\x41\x00\x00\x00\x00\x00\x00\x00", 8);
	cmd.args[4] = (c->frequency >>  0) & 0xff;
	cmd.args[5] = (c->frequency >>  8) & 0xff;
	cmd.args[6] = (c->frequency >> 16) & 0xff;
	cmd.args[7] = (c->frequency >> 24) & 0xff;
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	dev->bandwidth = bandwidth;
	dev->frequency = c->frequency;

	return 0;
err:
	dev->bandwidth = 0;
	dev->frequency = 0;
	dev->if_frequency = 0;
	dprintk("failed=%d", ret);
	return ret;
}

static int si2157_set_analog_params(struct dvb_frontend *fe,
				      struct analog_parameters *params)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);
	char *std; /* for debugging */
	int ret;
	struct si2157_cmd cmd;
	u32 bandwidth = 0;
	u32 if_frequency = 0;
	u32 freq = 0;
	u64 tmp_lval = 0;
	u8 system = 0;
	u8 color = 0;    /* 0=NTSC/PAL, 0x10=SECAM */
	u8 invert_analog = 1; /* analog tuner spectrum; 0=normal, 1=inverted */

	if (dev->chiptype != SI2157_CHIPTYPE_SI2157) {
		dev_info(&client->dev, "%s: Analog tuning not supported for chiptype=%u\n",
				__func__, dev->chiptype);
		ret = -EINVAL;
		goto err;
	}

	if (!dev->active)
		si2157_init(fe);

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}
	if (params->mode == V4L2_TUNER_RADIO) {
	/*
	 * std = "fm";
	 * bandwidth = 1700000; //best can do for FM, AGC will be a mess though
	 * if_frequency = 1250000;  //HVR-225x(saa7164), HVR-12xx(cx23885)
	 * if_frequency = 6600000;  //HVR-9xx(cx231xx)
	 * if_frequency = 5500000;  //HVR-19xx(pvrusb2)
	 */
		dev_dbg(&client->dev, "si2157 does not currently support FM radio\n");
		ret = -EINVAL;
		goto err;
	}
	tmp_lval = params->frequency * 625LL;
	do_div(tmp_lval, 10); /* convert to HZ */
	freq = (u32)tmp_lval;

	if (freq < 1000000) /* is freq in KHz */
		freq = freq * 1000;
	dev->frequency = freq;

	if (params->std & (V4L2_STD_B|V4L2_STD_GH)) {
		if (freq >= 470000000) {
			std = "palGH";
			bandwidth = 8000000;
			if_frequency = 6000000; /* matches tda18271C2, works w/cx23885 */
			system = 1;
			if (params->std & (V4L2_STD_SECAM_G|V4L2_STD_SECAM_H)) {
				std = "secamGH";
				color = 0x10;
			}
		} else {
			std = "palB";
			bandwidth = 7000000;
			if_frequency = 6000000; /* matches tda18271C2, works w/cx23885 */;
			system = 0;
			if (params->std & V4L2_STD_SECAM_B) {
				std = "secamB";
				color = 0x10;
			}
		}
	} else if (params->std & V4L2_STD_MN) {
		std = "MN";
		bandwidth = 6000000;
		if_frequency = 5400000; /* matches tda18271C2, works w/cx23885 */
		system = 2;
	} else if (params->std & V4L2_STD_PAL_I) {
		std = "palI";
		bandwidth = 8000000;
		if_frequency = 7250000; /* matches tda18271C2, does not work yet w/cx23885 */
		system = 4;
	} else if (params->std & V4L2_STD_DK) {
		std = "palDK";
		bandwidth = 8000000;
		if_frequency = 6900000; /* matches tda18271C2, does not work yet w/cx23885 */
		system = 5;
		if (params->std & V4L2_STD_SECAM_DK) {
			std = "secamDK";
			color = 0x10;
		}
	} else if (params->std & V4L2_STD_SECAM_L) {
		std = "secamL";
		bandwidth = 8000000;
		if_frequency = 6750000; /* not tested yet w/cx23885 */
		system = 6;  color = 0x10;
	} else if (params->std & V4L2_STD_SECAM_LC) {
		std = "secamL'";
		bandwidth = 7000000;
		if_frequency = 1250000; /* not tested yet w/cx23885 */
		system = 7;  color = 0x10;
	} else {
		std = "unknown";
	}
	/* calc channel center freq */
	freq = freq - 1250000 + (bandwidth/2);

	dev_dbg(&client->dev,
			"mode=%d system=%u std='%s' params->frequency=%u center freq=%u if=%u bandwidth=%u\n",
			params->mode, system, std, params->frequency,
			freq, if_frequency, bandwidth);

	/* set analog IF port */
	memcpy(cmd.args, "\x14\x00\x03\x06\x08\x02", 6);
	/* in using dev->if_port, we assume analog and digital IF's */
	/*  are always on different ports */
	/* assumes if_port definition is 0 or 1 for digital out */
	cmd.args[4] = (dev->if_port == 1)?8:10;
	cmd.args[5] = (dev->if_port == 1)?2:1; /* Analog AGC assumed external */
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set analog IF output config */
	memcpy(cmd.args, "\x14\x00\x0d\x06\x94\x64", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* make this distinct from a digital IF */
	dev->if_frequency = if_frequency | 1;

	/* calc and set tuner analog if center frequency */
	if_frequency = if_frequency + 1250000 - (bandwidth/2);
	dev_dbg(&client->dev, "IF Ctr freq=%d\n", if_frequency);

	memcpy(cmd.args, "\x14\x00\x0C\x06", 4);
	cmd.args[4] = (if_frequency / 1000) & 0xff;
	cmd.args[5] = ((if_frequency / 1000) >> 8) & 0xff;
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set analog AGC config */
	memcpy(cmd.args, "\x14\x00\x07\x06\x32\xc8", 6);
	cmd.wlen = 6;
	cmd.rlen = 4;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set analog video mode */
	memcpy(cmd.args, "\x14\x00\x04\x06\x00\x00", 6);
	cmd.args[4] = system | color;
#if 1 /* can use dev->inversion if assumed it applies to both digital/analog */
	if (invert_analog)
		cmd.args[5] |= 0x02;
#else
	if (dev->inversion)
		cmd.args[5] |= 0x02;
#endif
	cmd.wlen = 6;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

	/* set analog frequency */
	memcpy(cmd.args, "\x41\x01\x00\x00\x00\x00\x00\x00", 8);
	cmd.args[4] = (freq >>  0) & 0xff;
	cmd.args[5] = (freq >>  8) & 0xff;
	cmd.args[6] = (freq >> 16) & 0xff;
	cmd.args[7] = (freq >> 24) & 0xff;
	cmd.wlen = 8;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret)
		goto err;

#if 1 /* testing */
	/* get tuner status, RSSI values */
	memcpy(cmd.args, "\x42\x01", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	ret = si2157_cmd_execute(client, &cmd);

	dev_info(&client->dev, "%s: tuner status: ret=%d rssi=%d mode=%x freq=%d\n",
		__func__, ret, cmd.args[3], cmd.args[8],
		(cmd.args[7]<<24 | cmd.args[6]<<16 |
		cmd.args[5]<<8 | cmd.args[4]));
#endif

	dev->bandwidth = bandwidth;

	return 0;
err:
	dev->bandwidth = 0;
	dev->frequency = 0;
	dev->if_frequency = 0;
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int si2157_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);

	*frequency = dev->frequency;
	dev_info(&client->dev, "%s: freq=%u\n", __func__, dev->frequency);
	return 0;
}

static int si2157_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);

	*bandwidth = dev->bandwidth;
	dev_info(&client->dev, "%s: bandwidth=%u\n", __func__, dev->bandwidth);
	return 0;
}

static int si2157_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = i2c_get_clientdata(client);

	*frequency = dev->if_frequency & ~1; /* strip analog IF indicator bit */
	return 0;
}

static int si2157_get_rf_strength(struct dvb_frontend *fe,
				       u16 *signal_strength)
{
	struct i2c_client *client = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct si2157_cmd cmd;
	int ret;

	memcpy(cmd.args, "\x42\x00", 2);
	cmd.wlen = 2;
	cmd.rlen = 12;
	ret = si2157_cmd_execute(client, &cmd);
	if (ret) {
		dprintk("failed=%d", ret);
		return -1;
	}

	c->strength.stat[0].scale = FE_SCALE_DECIBEL;
	c->strength.stat[0].svalue = (s8)cmd.args[3] * 1000;
	*signal_strength = cmd.args[3];

	return 0;
}

static void si2157_release(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = NULL;

	pr_info("%s: client=%p\n", __func__, client);
	if (client == NULL)
		return;

	dev = i2c_get_clientdata(client);
	pr_info("%s: dev=%p\n", __func__, dev);
	if (dev == NULL)
		return;

	/* only remove dev reference from final instance */
	if (hybrid_tuner_report_instance_count(dev) == 1)
		i2c_set_clientdata(client, NULL);

	mutex_lock(&si2157_list_mutex);
	hybrid_tuner_release_state(dev);
	mutex_unlock(&si2157_list_mutex);

	fe->tuner_priv = NULL;
}

static int si2157_setup_configuration(struct dvb_frontend *fe,
					struct si2157_config *cfg)
{
	struct i2c_client *client = fe->tuner_priv;
	struct si2157_dev *dev = NULL;

	pr_info("%s: client=%p\n", __func__, client);
	if (client == NULL)
		return -1;

	dev = i2c_get_clientdata(client);
	pr_info("%s: dev=%p\n", __func__, dev);
	if (dev == NULL)
		return -1;

	if (cfg) {
		pr_info("%s(0x%02X): dvb driver submitted configuration; port=%d invert=%d\n",
				__func__, dev->addr,
				cfg->if_port, cfg->inversion);
		dev->inversion = cfg->inversion;
		dev->if_port   = cfg->if_port;
	} else {
		pr_info("%s(0x%02X): default configuration\n",
				__func__, dev->addr);
		dev->inversion = true;
		dev->if_port   = 1;
	}
	return 0;
}
static const struct dvb_tuner_ops si2157_ops = {
	.info = {
		.name           = "Silicon Labs Si2141/2146/2147/2148/2157/2158",
		.frequency_min  = 42000000,
		.frequency_max  = 870000000,
	},

	.init			= si2157_init,
	.sleep			= si2157_sleep,
	.set_params		= si2157_set_params,
	.set_analog_params      = si2157_set_analog_params,
	.release                = si2157_release,
	.get_frequency          = si2157_get_frequency,
	.get_bandwidth          = si2157_get_bandwidth,
	.get_if_frequency	= si2157_get_if_frequency,
	.get_rf_strength	= si2157_get_rf_strength,
};

static int si2157_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct si2157_config *cfg = client->dev.platform_data;
	struct dvb_frontend *fe = cfg->fe;
	struct si2157_dev *dev = NULL;
	unsigned short addr = client->addr;
	int ret = 0;

	pr_info("%s: probing si2157 tuner fe=%p cfg=%p addr=0X%2x\n",
			__func__, fe, cfg, addr);
	fe->tuner_priv = client;

	if (si2157_attach(fe, (u8)addr, client->adapter, cfg) == NULL) {
		dev_err(&client->dev, "%s: attaching si2157 tuner failed\n",
				__func__);
		goto err;
	}
	fe->ops.tuner_ops.release = NULL;

	dev = i2c_get_clientdata(client);
	dev->chiptype = (u8)id->driver_data;

	dprintk("Silicon Labs %s successfully attached",
			dev->chiptype == SI2157_CHIPTYPE_SI2141 ?  "Si2141" :
			dev->chiptype == SI2157_CHIPTYPE_SI2146 ?
			"Si2146" : "Si2147/2148/2157/2158");

	return 0;

err:
	dprintk("failed=%d", ret);
	return ret;
}

static int si2157_remove(struct i2c_client *client)
{
	struct si2157_dev *dev = i2c_get_clientdata(client);
	struct dvb_frontend *fe = NULL;

	if (dev == NULL) {
		return 0;
	}
	fe = dev->fe;

	dprintk("");

	/* stop statistics polling */
	cancel_delayed_work_sync(&dev->stat_work);

#ifdef CONFIG_MEDIA_CONTROLLER_DVB
	if (dev->mdev)
		media_device_unregister_entity(&dev->ent);
#endif

	memset(&fe->ops.tuner_ops, 0, sizeof(struct dvb_tuner_ops));
	si2157_release(fe);

	return 0;
}

static const struct i2c_device_id si2157_id_table[] = {
	{"si2157", SI2157_CHIPTYPE_SI2157},
	{"si2146", SI2157_CHIPTYPE_SI2146},
	{"si2141", SI2157_CHIPTYPE_SI2141},
	{}
};
MODULE_DEVICE_TABLE(i2c, si2157_id_table);

static struct i2c_driver si2157_driver = {
	.driver = {
		.name		     = "si2157",
		.suppress_bind_attrs = true,
	},
	.probe		= si2157_probe,
	.remove		= si2157_remove,
	.id_table	= si2157_id_table,
};

module_i2c_driver(si2157_driver);

struct dvb_frontend *si2157_attach(struct dvb_frontend *fe, u8 addr,
		struct i2c_adapter *i2c,
		struct si2157_config *cfg)
{
	struct i2c_client *client = NULL;
	struct si2157_dev *dev = NULL;
	struct si2157_cmd cmd;
	int instance = 0, ret;

	pr_info("%s (%d-%04x)\n", __func__,
	       i2c ? i2c_adapter_id(i2c) : 0,
	       addr);

	mutex_lock(&si2157_list_mutex);

	if (!cfg) {
		pr_info("no configuration submitted\n");
		goto fail;
	}

	if (!fe) {
		pr_info("fe is NULL\n");
		goto fail;
	}

	client = fe->tuner_priv;
	if (!client) {
		pr_info("client is NULL\n");
		goto fail;
	}

	instance = hybrid_tuner_request_state(struct si2157_dev, dev,
			hybrid_tuner_instance_list,
			i2c, addr, "si2157");

	pr_info("%s: instance=%d\n", __func__, instance);
	switch (instance) {
	case 0:
		goto fail;
	case 1:
		/* new tuner instance */
		pr_info("%s(): new instance for tuner @0x%02x\n",
				__func__, addr);
		dev->addr = addr;
		i2c_set_clientdata(client, dev);

		si2157_setup_configuration(fe, cfg);

		dev->fe = fe;
		/* BUGBUG - should chiptype come from config? */
		dev->chiptype = (u8)SI2157_CHIPTYPE_SI2157;
		dev->if_frequency = 0;

		mutex_init(&dev->i2c_mutex);
		break;
	default:
		/* existing tuner instance */
		pr_info("%s(): using existing instance for tuner @0x%02x\n",
				 __func__, addr);

		/* allow dvb driver to override configuration settings */
		if (cfg)
			si2157_setup_configuration(fe, cfg);

		break;
	}

	/* check if the tuner is there */
	cmd.wlen = 0;
	cmd.rlen = 1;
	ret = si2157_cmd_execute(client, &cmd);
	/* verify no i2c error and CTS is set */
	if (ret && (ret != -EAGAIN)) {
		pr_info("no HW found ret=%d\n", ret);
		goto fail;
	}

	memcpy(&fe->ops.tuner_ops, &si2157_ops, sizeof(struct dvb_tuner_ops));

#ifdef CONFIG_MEDIA_CONTROLLER
	if (instance == 1 && cfg->mdev) {
		pr_info("cfg->mdev=%p\n", cfg->mdev);
		dev->mdev = cfg->mdev;

		dev->ent.name = KBUILD_MODNAME;
		dev->ent.function = MEDIA_ENT_F_TUNER;

		dev->pad[TUNER_PAD_RF_INPUT].flags = MEDIA_PAD_FL_SINK;
		dev->pad[TUNER_PAD_OUTPUT].flags = MEDIA_PAD_FL_SOURCE;
		dev->pad[TUNER_PAD_AUD_OUT].flags = MEDIA_PAD_FL_SOURCE;

		ret = media_entity_pads_init(&dev->ent, TUNER_NUM_PADS,
					     &dev->pad[0]);

		if (ret)
			goto fail;

		ret = media_device_register_entity(cfg->mdev, &dev->ent);
		if (ret) {
			pr_info("media_device_regiser_entity returns %d\n", ret);
			media_entity_cleanup(&dev->ent);
			goto fail;
		}
	}
#endif

	mutex_unlock(&si2157_list_mutex);

	return fe;

fail:
	pr_info("%s: Failed\n", __func__);
	mutex_unlock(&si2157_list_mutex);

	if (fe && (instance == 1))
		si2157_release(fe);
	return NULL;
}
EXPORT_SYMBOL(si2157_attach);

MODULE_DESCRIPTION("Silicon Labs Si2141/2146/2147/2148/2157/2158 silicon tuner driver");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(SI2158_A20_FIRMWARE);
MODULE_FIRMWARE(SI2141_A10_FIRMWARE);
