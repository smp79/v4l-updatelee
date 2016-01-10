#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>

#include "saa716x_mod.h"

#include "saa716x_gpio_reg.h"
#include "saa716x_greg_reg.h"
#include "saa716x_msi_reg.h"
#include "saa716x_dma_reg.h"
#include "saa716x_fgpi_reg.h"

#include "saa716x_adap.h"
#include "saa716x_i2c.h"
#include "saa716x_msi.h"
#include "saa716x_budget.h"
#include "saa716x_gpio.h"
#include "saa716x_rom.h"
#include "saa716x_spi.h"
#include "saa716x_priv.h"

#include "mb86a16.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "stv6120.h"
#include "stv0910.h"
#include "tas2101.h"
#include "av201x.h"
#include "cx24117.h"
#include "isl6422.h"
#include "ds3103.h"
#include "ts2022.h"
#include "stb6100.h"
#include "stb6100_cfg.h"
#include "tda18212.h"
#include "cxd2820r.h"
#include "si2168.h"
#include "si2157.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

unsigned int int_type;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

static int tsout = 1;
module_param(tsout, int, 0644);
MODULE_PARM_DESC(tsout, "TBS 6925 output format 1=TS, 0=BB (default:1)");

#define DRIVER_NAME	"SAA716x Budget"

static int saa716x_budget_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_budget_pci_probe ERROR: out of memory\n");
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->verbose	= verbose;
	saa716x->int_type	= int_type;
	saa716x->pdev		= pdev;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x CGU Init failed");
		goto fail1;
	}

	err = saa716x_core_boot(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Core Boot failed");
		goto fail2;
	}
	dprintk(SAA716x_DEBUG, 1, "SAA716x Core Boot Success");

	err = saa716x_msi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x MSI Init failed");
		goto fail2;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Jetpack core initialization failed");
		goto fail1;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail3;
	}

	saa716x_gpio_init(saa716x);
#if 0
	err = saa716x_dump_eeprom(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM dump failed");
	}

	err = saa716x_eeprom_data(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM read failed");
	}

	/* set default port mapping */
	SAA716x_EPWR(GREG, GREG_VI_CTRL, 0x04080FA9);
	/* enable FGPI3 and FGPI1 for TS input from Port 2 and 6 */
	SAA716x_EPWR(GREG, GREG_FGPI_CTRL, 0x321);
#endif

	/* set default port mapping */
	SAA716x_EPWR(GREG, GREG_VI_CTRL, 0x2C688F0A);
	/* enable FGPI3, FGPI2, FGPI1 and FGPI0 for TS input from Port 2 and 6 */
	SAA716x_EPWR(GREG, GREG_FGPI_CTRL, 0x322);

	err = saa716x_dvb_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x DVB initialization failed");
		goto fail4;
	}

	return 0;

fail4:
	saa716x_dvb_exit(saa716x);
fail3:
	saa716x_i2c_exit(saa716x);
fail2:
	saa716x_pci_exit(saa716x);
fail1:
	kfree(saa716x);
fail0:
	return err;
}

static void saa716x_budget_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_dvb_exit(saa716x);
	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_budget_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;

	u32 stat_h, stat_l, mask_h, mask_l;

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

	stat_l = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	stat_h = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	mask_l = SAA716x_EPRD(MSI, MSI_INT_ENA_L);
	mask_h = SAA716x_EPRD(MSI, MSI_INT_ENA_H);

	if (!((stat_l & mask_l) || (stat_h & mask_h)))
		return IRQ_NONE;

	if (stat_l)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, stat_l);

	if (stat_h)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, stat_h);

	saa716x_msi_event(saa716x, stat_l, stat_h);
#if 0
	dprintk(SAA716x_DEBUG, 1, "VI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(VI0, INT_STATUS),
		SAA716x_EPRD(VI1, INT_STATUS),
		SAA716x_EPRD(VI0, INT_ENABLE),
		SAA716x_EPRD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(FGPI0, INT_STATUS),
		SAA716x_EPRD(FGPI1, INT_STATUS),
		SAA716x_EPRD(FGPI0, INT_ENABLE),
		SAA716x_EPRD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=<%02x> 3=<%02x>, CTL 2=<%02x> 3=<%02x>",
		SAA716x_EPRD(FGPI2, INT_STATUS),
		SAA716x_EPRD(FGPI3, INT_STATUS),
		SAA716x_EPRD(FGPI2, INT_ENABLE),
		SAA716x_EPRD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(AI0, AI_STATUS),
		SAA716x_EPRD(AI1, AI_STATUS),
		SAA716x_EPRD(AI0, AI_CTL),
		SAA716x_EPRD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(I2C_A, INT_STATUS),
		SAA716x_EPRD(I2C_B, INT_STATUS),
		SAA716x_EPRD(I2C_A, INT_ENABLE),
		SAA716x_EPRD(I2C_B, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=<%02x>, CTL=<%02x>",
		SAA716x_EPRD(DCS, DCSC_INT_STATUS),
		SAA716x_EPRD(DCS, DCSC_INT_ENABLE));
#endif

	if (stat_l) {
		if (stat_l & MSI_INT_TAGACK_FGPI_0) {
			tasklet_schedule(&saa716x->fgpi[0].tasklet);
		}
		if (stat_l & MSI_INT_TAGACK_FGPI_1) {
			tasklet_schedule(&saa716x->fgpi[1].tasklet);
		}
		if (stat_l & MSI_INT_TAGACK_FGPI_2) {
			tasklet_schedule(&saa716x->fgpi[2].tasklet);
		}
		if (stat_l & MSI_INT_TAGACK_FGPI_3) {
			tasklet_schedule(&saa716x->fgpi[3].tasklet);
		}
	}

	return IRQ_HANDLED;
}

static void demux_worker(unsigned long data)
{
	struct saa716x_fgpi_stream_port *fgpi_entry = (struct saa716x_fgpi_stream_port *)data;
	struct saa716x_dev *saa716x = fgpi_entry->saa716x;
	struct dvb_demux *demux;
	u32 fgpi_index;
	u32 i;
	u32 write_index;

	fgpi_index = fgpi_entry->dma_channel - 6;
	demux = NULL;
	for (i = 0; i < saa716x->config->adapters; i++) {
		if (saa716x->config->adap_config[i].ts_port == fgpi_index) {
			demux = &saa716x->saa716x_adap[i].demux;
			break;
		}
	}
	if (demux == NULL) {
		printk(KERN_ERR "%s: unexpected channel %u\n",
		       __func__, fgpi_entry->dma_channel);
		return;
	}

	write_index = saa716x_fgpi_get_write_index(saa716x, fgpi_index);
	if (write_index < 0)
		return;

	if (write_index == fgpi_entry->read_index) {
		printk(KERN_DEBUG "%s: called but nothing to do\n", __func__);
		return;
	}

	do {
		u8 *data = (u8 *)fgpi_entry->dma_buf[fgpi_entry->read_index].mem_virt;

		pci_dma_sync_sg_for_cpu(saa716x->pdev,
			fgpi_entry->dma_buf[fgpi_entry->read_index].sg_list,
			fgpi_entry->dma_buf[fgpi_entry->read_index].list_len,
			PCI_DMA_FROMDEVICE);

		if (tsout == 0) {
			dvb_dmx_swfilter_data(demux, FE_DFMT_BB_FRAME, data, 348 * 188);
		} else {
			dvb_dmx_swfilter(demux, data, 348 * 188);
		}

		fgpi_entry->read_index = (fgpi_entry->read_index + 1) & 7;
	} while (write_index != fgpi_entry->read_index);
}

static int saa716x_set_frame_ops(struct dvb_frontend *fe, struct dvb_frame frame_ops)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "frame_ops.frame_size = %d", frame_ops.frame_size);
	saa716x->saa716x_adap->demux.frame_ops = frame_ops;

	return 0;
}

#define SAA716x_MODEL_TWINHAN_VP3071	"Twinhan/Azurewave VP-3071"
#define SAA716x_DEV_TWINHAN_VP3071	"2x DVB-T"

static int saa716x_vp3071_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_vp3071_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP3071,
	.dev_type		= SAA716x_DEV_TWINHAN_VP3071,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_vp3071_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};


#define SAA716x_MODEL_TWINHAN_VP1028	"Twinhan/Azurewave VP-1028"
#define SAA716x_DEV_TWINHAN_VP1028	"DVB-S"

static int vp1028_dvbs_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct saa716x_dev *saa716x = fe->dvb->priv;

	switch (voltage) {
	case SEC_VOLTAGE_13:
		dprintk(SAA716x_ERROR, 1, "Polarization=[13V]");
		break;
	case SEC_VOLTAGE_18:
		dprintk(SAA716x_ERROR, 1, "Polarization=[18V]");
		break;
	case SEC_VOLTAGE_OFF:
		dprintk(SAA716x_ERROR, 1, "Frontend (dummy) POWERDOWN");
		break;
	default:
		dprintk(SAA716x_ERROR, 1, "Invalid = (%d)", (u32 ) voltage);
		return -EINVAL;
	}

	return 0;
}

struct mb86a16_config vp1028_mb86a16_config = {
	.demod_address	= 0x08,
	.set_voltage	= vp1028_dvbs_set_voltage,
};

static int saa716x_vp1028_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[1];

	if (count == 0) {

		mutex_lock(&saa716x->adap_lock);

		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Power ON", count);
		saa716x_gpio_set_output(saa716x, 10);
		msleep(1);

		/* VP-1028 has inverted power supply control */
		saa716x_gpio_write(saa716x, 10, 1); /* set to standby */
		saa716x_gpio_write(saa716x, 10, 0); /* switch it on */
		msleep(100);

		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Reset", count);
		saa716x_gpio_set_output(saa716x, 12);
		msleep(1);

		/* reset demodulator (Active LOW) */
		saa716x_gpio_write(saa716x, 12, 1);
		msleep(100);
		saa716x_gpio_write(saa716x, 12, 0);
		msleep(100);
		saa716x_gpio_write(saa716x, 12, 1);
		msleep(100);

		mutex_unlock(&saa716x->adap_lock);

		dprintk(SAA716x_ERROR, 1, "Probing for MB86A16 (DVB-S/DSS)");
		adapter->fe = dvb_attach(mb86a16_attach, &vp1028_mb86a16_config, &i2c->i2c_adapter);
		if (adapter->fe) {
			dprintk(SAA716x_ERROR, 1, "found MB86A16 DVB-S/DSS frontend @0x%02x",
				vp1028_mb86a16_config.demod_address);

		} else {
			goto exit;
		}
		dprintk(SAA716x_ERROR, 1, "Done!");
	}

	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_vp1028_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP1028,
	.dev_type		= SAA716x_DEV_TWINHAN_VP1028,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= saa716x_vp1028_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};


#define SAA716x_MODEL_TWINHAN_VP6002	"Twinhan/Azurewave VP-6002"
#define SAA716x_DEV_TWINHAN_VP6002	"DVB-S"

static int saa716x_vp6002_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_vp6002_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6002,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6002,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= saa716x_vp6002_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};


#define SAA716x_MODEL_KNC1_DUALS2	"KNC One Dual S2"
#define SAA716x_DEV_KNC1_DUALS2		"1xDVB-S + 1xDVB-S/S2"

static int saa716x_knc1_duals2_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_knc1_duals2_config = {
	.model_name		= SAA716x_MODEL_KNC1_DUALS2,
	.dev_type		= SAA716x_DEV_KNC1_DUALS2,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_knc1_duals2_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};

#define SAA716x_MODEL_SKYSTAR2_EXPRESS_HD	"SkyStar 2 eXpress HD"
#define SAA716x_DEV_SKYSTAR2_EXPRESS_HD		"DVB-S/S2"

static struct stv090x_config skystar2_stv090x_config = {
	.device			= STV0903,
	.demod_mode		= STV090x_SINGLE,
	.clk_mode		= STV090x_CLK_EXT,

	.xtal			= 8000000,
	.address		= 0x68,

	.ts1_mode		= STV090x_TSMODE_DVBCI,
	.ts2_mode		= STV090x_TSMODE_SERIAL_CONTINUOUS,

	.repeater_level		= STV090x_RPTLEVEL_16,
	.adc1_range		= STV090x_ADC_1Vpp,
	.tuner_bbgain		= 6,

	.tuner_init		= NULL,
	.tuner_sleep		= NULL,
	.tuner_set_mode		= NULL,
	.tuner_set_frequency	= NULL,
	.tuner_get_frequency	= NULL,
	.tuner_set_bandwidth	= NULL,
	.tuner_get_bandwidth	= NULL,
	.tuner_set_bbgain	= NULL,
	.tuner_get_bbgain	= NULL,
	.tuner_set_refclk	= NULL,
	.tuner_get_status	= NULL,
	.name					= "STV090x Skystar 2 Express HD",
};

static int skystar2_set_voltage(struct dvb_frontend *fe,
				enum fe_sec_voltage voltage)
{
	int err;
	u8 en = 0;
	u8 sel = 0;

	switch (voltage) {
	case SEC_VOLTAGE_OFF:
		en = 0;
		break;
	case SEC_VOLTAGE_13:
		en = 1;
		sel = 0;
		break;
	case SEC_VOLTAGE_18:
		en = 1;
		sel = 1;
		break;
	default:
		break;
	}

	err = skystar2_stv090x_config.set_gpio(fe, 2, 0, en, 0);
	if (err < 0)
		goto exit;
	err = skystar2_stv090x_config.set_gpio(fe, 3, 0, sel, 0);
	if (err < 0)
		goto exit;

	return 0;
exit:
	return err;
}

static int skystar2_voltage_boost(struct dvb_frontend *fe, long arg)
{
	int err;
	u8 value;

	if (arg)
		value = 1;
	else
		value = 0;

	err = skystar2_stv090x_config.set_gpio(fe, 4, 0, value, 0);
	if (err < 0)
		goto exit;

	return 0;
exit:
	return err;
}

static struct stv6110x_config skystar2_stv6110x_config = {
	.addr			= 0x60,
	.refclk			= 16000000,
	.clk_div		= 2,
};

static int skystar2_express_hd_frontend_attach(struct saa716x_adapter *adapter,
					       int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[SAA716x_I2C_BUS_B];
	struct stv6110x_devctl *ctl;

	if (count < saa716x->config->adapters) {
		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init",
			count);
		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count,
			saa716x->pdev->subsystem_device);

		saa716x_gpio_set_output(saa716x, 26);

		/* Reset the demodulator */
		saa716x_gpio_write(saa716x, 26, 1);
		msleep(10);
		saa716x_gpio_write(saa716x, 26, 0);
		msleep(10);
		saa716x_gpio_write(saa716x, 26, 1);
		msleep(10);

		adapter->fe = dvb_attach(stv090x_attach,
					 &skystar2_stv090x_config,
					 &i2c->i2c_adapter,
					 STV090x_DEMODULATOR_0);

		if (adapter->fe == NULL) {
			goto exit;
		}
		dprintk(SAA716x_NOTICE, 1, "found STV0903 @0x%02x", skystar2_stv090x_config.address);

		ctl = dvb_attach(stv6110x_attach,
				 adapter->fe,
				 &skystar2_stv6110x_config,
				 &i2c->i2c_adapter);

		if (ctl == NULL) {
			goto exit;
		}
		dprintk(SAA716x_NOTICE, 1, "found STV6110(A) @0x%02x", skystar2_stv6110x_config.addr);

		skystar2_stv090x_config.tuner_init	    = ctl->tuner_init;
		skystar2_stv090x_config.tuner_sleep	    = ctl->tuner_sleep;
		skystar2_stv090x_config.tuner_set_mode	    = ctl->tuner_set_mode;
		skystar2_stv090x_config.tuner_set_frequency = ctl->tuner_set_frequency;
		skystar2_stv090x_config.tuner_get_frequency = ctl->tuner_get_frequency;
		skystar2_stv090x_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;
		skystar2_stv090x_config.tuner_get_bandwidth = ctl->tuner_get_bandwidth;
		skystar2_stv090x_config.tuner_set_bbgain    = ctl->tuner_set_bbgain;
		skystar2_stv090x_config.tuner_get_bbgain    = ctl->tuner_get_bbgain;
		skystar2_stv090x_config.tuner_set_refclk    = ctl->tuner_set_refclk;
		skystar2_stv090x_config.tuner_get_status    = ctl->tuner_get_status;

		/* call the init function once to initialize
		   tuner's clock output divider and demod's
		   master clock */
		if (adapter->fe->ops.init)
			adapter->fe->ops.init(adapter->fe);

		adapter->fe->ops.set_voltage				= skystar2_set_voltage;
		adapter->fe->ops.enable_high_lnb_voltage	= skystar2_voltage_boost;
		adapter->fe->ops.set_frame_ops				= saa716x_set_frame_ops;
		
		dprintk(SAA716x_ERROR, 1, "Done!");
		return 0;
	}
exit:
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config skystar2_express_hd_config = {
	.model_name		= SAA716x_MODEL_SKYSTAR2_EXPRESS_HD,
	.dev_type		= SAA716x_DEV_SKYSTAR2_EXPRESS_HD,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= skystar2_express_hd_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		}
	}
};

#define SAA716x_MODEL_TBS6284		"TurboSight TBS 6284"
#define SAA716x_DEV_TBS6284		"DVB-T/T2/C"

static struct cxd2820r_config cxd2820r_config[] = {
	{
		.i2c_address = 0x6c, /* (0xd8 >> 1) */
		.ts_mode = 0x38,
	},
	{
		.i2c_address = 0x6d, /* (0xda >> 1) */
		.ts_mode = 0x38,
	}
};

static struct tda18212_config tda18212_config[] = {
	{
		.if_dvbt_6 = 3550,
		.if_dvbt_7 = 3700,
		.if_dvbt_8 = 4150,
		.if_dvbt2_6 = 3250,
		.if_dvbt2_7 = 4000,
		.if_dvbt2_8 = 4000,
		.if_dvbc = 5000,
		.loop_through = 1,
		.xtout = 1
	},
	{
		.if_dvbt_6 = 3550,
		.if_dvbt_7 = 3700,
		.if_dvbt_8 = 4150,
		.if_dvbt2_6 = 3250,
		.if_dvbt2_7 = 4000,
		.if_dvbt2_8 = 4000,
		.if_dvbc = 5000,
		.loop_through = 0,
		.xtout = 0
	}
};

static int saa716x_tbs6284_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct saa716x_i2c *i2c = &dev->i2c[1 - (count >> 1)];

	struct i2c_client *i2c_client;
	struct i2c_board_info board_info = {
		.type = "tda18212",
		.addr = count ? 0x63 : 0x60,
		.platform_data = &tda18212_config[count & 1],
	};

	if (count > 3)
		goto err;

	/* reset */
	if (count == 0) {
		saa716x_gpio_set_output(dev, 22);
		saa716x_gpio_write(dev, 22, 0);
		msleep(200);
		saa716x_gpio_write(dev, 22, 1);
		msleep(400);
	} else if (count == 2) {
		saa716x_gpio_set_output(dev, 12);
		saa716x_gpio_write(dev, 12, 0);
		msleep(200);
		saa716x_gpio_write(dev, 12, 1);
		msleep(400);
	}

	/* attach frontend */
	adapter->fe = dvb_attach(cxd2820r_attach, &cxd2820r_config[count & 1],
				&i2c->i2c_adapter, NULL);
	if (!adapter->fe)
		goto err;

	/* attach tuner */
	tda18212_config[count & 1].fe = adapter->fe;
	request_module("tda18212");
	i2c_client = i2c_new_device(&i2c->i2c_adapter, &board_info);
	if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
		dvb_frontend_detach(adapter->fe);
		goto err;
	}

	if (!try_module_get(i2c_client->dev.driver->owner)) {
		i2c_unregister_device(i2c_client);
		dvb_frontend_detach(adapter->fe);
		goto err;
	}

	adapter->i2c_client_tuner = i2c_client;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);
	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6284_config = {
	.model_name		= SAA716x_MODEL_TBS6284,
	.dev_type		= SAA716x_DEV_TBS6284,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 4,
	.frontend_attach	= saa716x_tbs6284_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 2,
			.worker = demux_worker
		},
		{
			/* adapter 2 */
			.ts_port = 1,
			.worker = demux_worker
		},
		{
			/* adapter 3 */
			.ts_port = 0,
			.worker = demux_worker
		}
	},
};


#define SAA716x_MODEL_TBS6280		"TurboSight TBS 6280"
#define SAA716x_DEV_TBS6280		"DVB-T/T2/C"

static int saa716x_tbs6280_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c0 = &saa716x->i2c[SAA716x_I2C_BUS_A];

	struct i2c_client *i2c_client;
	struct i2c_board_info board_info = {
		.type = "tda18212",
		.addr = count ? 0x63 : 0x60,
		.platform_data = &tda18212_config[count & 1],
	};

	switch (count) {
	case 0:
		/* reset */
		saa716x_gpio_set_output(saa716x, 2);
		saa716x_gpio_write(saa716x, 2, 0);
		msleep(200);
		saa716x_gpio_write(saa716x, 2, 1);
		msleep(400);
	case 1:
		dprintk(SAA716x_ERROR, 1, "Probing for cxd2820r (%d)", count);
		adapter->fe = dvb_attach(cxd2820r_attach, &cxd2820r_config[count],
					&i2c0->i2c_adapter, NULL);
		if (!adapter->fe)
			goto err;

		/* attach tuner */
		tda18212_config[count & 1].fe = adapter->fe;
		request_module("tda18212");
		i2c_client = i2c_new_device(&i2c0->i2c_adapter, &board_info);
		if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
			dvb_frontend_detach(adapter->fe);
			goto err;
		}

		if (!try_module_get(i2c_client->dev.driver->owner)) {
			i2c_unregister_device(i2c_client);
			dvb_frontend_detach(adapter->fe);
			goto err;
		}

		adapter->i2c_client_tuner = i2c_client;
		break;
	default:
		goto err;
	}

	dprintk(SAA716x_ERROR, 1, "Done!");
	return 0;
err:
	printk(KERN_ERR "%s: frontend initialization failed\n",
					adapter->saa716x->config->model_name);
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6280_config = {
	.model_name		= SAA716x_MODEL_TBS6280,
	.dev_type		= SAA716x_DEV_TBS6280,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6280_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6281		"TurboSight TBS 6281"
#define SAA716x_DEV_TBS6281		"DVB-T/T2/C"

static int saa716x_tbs6281_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct i2c_adapter *i2c_adapter;
	struct i2c_client *i2c_client;
	struct i2c_board_info info;
	struct si2168_config si2168_config;
	struct si2157_config si2157_config;

	if (count > 1)
		goto err;

	/* reset */
	saa716x_gpio_set_output(dev, count ? 2 : 16);
	saa716x_gpio_write(dev, count ? 2 : 16, 0);
	msleep(50);
	saa716x_gpio_write(dev, count ? 2 : 16, 1);
	msleep(100);

	/* attach demod */
	si2168_config.i2c_adapter = &i2c_adapter;
	si2168_config.fe = &adapter->fe;
	si2168_config.ts_mode = SI2168_TS_PARALLEL;
	memset(&info, 0, sizeof(struct i2c_board_info));
	strlcpy(info.type, "si2168", I2C_NAME_SIZE);
	info.addr = 0x64;
	info.platform_data = &si2168_config;
	request_module(info.type);
	i2c_client = i2c_new_device(&dev->i2c[1 - count].i2c_adapter, &info);
	if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
		goto err;
	}
	if (!try_module_get(i2c_client->dev.driver->owner)) {
		i2c_unregister_device(i2c_client);
		goto err;
	}
	adapter->i2c_client_demod = i2c_client;

	/* attach tuner */
	si2157_config.fe = adapter->fe;
	memset(&info, 0, sizeof(struct i2c_board_info));
	strlcpy(info.type, "si2157", I2C_NAME_SIZE);
	info.addr = 0x60;
	info.platform_data = &si2157_config;
	request_module(info.type);
	i2c_client = i2c_new_device(i2c_adapter, &info);
	if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
		module_put(adapter->i2c_client_demod->dev.driver->owner);
		i2c_unregister_device(adapter->i2c_client_demod);
		goto err;
	}
	if (!try_module_get(i2c_client->dev.driver->owner)) {
		i2c_unregister_device(i2c_client);
		module_put(adapter->i2c_client_demod->dev.driver->owner);
		i2c_unregister_device(adapter->i2c_client_demod);
		goto err;
	}
	adapter->i2c_client_tuner = i2c_client;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6281_config = {
	.model_name		= SAA716x_MODEL_TBS6281,
	.dev_type		= SAA716x_DEV_TBS6281,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6281_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6285		"TurboSight TBS 6285"
#define SAA716x_DEV_TBS6285		"DVB-T/T2/C"

static int saa716x_tbs6285_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct i2c_adapter *i2c_adapter;
	struct i2c_client *i2c_client;
	struct i2c_board_info info;
	struct si2168_config si2168_config;
	struct si2157_config si2157_config;

	if (count > 3)
		goto err;

	/* attach demod */
	si2168_config.i2c_adapter = &i2c_adapter;
	si2168_config.fe = &adapter->fe;
	si2168_config.ts_mode = SI2168_TS_SERIAL;
	memset(&info, 0, sizeof(struct i2c_board_info));
	strlcpy(info.type, "si2168", I2C_NAME_SIZE);
	info.addr = ((count == 0) || (count == 2)) ? 0x64 : 0x66;
	info.platform_data = &si2168_config;
	request_module(info.type);
	i2c_client = i2c_new_device( ((count == 0) || (count == 1)) ?
		&dev->i2c[1].i2c_adapter : &dev->i2c[0].i2c_adapter,
		&info);
	if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
		goto err;
	}

	if (!try_module_get(i2c_client->dev.driver->owner)) {
		i2c_unregister_device(i2c_client);
		goto err;
	}
	adapter->i2c_client_demod = i2c_client;

	/* attach tuner */
	si2157_config.fe = adapter->fe;
	memset(&info, 0, sizeof(struct i2c_board_info));
	strlcpy(info.type, "si2157", I2C_NAME_SIZE);
	info.addr = ((count == 0) || (count == 2)) ? 0x62 : 0x60;
	info.platform_data = &si2157_config;
	request_module(info.type);
	i2c_client = i2c_new_device(i2c_adapter, &info);
	if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
		module_put(adapter->i2c_client_demod->dev.driver->owner);
		i2c_unregister_device(adapter->i2c_client_demod);
		goto err;
	}
	if (!try_module_get(i2c_client->dev.driver->owner)) {
		i2c_unregister_device(i2c_client);
		module_put(adapter->i2c_client_demod->dev.driver->owner);
		i2c_unregister_device(adapter->i2c_client_demod);
		goto err;
	}
	adapter->i2c_client_tuner = i2c_client;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6285_config = {
	.model_name		= SAA716x_MODEL_TBS6285,
	.dev_type		= SAA716x_DEV_TBS6285,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 4,
	.frontend_attach	= saa716x_tbs6285_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 2,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 1,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 0,
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6220		"TurboSight TBS 6220"
#define SAA716x_DEV_TBS6220		"DVB-T/T2/C"

static int saa716x_tbs6220_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[SAA716x_I2C_BUS_A];

	struct i2c_client *i2c_client;
	struct i2c_board_info board_info = {
		.type = "tda18212",
		.addr = count ? 0x63 : 0x60,
		.platform_data = &tda18212_config[count & 1],
	};

	if (count == 0) {
		dprintk(SAA716x_ERROR, 1, "Probing for cxd2820r (%d)", count);
		adapter->fe = dvb_attach(cxd2820r_attach, &cxd2820r_config[count],
					&i2c->i2c_adapter, NULL);
		if (!adapter->fe)
			goto err;

		/* attach tuner */
		tda18212_config[count & 1].fe = adapter->fe;
		request_module("tda18212");
		i2c_client = i2c_new_device(&i2c->i2c_adapter, &board_info);
		if (i2c_client == NULL || i2c_client->dev.driver == NULL) {
			dvb_frontend_detach(adapter->fe);
			goto err;
		}

		if (!try_module_get(i2c_client->dev.driver->owner)) {
			i2c_unregister_device(i2c_client);
			dvb_frontend_detach(adapter->fe);
			goto err;
		}

		adapter->i2c_client_tuner = i2c_client;
		dprintk(SAA716x_ERROR, 1, "Done!");
		return 0;
	}

err:
	printk(KERN_ERR "%s: frontend initialization failed\n",
					adapter->saa716x->config->model_name);
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6220_config = {
	.model_name		= SAA716x_MODEL_TBS6220,
	.dev_type		= SAA716x_DEV_TBS6220,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= saa716x_tbs6220_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6922		"TurboSight TBS 6922"
#define SAA716x_DEV_TBS6922		"DVB-S/S2"

static void tbs6922_lnb_power(struct dvb_frontend *fe, int onoff)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;
	int enpwr_pin = 17;

	/* lnb power, active high */
	saa716x_gpio_set_output(dev, enpwr_pin);
	if (onoff)
		saa716x_gpio_write(dev, enpwr_pin, 1);
	else
		saa716x_gpio_write(dev, enpwr_pin, 0);
}


static struct tas2101_config tbs6922_cfg = {
	.i2c_address   = 0x68,
	.id            = ID_TAS2100,
	.reset_demod   = NULL,
	.lnb_power     = tbs6922_lnb_power,
	.init          = {0x10, 0x32, 0x54, 0x76, 0xb8, 0x9a, 0x33},
};

static struct av201x_config tbs6922_av201x_cfg = {
	.i2c_address = 0x63,
	.id          = ID_AV2012,
	.xtal_freq   = 27000,		/* kHz */
};

static int saa716x_tbs6922_frontend_attach(
	struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attaching\n",
		dev->config->model_name, count);
	if (count > 0)
		goto err;

	saa716x_gpio_set_output(dev, 2);
	saa716x_gpio_write(dev, 2, 0);
	msleep(60);
	saa716x_gpio_write(dev, 2, 1);
	msleep(120);

	adapter->fe = dvb_attach(tas2101_attach, &tbs6922_cfg,
				&dev->i2c[SAA716x_I2C_BUS_A].i2c_adapter);
	if (adapter->fe == NULL)
		goto err;

	if (dvb_attach(av201x_attach, adapter->fe, &tbs6922_av201x_cfg,
			tas2101_get_i2c_adapter(adapter->fe, 2)) == NULL) {
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
		dev_dbg(&dev->pdev->dev,
			"%s frontend %d tuner attach failed\n",
			dev->config->model_name, count);
		goto err;
	}

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6922_config = {
	.model_name		= SAA716x_MODEL_TBS6922,
	.dev_type		= SAA716x_DEV_TBS6922,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= saa716x_tbs6922_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
	},
};

#define SAA716x_MODEL_TBS6983	"TBS 6983"
#define SAA716x_DEV_TBS6983	"DVB-S/S2"

static struct stv0910_cfg tbs6983_stv0910_config = {
	.name     = "STV090x TBS 6983",
	.adr      = 0x68,
	.parallel = 1,
	.rptlvl   = 4,
	.clk      = 30000000,

	.tuner_init		= NULL,
	.tuner_set_mode		= NULL,
	.tuner_set_frequency	= NULL,
	.tuner_set_bandwidth	= NULL,
	.tuner_set_params	= NULL,
};

static struct stv6120_config tbs6983_stv6120_0_config = {
	.addr			= 0x60,
	.refclk			= 30000000,
	.clk_div		= 2,
	.tuner			= 1,
};

static struct stv6120_config tbs6983_stv6120_1_config = {
	.addr			= 0x60,
	.refclk			= 30000000,
	.clk_div		= 2,
	.tuner			= 0,
};

static int saa716x_tbs6983_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;
	struct saa716x_dev *saa716x = adapter->saa716x;

	u8 adapter_gpio_0 = adapter->count ? 16 : 5;
	u8 adapter_gpio_1 = adapter->count ? 2  : 3;

	saa716x_gpio_set_output(saa716x, adapter_gpio_0);
	saa716x_gpio_set_output(saa716x, adapter_gpio_1);
	msleep(1);

	switch (voltage) {
	case SEC_VOLTAGE_13:
		dprintk(SAA716x_INFO, 1, "Adapter: %d, Polarization=[13V]", adapter->count);
		saa716x_gpio_write(saa716x, adapter_gpio_0, 0);
		saa716x_gpio_write(saa716x, adapter_gpio_1, 0);
		break;
	case SEC_VOLTAGE_18:
		dprintk(SAA716x_INFO, 1, "Adapter: %d, Polarization=[18V]", adapter->count);
		saa716x_gpio_write(saa716x, adapter_gpio_0, 1);
		saa716x_gpio_write(saa716x, adapter_gpio_1, 0);
		break;
	case SEC_VOLTAGE_OFF:
		dprintk(SAA716x_INFO, 1, "Adapter: %d, Polarization=[OFF]", adapter->count);
		saa716x_gpio_write(saa716x, adapter_gpio_0, 1);
		saa716x_gpio_write(saa716x, adapter_gpio_1, 1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int saa716x_tbs6983_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[1];
	struct stv6120_devctl *ctl;

	dprintk(SAA716x_NOTICE, 1, "TBS6983: %d", count);

	if (count == 0) {
		saa716x_gpio_set_output(saa716x, 17);
		msleep(1);
		saa716x_gpio_write(saa716x, 17, 0);
		msleep(50);
		saa716x_gpio_write(saa716x, 17, 1);
		msleep(100);
	}

	adapter->fe = dvb_attach(stv0910_attach,
				 &i2c->i2c_adapter,
				 &tbs6983_stv0910_config,
				 count);

	if (adapter->fe == NULL) {
		goto exit;
	}
	dprintk(SAA716x_NOTICE, 1, "found STV0910");


	adapter->fe->ops.i2c_gate_ctrl(adapter->fe, true);
	if (count == 0) {
		ctl = dvb_attach(stv6120_attach,
				 adapter->fe,
				 &tbs6983_stv6120_0_config,
				 &i2c->i2c_adapter);
	} else {
		ctl = dvb_attach(stv6120_attach,
				 adapter->fe,
				 &tbs6983_stv6120_1_config,
				 &i2c->i2c_adapter);
	}
	adapter->fe->ops.i2c_gate_ctrl(adapter->fe, false);

	if (ctl == NULL) {
		goto exit;
	}
	dprintk(SAA716x_NOTICE, 1, "found STV6120");

	tbs6983_stv0910_config.tuner_set_mode      = ctl->tuner_set_mode;
	tbs6983_stv0910_config.tuner_set_params    = ctl->tuner_set_params;
	tbs6983_stv0910_config.tuner_set_frequency = ctl->tuner_set_frequency;
	tbs6983_stv0910_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;

//	dprintk(SAA716x_NOTICE, 1, "init start");
//	if (adapter->fe->ops.init)
//		adapter->fe->ops.init(adapter->fe);
//	dprintk(SAA716x_NOTICE, 1, "init complete");

	adapter->fe->ops.set_voltage = saa716x_tbs6983_set_voltage;

	dprintk(SAA716x_NOTICE, 1, "Done!");
	return 0;
exit:
	printk(KERN_ERR "%s: frontend initialization failed\n",
					adapter->saa716x->config->model_name);
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6983_config = {
	.model_name		= SAA716x_MODEL_TBS6983,
	.dev_type		= SAA716x_DEV_TBS6983,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6983_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.adap_config		= {
		{ // adapter 0
			.ts_port = 3,
			.worker  = demux_worker
		},
		{ // adapter 1
			.ts_port = 1,
			.worker  = demux_worker
		},
	}
};

#define SAA716x_MODEL_TBS6925	"TBS 6925"
#define SAA716x_DEV_TBS6925		"DVB-S/S2"

static struct stv090x_config tbs6925_stv090x_config = {
	.device			= STV0900,
	.demod_mode		= STV090x_SINGLE,
	.clk_mode		= STV090x_CLK_EXT,

	.xtal			= 27000000,
	.address		= 0x68,

	.ts1_mode		= STV090x_TSMODE_PARALLEL_PUNCTURED,
	.ts2_mode		= STV090x_TSMODE_PARALLEL_PUNCTURED,

	.repeater_level		= STV090x_RPTLEVEL_16,
	.adc1_range		= STV090x_ADC_2Vpp,
	.tuner_bbgain		= 8,

	.tuner_get_frequency	= stb6100_get_frequency,
	.tuner_set_frequency	= stb6100_set_frequency,
	.tuner_set_bandwidth	= stb6100_set_bandwidth,
	.tuner_get_bandwidth	= stb6100_get_bandwidth,
	.name					= "STV090x TBS 6925",
	.tun1_iqswap			= 1,
	.tun2_iqswap			= 1,
};

static struct stb6100_config tbs6925_stb6100_config = {
	.tuner_address	= 0x60,
	.refclock	= 27000000
};

static int tbs6925_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;
	struct saa716x_dev *saa716x = adapter->saa716x;

	// TODO
	// GPIO 17 looks like it controls power on/off

	saa716x_gpio_set_output(saa716x, 16);
	msleep(1);
	switch (voltage) {
	case SEC_VOLTAGE_13:
			dprintk(SAA716x_ERROR, 1, "Polarization=[13V]");
			saa716x_gpio_write(saa716x, 16, 0);
			break;
	case SEC_VOLTAGE_18:
			dprintk(SAA716x_ERROR, 1, "Polarization=[18V]");
			saa716x_gpio_write(saa716x, 16, 1);
			break;
	case SEC_VOLTAGE_OFF:
			dprintk(SAA716x_ERROR, 1, "Frontend (dummy) POWERDOWN");
			break;
	default:
			dprintk(SAA716x_ERROR, 1, "Invalid = (%d)", (u32 ) voltage);
			return -EINVAL;
	}
	msleep(100);

	return 0;
}

static int tbs6925_frontend_attach(struct saa716x_adapter *adapter,
						   int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[SAA716x_I2C_BUS_A];
	struct stv6110x_devctl *ctl;

	printk("tbs6925_frontend_attach() Starting...\n");

	if (count < saa716x->config->adapters) {
		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init",
			count);
		dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count,
			saa716x->pdev->subsystem_device);

		/* Reset the demodulator */
		saa716x_gpio_set_output(saa716x, 2);
		msleep(1);
		saa716x_gpio_write(saa716x, 2, 0);
		msleep(50);
		saa716x_gpio_write(saa716x, 2, 1);
		msleep(100);

		adapter->fe = dvb_attach(stv090x_attach, &tbs6925_stv090x_config, &i2c->i2c_adapter, STV090x_DEMODULATOR_0);
		if (adapter->fe) {
			dprintk(SAA716x_NOTICE, 1, "found STV0900 @0x%02x", tbs6925_stv090x_config.address);
		} else {
			goto exit;
		}

		ctl = dvb_attach(stb6100_attach, adapter->fe, &tbs6925_stb6100_config, &i2c->i2c_adapter);
		if (ctl) {
			dprintk(SAA716x_NOTICE, 1, "found STB6100");

			/* call the init function once to initialize
			   tuner's clock output divider and demod's
			   master clock */
			if (adapter->fe->ops.init)
				adapter->fe->ops.init(adapter->fe);
		} else {
			goto exit;
		}

		adapter->fe->ops.set_voltage	= tbs6925_set_voltage;
		adapter->fe->ops.set_frame_ops	= saa716x_set_frame_ops;

		dprintk(SAA716x_ERROR, 1, "Done!");
		return 0;
	}
exit:
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6925_config = {
	.model_name		= SAA716x_MODEL_TBS6925,
	.dev_type		= SAA716x_DEV_TBS6925,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= tbs6925_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_port = 3, /* using FGPI 1 */
			.worker = demux_worker
		}
	}
};

#define SAA716x_MODEL_TBS6982		"TurboSight TBS 6982"
#define SAA716x_DEV_TBS6982		"DVB-S/S2"

static void tbs6982_reset_fe(struct dvb_frontend *fe, int reset_pin)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* reset frontend, active low */
	saa716x_gpio_set_output(dev, reset_pin);
	saa716x_gpio_write(dev, reset_pin, 0);
	msleep(60);
	saa716x_gpio_write(dev, reset_pin, 1);
	msleep(120);
}

static void tbs6982_reset_fe0(struct dvb_frontend *fe)
{
	tbs6982_reset_fe(fe, 2);
}

static void tbs6982_reset_fe1(struct dvb_frontend *fe)
{
	tbs6982_reset_fe(fe, 17);
}

static void tbs6982_lnb_power(struct dvb_frontend *fe,
	int enpwr_pin, int onoff)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* lnb power, active low */
	saa716x_gpio_set_output(dev, enpwr_pin);
	if (onoff)
		saa716x_gpio_write(dev, enpwr_pin, 0);
	else
		saa716x_gpio_write(dev, enpwr_pin, 1);
}

static void tbs6982_lnb0_power(struct dvb_frontend *fe, int onoff)
{
	tbs6982_lnb_power(fe, 5, onoff);
}

static void tbs6982_lnb1_power(struct dvb_frontend *fe, int onoff)
{
	tbs6982_lnb_power(fe, 3, onoff);
}

static struct tas2101_config tbs6982_cfg[] = {
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6982_reset_fe0,
		.lnb_power     = tbs6982_lnb0_power,
		.init          = {0x10, 0x32, 0x54, 0x76, 0xb8, 0x9a, 0x33},
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6982_reset_fe1,
		.lnb_power     = tbs6982_lnb1_power,
		.init          = {0x8a, 0x6b, 0x13, 0x70, 0x45, 0x92, 0x33},
	}
};

static struct av201x_config tbs6982_av201x_cfg = {
	.i2c_address = 0x63,
	.id          = ID_AV2012,
	.xtal_freq   = 27000,		/* kHz */
};

static int saa716x_tbs6982_frontend_attach(
	struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attaching\n",
		dev->config->model_name, count);
	if (count > 1)
		goto err;

	saa716x_gpio_set_output(dev, 16);
	saa716x_gpio_write(dev, 16, 0);
	msleep(60);
	saa716x_gpio_write(dev, 16, 1);
	msleep(120);

	adapter->fe = dvb_attach(tas2101_attach, &tbs6982_cfg[count],
				&dev->i2c[1 - count].i2c_adapter);
	if (adapter->fe == NULL)
		goto err;

	if (dvb_attach(av201x_attach, adapter->fe, &tbs6982_av201x_cfg,
			tas2101_get_i2c_adapter(adapter->fe, 2)) == NULL) {
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
		dev_dbg(&dev->pdev->dev,
			"%s frontend %d tuner attach failed\n",
			dev->config->model_name, count);
		goto err;
	}

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6982_config = {
	.model_name		= SAA716x_MODEL_TBS6982,
	.dev_type		= SAA716x_DEV_TBS6982,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6982_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6982SE		"TurboSight TBS 6982SE"
#define SAA716x_DEV_TBS6982SE		"DVB-S/S2"

static void tbs6982se_reset_fe(struct dvb_frontend *fe, int reset_pin)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* reset frontend, active low */
	saa716x_gpio_set_output(dev, reset_pin);
	saa716x_gpio_write(dev, reset_pin, 0);
	msleep(60);
	saa716x_gpio_write(dev, reset_pin, 1);
	msleep(120);
}

static void tbs6982se_reset_fe0(struct dvb_frontend *fe)
{
	tbs6982se_reset_fe(fe, 2);
}

static void tbs6982se_reset_fe1(struct dvb_frontend *fe)
{
	tbs6982se_reset_fe(fe, 17);
}

static void tbs6982se_lnb_power(struct dvb_frontend *fe,
	int enpwr_pin, int onoff)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* lnb power, active low */
	saa716x_gpio_set_output(dev, enpwr_pin);
	if (onoff)
		saa716x_gpio_write(dev, enpwr_pin, 0);
	else
		saa716x_gpio_write(dev, enpwr_pin, 1);
}

static void tbs6982se_lnb0_power(struct dvb_frontend *fe, int onoff)
{
	tbs6982se_lnb_power(fe, 3, onoff);
}

static void tbs6982se_lnb1_power(struct dvb_frontend *fe, int onoff)
{
	tbs6982se_lnb_power(fe, 16, onoff);
}

static struct tas2101_config tbs6982se_cfg[] = {
	{
		.i2c_address   = 0x60,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6982se_reset_fe0,
		.lnb_power     = tbs6982se_lnb0_power,
		.init          = {0x10, 0x32, 0x54, 0x76, 0xb8, 0x9a, 0x33},
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6982se_reset_fe1,
		.lnb_power     = tbs6982se_lnb1_power,
		.init          = {0x8a, 0x6b, 0x13, 0x70, 0x45, 0x92, 0x33},
	}
};

static struct av201x_config tbs6982se_av201x_cfg = {
	.i2c_address = 0x63,
	.id          = ID_AV2012,
	.xtal_freq   = 27000,		/* kHz */
};

static int saa716x_tbs6982se_frontend_attach(
	struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;

	dev_dbg(&dev->pdev->dev, "%s frontend %d attaching\n",
		dev->config->model_name, count);
	if (count > 1)
		goto err;

	adapter->fe = dvb_attach(tas2101_attach, &tbs6982se_cfg[count],
				&dev->i2c[count].i2c_adapter);
	if (adapter->fe == NULL)
		goto err;

	if (dvb_attach(av201x_attach, adapter->fe, &tbs6982se_av201x_cfg,
			tas2101_get_i2c_adapter(adapter->fe, 2)) == NULL) {
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
		dev_dbg(&dev->pdev->dev,
			"%s frontend %d tuner attach failed\n",
			dev->config->model_name, count);
		goto err;
	}

	dev_dbg(&dev->pdev->dev, "%s frontend %d attached\n",
		dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6982se_config = {
	.model_name		= SAA716x_MODEL_TBS6982SE,
	.dev_type		= SAA716x_DEV_TBS6982SE,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 2,
	.frontend_attach	= saa716x_tbs6982se_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 3, /* using FGPI 3 */
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6984		"TurboSight TBS 6984"
#define SAA716x_DEV_TBS6984		"DVB-S/S2"

static void saa716x_tbs6984_init(struct saa716x_dev *saa716x)
{
	int i;
	const u8 buf[] = {
		0xe0, 0x06, 0x66, 0x33, 0x65,
		0x01, 0x17, 0x06, 0xde};

#define TBS_CK 7
#define TBS_CS 8
#define TBS_DT 11

	/* send init bitstream through a bitbanged spi */
	/* set pins as output */
	saa716x_gpio_set_output(saa716x, TBS_CK);
	saa716x_gpio_set_output(saa716x, TBS_CS);
	saa716x_gpio_set_output(saa716x, TBS_DT);

	/* set all pins high */
	saa716x_gpio_write(saa716x, TBS_CK, 1);
	saa716x_gpio_write(saa716x, TBS_CS, 1);
	saa716x_gpio_write(saa716x, TBS_DT, 1);
	msleep(20);

	/* CS low */
	saa716x_gpio_write(saa716x, TBS_CS, 0);
	msleep(20);
	/* send bitstream */
	for (i = 0; i < 9 * 8; i++) {
		/* clock low */
		saa716x_gpio_write(saa716x, TBS_CK, 0);
		msleep(20);
		/* set data pin */
		saa716x_gpio_write(saa716x, TBS_DT,
			((buf[i >> 3] >> (7 - (i & 7))) & 1));
		/* clock high */
		saa716x_gpio_write(saa716x, TBS_CK, 1);
		msleep(20);
	}
	/* raise cs, clk and data */
	saa716x_gpio_write(saa716x, TBS_CS, 1);
	saa716x_gpio_write(saa716x, TBS_CK, 1);
	saa716x_gpio_write(saa716x, TBS_DT, 1);

	/* power up LNB supply and control chips */
	saa716x_gpio_set_output(saa716x, 19);	/* a0 */
	saa716x_gpio_set_output(saa716x, 2);	/* a1 */
	saa716x_gpio_set_output(saa716x, 5);	/* a2 */
	saa716x_gpio_set_output(saa716x, 3);	/* a3 */

	/* power off */
	saa716x_gpio_write(saa716x, 19, 1); /* a0 */
	saa716x_gpio_write(saa716x, 2, 1); /* a1 */
	saa716x_gpio_write(saa716x, 5, 1); /* a2 */
	saa716x_gpio_write(saa716x, 3, 1); /* a3 */
}


static void tbs6984_lnb_pwr(struct dvb_frontend *fe, int pin, int onoff)
{
	struct i2c_adapter *i2c_adapter = cx24117_get_i2c_adapter(fe);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* lnb power, active low */
	if (onoff)
		saa716x_gpio_write(dev, pin , 0);
	else
		saa716x_gpio_write(dev, pin, 1);
}

void tbs6984_lnb_pwr0(struct dvb_frontend *fe, int demod, int onoff)
{
	tbs6984_lnb_pwr(fe, (demod == 0) ? 19 : 2, onoff);
}

void tbs6984_lnb_pwr1(struct dvb_frontend *fe, int demod, int onoff)
{
	tbs6984_lnb_pwr(fe, (demod == 0) ? 5 : 3, onoff);
}

static struct cx24117_config tbs6984_cx24117_cfg[] = {
	{
		.demod_address = 0x55,
		.lnb_power = tbs6984_lnb_pwr0,
	},
	{
		.demod_address = 0x05,
		.lnb_power = tbs6984_lnb_pwr1,
	},
};

static struct isl6422_config tbs6984_isl6422_cfg[] = {
	{
		.current_max		= SEC_CURRENT_570m,
		.curlim			= SEC_CURRENT_LIM_ON,
		.mod_extern		= 1,
		.addr			= 0x08,
		.id			= 0,
	},
	{
		.current_max		= SEC_CURRENT_570m,
		.curlim			= SEC_CURRENT_LIM_ON,
		.mod_extern		= 1,
		.addr			= 0x08,
		.id			= 1,
	}

};

static int saa716x_tbs6984_frontend_attach(
	struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;
	struct saa716x_i2c *i2c = &dev->i2c[1 - (count >> 1)];

	dev_dbg(&dev->pdev->dev, "%s frontend %d attaching\n",
		dev->config->model_name, count);

	if (count > 3)
		goto err;

	if (count == 0)
		saa716x_tbs6984_init(dev);

	adapter->fe = dvb_attach(cx24117_attach, &tbs6984_cx24117_cfg[count >> 1],
			&i2c->i2c_adapter);
	if (adapter->fe == NULL)
		goto err;

	if (dvb_attach(isl6422_attach, adapter->fe, &i2c->i2c_adapter,
			&tbs6984_isl6422_cfg[count & 0x01]) == NULL)
		dev_info(&dev->pdev->dev,
			"%s frontend %d doesn't seem to have a isl6422b on the i2c bus.\n",
			dev->config->model_name, count);

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6984_config = {
	.model_name		= SAA716x_MODEL_TBS6984,
	.dev_type		= SAA716x_DEV_TBS6984,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 4,
	.frontend_attach	= saa716x_tbs6984_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 2,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 3,
			.worker = demux_worker
		},
		{
			/* adapter 2 */
			.ts_port = 0,
			.worker = demux_worker
		},
		{
			/* adapter 3 */
			.ts_port = 1,
			.worker = demux_worker
		},
	},
};


#define SAA716x_MODEL_TBS6985 "TurboSight TBS 6985"
#define SAA716x_DEV_TBS6985   "DVB-S/S2"

static void tbs6985_reset_fe(struct dvb_frontend *fe, int reset_pin)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* reset frontend, active low */
	saa716x_gpio_set_output(dev, reset_pin);
	saa716x_gpio_write(dev, reset_pin, 0);
	msleep(60);
	saa716x_gpio_write(dev, reset_pin, 1);
	msleep(120);
}

static void tbs6985_reset_fe0(struct dvb_frontend *fe)
{
	tbs6985_reset_fe(fe, 5);
}

static void tbs6985_reset_fe1(struct dvb_frontend *fe)
{
	tbs6985_reset_fe(fe, 2);
}

static void tbs6985_reset_fe2(struct dvb_frontend *fe)
{
	tbs6985_reset_fe(fe, 13);
}

static void tbs6985_reset_fe3(struct dvb_frontend *fe)
{
	tbs6985_reset_fe(fe, 3);
}

static void tbs6985_lnb_power(struct dvb_frontend *fe, int enpwr_pin, int onoff)
{
	struct i2c_adapter *i2c_adapter = tas2101_get_i2c_adapter(fe, 0);
	struct saa716x_i2c *i2c = i2c_get_adapdata(i2c_adapter);
	struct saa716x_dev *dev = i2c->saa716x;

	/* lnb power, active low */
	saa716x_gpio_set_output(dev, enpwr_pin);
	if (onoff)
		saa716x_gpio_write(dev, enpwr_pin, 0);
	else
		saa716x_gpio_write(dev, enpwr_pin, 1);
}

static void tbs6985_lnb0_power(struct dvb_frontend *fe, int onoff)
{
	tbs6985_lnb_power(fe, 27, onoff);
}

static void tbs6985_lnb1_power(struct dvb_frontend *fe, int onoff)
{
	tbs6985_lnb_power(fe, 22, onoff);
}

static void tbs6985_lnb2_power(struct dvb_frontend *fe, int onoff)
{
	tbs6985_lnb_power(fe, 19, onoff);
}

static void tbs6985_lnb3_power(struct dvb_frontend *fe, int onoff)
{
	tbs6985_lnb_power(fe, 15, onoff);
}

#undef TBS6985_TSMODE0
static struct tas2101_config tbs6985_cfg[] = {
	{
		.i2c_address   = 0x60,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6985_reset_fe0,
		.lnb_power     = tbs6985_lnb0_power,
#ifdef TBS6985_TSMODE0
		.init          = {0x01, 0x32, 0x65, 0x74, 0xab, 0x98, 0x33},
#else
		.init          = {0x0b, 0x8a, 0x65, 0x74, 0xab, 0x98, 0xb1},
#endif
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6985_reset_fe1,
		.lnb_power     = tbs6985_lnb1_power,
#ifdef TBS6985_TSMODE0
		.init          = {0x10, 0x32, 0x54, 0xb7, 0x86, 0x9a, 0x33},
#else
		.init          = {0x0a, 0x8b, 0x54, 0xb7, 0x86, 0x9a, 0xb1},
#endif
	},
	{
		.i2c_address   = 0x60,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6985_reset_fe2,
		.lnb_power     = tbs6985_lnb2_power,
#ifdef TBS6985_TSMODE0
		.init          = {0x25, 0x36, 0x40, 0xb1, 0x87, 0x9a, 0x33},
#else
		.init          = {0xba, 0x80, 0x40, 0xb1, 0x87, 0x9a, 0xb1},
#endif
	},
	{
		.i2c_address   = 0x68,
		.id            = ID_TAS2101,
		.reset_demod   = tbs6985_reset_fe3,
		.lnb_power     = tbs6985_lnb3_power,
#ifdef TBS6985_TSMODE0
		.init          = {0x80, 0xba, 0x21, 0x53, 0x74, 0x96, 0x33},
#else
		.init          = {0xba, 0x80, 0x21, 0x53, 0x74, 0x96, 0xb1},
#endif
	}
};

static struct av201x_config tbs6985_av201x_cfg = {
	.i2c_address = 0x63,
	.id          = ID_AV2012,
	.xtal_freq   = 27000,		/* kHz */
};

static int saa716x_tbs6985_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *dev = adapter->saa716x;

	if (count > 3)
		goto err;

	adapter->fe = dvb_attach(tas2101_attach, &tbs6985_cfg[count],
				&dev->i2c[1 - (count >> 1)].i2c_adapter);
	if (adapter->fe == NULL)
		goto err;

	if (dvb_attach(av201x_attach, adapter->fe, &tbs6985_av201x_cfg,
			tas2101_get_i2c_adapter(adapter->fe, 2)) == NULL) {
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
		dev_dbg(&dev->pdev->dev,
			"%s frontend %d tuner attach failed\n",
			dev->config->model_name, count);
		goto err;
	}

	return 0;
err:
	dev_err(&dev->pdev->dev, "%s frontend %d attach failed\n",
		dev->config->model_name, count);
	return -ENODEV;
}

static struct saa716x_config saa716x_tbs6985_config = {
	.model_name		= SAA716x_MODEL_TBS6985,
	.dev_type		= SAA716x_DEV_TBS6985,
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 4,
	.frontend_attach	= saa716x_tbs6985_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_400,
	.i2c_mode		= SAA716x_I2C_MODE_POLLING,
	.adap_config		= {
		{
			/* adapter 0 */
			.ts_port = 2,
			.worker = demux_worker
		},
		{
			/* adapter 1 */
			.ts_port = 3,
			.worker = demux_worker
		},
		{
			/* adapter 2 */
			.ts_port = 0,
			.worker = demux_worker
		},
		{
			/* adapter 3 */
			.ts_port = 1,
			.worker = demux_worker
		}
	},
};

static struct ds3103_config s472_ds3103_config = {
	.demod_address = 0x68,
	.ci_mode = 1,
};

static int saa716x_s472_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[1];

	if (count != 0)
		return 0;

	dprintk(SAA716x_ERROR, 1, "Probing for DS3103 (DVB-S/S2)");
	adapter->fe = dvb_attach(ds3103_attach, &s472_ds3103_config,
				 &i2c->i2c_adapter);

	if (adapter->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
		return -ENODEV;
	}

	dprintk(SAA716x_ERROR, 1, "found DS3103 DVB-S/S2 frontend @0x%02x",
		s472_ds3103_config.demod_address);
	if (NULL == dvb_attach(ts2022_attach, adapter->fe, 0x60, &i2c->i2c_adapter))
		dprintk(SAA716x_ERROR, 1, "ts2022 attach failed");
	else
		dprintk(SAA716x_ERROR, 1, "ts2022 attached!");

	dprintk(SAA716x_ERROR, 1, "Done!");
	return 0;

}

static struct saa716x_config tevii_s472_config = {
	.model_name		= "TeVii S472 DVB-S2",
	.dev_type		= "DVB-S/S2",
	.boot_mode		= SAA716x_EXT_BOOT,
	.adapters		= 1,
	.frontend_attach	= saa716x_s472_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_port = 1, /* using FGPI 1 */
			.worker = demux_worker
		}
	}
};

static struct pci_device_id saa716x_budget_pci_table[] = {
	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_1028, SAA7160, &saa716x_vp1028_config), /* VP-1028 */
	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_3071, SAA7160, &saa716x_vp3071_config), /* VP-3071 */
	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6002, SAA7160, &saa716x_vp6002_config), /* VP-6002 */
	MAKE_ENTRY(KNC_One, KNC_Dual_S2, SAA7160, &saa716x_knc1_duals2_config),
	MAKE_ENTRY(TECHNISAT, SKYSTAR2_EXPRESS_HD, SAA7160, &skystar2_express_hd_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6284, TBS6284,   SAA7160, &saa716x_tbs6284_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6280, TBS6280,   SAA7160, &saa716x_tbs6280_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6281, TBS6281,   SAA7160, &saa716x_tbs6281_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6285, TBS6285,   SAA7160, &saa716x_tbs6285_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6220, TBS6220,   SAA7160, &saa716x_tbs6220_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6922, TBS6922,   SAA7160, &saa716x_tbs6922_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6925, TBS6925,   SAA7160, &saa716x_tbs6925_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6982, TBS6982,   SAA7160, &saa716x_tbs6982_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6982, TBS6982SE, SAA7160, &saa716x_tbs6982se_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6984, TBS6984,   SAA7160, &saa716x_tbs6984_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6985, TBS6985,   SAA7160, &saa716x_tbs6985_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6985, TBS6985+1, SAA7160, &saa716x_tbs6985_config),
	MAKE_ENTRY(TECHNOTREND,        TT4100,    SAA7160, &saa716x_tbs6922_config),
	MAKE_ENTRY(TEVII, TEVII_S472, SAA7160, &tevii_s472_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6983, TBS6983, SAA7160, &saa716x_tbs6983_config),
	MAKE_ENTRY(TURBOSIGHT_TBS6983, TBS6983+1, SAA7160, &saa716x_tbs6983_config),
	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_budget_pci_table);

static struct pci_driver saa716x_budget_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_budget_pci_table,
	.probe			= saa716x_budget_pci_probe,
	.remove			= saa716x_budget_pci_remove,
};

module_pci_driver(saa716x_budget_pci_driver);

MODULE_DESCRIPTION("SAA716x Budget driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
