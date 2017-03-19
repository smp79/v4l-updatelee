#ifndef _STV0910_H_
#define _STV0910_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include "stv6110x.h"

struct stv0910_cfg {
	u32 clk;
	u8  adr;
	u8  parallel;
	u8  rptlvl;

	char name[128];

	int (*tuner_init) (struct dvb_frontend *fe);
	int (*tuner_set_mode) (struct dvb_frontend *fe, enum tuner_mode mode);
	int (*tuner_set_params) (struct dvb_frontend *fe);
	int (*tuner_set_frequency) (struct dvb_frontend *fe, u32 frequency);
	int (*tuner_set_bandwidth) (struct dvb_frontend *fe, u32 bandwidth);
};

enum stv0910_algo {
	STV0910_BLIND_SEARCH,
	STV0910_COLD_SEARCH,
	STV0910_WARM_SEARCH,
	STV0910_NOTUNE
};

#define MAKEWORD16(__a, __b)			(((__a) << 8) | (__b))

#if defined(CONFIG_DVB_STV0910) || \
	(defined(CONFIG_DVB_STV0910_MODULE) && defined(MODULE))

extern struct dvb_frontend *stv0910_attach(struct i2c_adapter *i2c,
					   struct stv0910_cfg *cfg, int nr);
#else

static inline struct dvb_frontend *stv0910_attach(struct i2c_adapter *i2c,
						  struct stv0910_cfg *cfg,
						  int nr)
{
	pr_warn("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif

#endif
