/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2022. All rights reserved.
 * 2022-12-23 File created.
 */

#ifndef __Q6FSM_V3_H__
#define __Q6FSM_V3_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6audio-v2.h>

#define fsm_info(fmt, args...) pr_info("%s: " fmt "\n", __func__, ##args)
#define fsm_dbg(fmt, args...) pr_debug("%s: " fmt "\n", __func__, ##args)
#define fsm_err(fmt, args...) pr_err("%s: " fmt "\n", __func__, ##args)

#define Q6FSM_VERSION     "v1.1.1"

// TODO update
#define Q6FSM_AFE_RX_PORT (AFE_PORT_ID_RX_CODEC_DMA_RX_0)
#define Q6FSM_AFE_TX_PORT (AFE_PORT_ID_TX_CODEC_DMA_TX_3)

#define Q6FSM_CHUNK_SIZE  (256)

// FSADSP MODULE ID
#define AFE_MODULE_ID_FSADSP_TX (0x10001110)
#define AFE_MODULE_ID_FSADSP_RX (0x10001111)
// FSADSP PARAM ID
#define CAPI_V2_PARAM_FSADSP_TX_ENABLE     0x11111601
#define CAPI_V2_PARAM_FSADSP_RX_ENABLE     0x11111611
#define CAPI_V2_PARAM_FSADSP_MODULE_ENABLE 0x10001FA1
#define CAPI_V2_PARAM_FSADSP_ROTATION      0x10001FB1
#define CAPI_V2_PARAM_FSADSP_BSG_VBAT      0x10001FB2
#define CAPI_V2_PARAM_FSADSP_BSGV2_PARAM   0x10001FB5 //set
#define CAPI_V2_PARAM_FSADSP_BSGV2_CONFIG  0x10001FB6 //get
#define CAPI_V2_PARAM_FSADSP_ROTATION      0x10001FB1
#define CAPI_V2_PARAM_FSADSP_FADE          0x10001FB7

#define Q6FSM_RETRY_TIME (3)
#define Q6FSM_SLEEP_TIME (10)

struct bsg_config_v2 {
	int bsg_mode;
	int bsg_enable;
	int bsg_interval;
	int tc_mode;
	int tc_mode_enable;
	int tc_interval;
};

struct bsg_param_v2 {
	int bsg_mode;
	int bsg_val;
	int tc_mode;
	int tc_val;
};

struct q6fsm_afe {
	struct workqueue_struct *vbat_wq;
	struct delayed_work vbat_work;
	struct power_supply *psy;
	struct mutex cmd_lock;
	atomic_t bsg_switch;
	unsigned long delay_period;
	bool monitor_en;
	int bsg_version;
	struct bsg_config_v2 config;
	int angle;
};

typedef enum _fs__chann_t
{
	FS_CH_LEFT = 1 << 0,
	FS_CH_RIGHT = 1 << 1,
	FS_CH_MONO = (FS_CH_LEFT|FS_CH_RIGHT),
}fs_chann_t;

typedef struct _fsm_fade_info_t
{
	int fade_type; //0x10 mute, 0x11 fade in,0x12 fade out
	int fade_time; // fade length time, unit is ms
	int start_db;
	fs_chann_t ch; // left or right channel
}fsm_fade_info_t;

typedef struct _fsm_rotation_info_t
{
	int angle; // 0,90,180,270
	int8_t ch_sequence[8];
}fsm_rotation_info_t;


static struct q6fsm_afe *g_fsm_afe;
static uint32_t *q6fsm_get_buffer;

static int q6afe_set_params(u16 port_id, int index,
			    struct mem_mapping_hdr *mem_hdr,
			    u8 *packed_param_data, u32 packed_data_size);
static int q6afe_get_params(u16 port_id,
			    struct mem_mapping_hdr *mem_hdr,
			    struct param_hdr_v3 *param_hdr);

static int q6fsm_afe_callback(void *payload, int size)
{
	uint32_t *payload32 = payload;

	if (payload32[0] != 0) {
		pr_err("invalid status: %d", payload32[0]);
		return -EINVAL;
	}

	fsm_dbg("payload size: %d", size);
	switch (payload32[1]) {
	case AFE_MODULE_ID_FSADSP_TX:
	case AFE_MODULE_ID_FSADSP_RX:
		memcpy(q6fsm_get_buffer, payload32 + 1, size - 4);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool q6fsm_check_dsp_ready(void)
{
	int topo_id;

	topo_id = afe_get_topology(Q6FSM_AFE_RX_PORT);
	fsm_dbg("topo_id: 0x%x", topo_id);

	if (topo_id <= 0)
		return false;
	else
		return true;
}

static int q6fsm_afe_set_params(struct param_hdr_v3 *param_hdr,
		uint8_t *params, uint32_t param_size)
{
	u8 *packed_param_data;
	int packed_data_size;
	int port_id, index;
	int retry = 0;
	int ret;

	if (param_hdr == NULL || params == NULL) {
		fsm_err("Bad parameters");
		return -EINVAL;
	}

	if (param_size >= Q6FSM_CHUNK_SIZE) {
		fsm_err("Invalid param_size: %d\n", param_size);
		return -EINVAL;
	}

	if (param_hdr->module_id == AFE_MODULE_ID_FSADSP_RX)
		port_id = Q6FSM_AFE_RX_PORT;
	else
		port_id = Q6FSM_AFE_TX_PORT;

	index = afe_get_port_index(port_id);
	if (index < 0) {
		fsm_err("invalid port: %x", port_id);
		return -EINVAL;
	}

	mutex_lock(&g_fsm_afe->cmd_lock);
	packed_data_size = sizeof(union param_hdrs) + param_size;
	packed_param_data = kzalloc(packed_data_size, GFP_KERNEL);
	if (packed_param_data == NULL) {
		fsm_err("alloc memory failed!");
		mutex_unlock(&g_fsm_afe->cmd_lock);
		return -ENOMEM;
	}

	do {
		ret = q6common_pack_pp_params(packed_param_data, param_hdr,
			params, &packed_data_size);
		if (ret) {
			fsm_err("Failed to pack data: %d", ret);
			break;
		}
		while (retry++ < Q6FSM_RETRY_TIME) {
			if (q6fsm_check_dsp_ready()) {
				ret = q6afe_set_params(port_id, index, NULL,
					packed_param_data, packed_data_size);
				break;
			}
			msleep(Q6FSM_SLEEP_TIME);
			ret = -ETIMEDOUT;
		}
	} while (0);
	if (ret)
		fsm_err("Failed to set params: %d", ret);

	mutex_unlock(&g_fsm_afe->cmd_lock);
	kfree(packed_param_data);
	packed_param_data = NULL;

	return ret;
}

static int q6fsm_afe_get_params(struct param_hdr_v3 *param_hdr,
		uint8_t *params, uint32_t length)
{
	int port_id;
	int retry = 0;
	int ret;

	if (param_hdr == NULL || params == NULL) {
		fsm_err("Bad parameters");
		return -EINVAL;
	}

	mutex_lock(&g_fsm_afe->cmd_lock);
	q6fsm_get_buffer = kzalloc(length + 16, GFP_KERNEL);
	if (q6fsm_get_buffer == NULL) {
		fsm_err("Memory allocation failed!");
		mutex_unlock(&g_fsm_afe->cmd_lock);
		return -ENOMEM;
	}

	if (param_hdr->module_id == AFE_MODULE_ID_FSADSP_RX)
		port_id = Q6FSM_AFE_RX_PORT;
	else
		port_id = Q6FSM_AFE_TX_PORT;

	do {
		while (retry++ < Q6FSM_RETRY_TIME) {
			if (q6fsm_check_dsp_ready()) {
				ret = q6afe_get_params(port_id, NULL, param_hdr);
				break;
			}
			msleep(Q6FSM_SLEEP_TIME);
			ret = -ETIMEDOUT;
		}
		if (!ret)
			memcpy(params, &q6fsm_get_buffer[4], length);
		else
			fsm_err("Get params failed: %d", ret);
	} while (0);

	kfree(q6fsm_get_buffer);
	q6fsm_get_buffer = NULL;
	mutex_unlock(&g_fsm_afe->cmd_lock);

	return ret;
}

bool q6fsm_get_rx_status(void)
{
	struct param_hdr_v3 param_hdr;
	int enable;
	int ret;

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_RX_ENABLE;
	param_hdr.param_size = sizeof(enable);

	ret = q6fsm_afe_get_params(&param_hdr,
		(uint8_t *)&enable, sizeof(enable));
	if (ret) {
		fsm_err("Get params failed: %d", ret);
		return false;
	}

	return !!enable;
}
EXPORT_SYMBOL(q6fsm_get_rx_status);

int q6fsm_set_rx_enable(int enable)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;
	struct param_hdr_v3 param_hdr;
	uint32_t param;
	int ret;

	if (q6fsm == NULL)
		return -EINVAL;

	if (!q6fsm_get_rx_status()) {
		fsm_info("RX module isn't ready");
		return -EINVAL;
	}

	param = (uint32_t)enable;
	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_RX_ENABLE;
	param_hdr.param_size = sizeof(int);
	ret = q6fsm_afe_set_params(&param_hdr,
		(uint8_t *)&param, sizeof(param));
	if (ret) {
		fsm_err("Set params fail:%d", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(q6fsm_set_rx_enable);

int q6fsm_set_rotation(int angle)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;
	struct param_hdr_v3 param_hdr;
	fsm_rotation_info_t param = {0};
	int ret;

	if (q6fsm == NULL)
		return -EINVAL;

	if (!q6fsm_get_rx_status()) {
		fsm_info("RX module isn't ready");
		return -EINVAL;
	}

	if(q6fsm->angle == angle){
		fsm_info("no need to rotation angle");
		return 0;
	}

	if(angle == 0){
		param.ch_sequence[0] = 0;
		param.ch_sequence[1] = 1;
	}else if(angle == 90){
		param.ch_sequence[0] = 1;
		param.ch_sequence[1] = 0;
	}else{
		fsm_err("not support angle:%d",angle);
		return  -EINVAL;
	}
	param.angle = angle;
	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_ROTATION;
	param_hdr.param_size = sizeof(int);
	ret = q6fsm_afe_set_params(&param_hdr,
		(uint8_t *)&param, sizeof(param));
	if (ret) {
		fsm_err("Set params fail:%d", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(q6fsm_set_rotation);

int q6fsm_get_rotation(int *angle)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;

	*angle = q6fsm->angle;

	return 0;
}
EXPORT_SYMBOL(q6fsm_get_rotation);


#define PSY_DESC psy->desc

static int q6fsm_get_batv(struct q6fsm_afe *q6fsm, uint32_t *batv)
{
	union power_supply_propval prop = { 0 };
	struct power_supply *psy;

	if (q6fsm == NULL  || batv == NULL) {
		fsm_err("Bad paramters");
		return -EINVAL;
	}

	q6fsm->psy = power_supply_get_by_name("battery");
	if (q6fsm->psy == NULL) {
		fsm_err("get power supply failed");
		return -EINVAL;
	}

	psy = q6fsm->psy;
	if (PSY_DESC == NULL || PSY_DESC->get_property == NULL) {
		fsm_err("Power desc/get_property is null!");
		return -EINVAL;
	}
	PSY_DESC->get_property(psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	*batv = prop.intval;
	fsm_dbg("voltage: %d", *batv);

	return 0;
}

static int q6fsm_get_bsg_config(struct q6fsm_afe *q6fsm)
{
	struct param_hdr_v3 param_hdr;
	struct bsg_config_v2 *config;
	int interval;
	int ret;

	if (q6fsm == NULL)
		return -EINVAL;

	if (q6fsm->delay_period > 0)
		return 0;

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_BSGV2_CONFIG;
	param_hdr.param_size = sizeof(q6fsm->config);
	ret = q6fsm_afe_get_params(&param_hdr,
		(uint8_t *)&q6fsm->config, sizeof(q6fsm->config));
	if (ret) {
		fsm_err("Get params fail:%d", ret);
		return ret;
	}

	config = &q6fsm->config;
	if (!config->bsg_enable && !config->tc_mode_enable) {
		fsm_info("BSG/CSG is disabled");
		return 0x1000; /* Not enabled, stop the monitor */
	}

	if (config->bsg_enable)
		interval = config->bsg_interval;
	if (config->tc_mode_enable)
		interval = config->tc_interval;

	fsm_info("interval: %d", interval);
	q6fsm->delay_period = interval*HZ/1000;

	return 0;
}

static int q6fsm_gen_bsg_param(struct q6fsm_afe *q6fsm,
		struct bsg_param_v2 *param)
{
	union power_supply_propval prop = { 0 };
	struct power_supply *psy;

	if (q6fsm == NULL || param == NULL) {
		fsm_err("Bad paramters");
		return -EINVAL;
	}

	q6fsm->psy = power_supply_get_by_name("battery");
	if (q6fsm->psy == NULL) {
		fsm_err("get power supply failed");
		return -EINVAL;
	}

	psy = q6fsm->psy;
	if (PSY_DESC == NULL || PSY_DESC->get_property == NULL) {
		fsm_err("Power desc/get_property is null!");
		return -EINVAL;
	}

	param->bsg_mode = q6fsm->config.bsg_mode;
	if (param->bsg_mode == 1) {
		PSY_DESC->get_property(psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		param->bsg_val = prop.intval;
	} else if (param->bsg_mode == 2) {
		PSY_DESC->get_property(psy,
				POWER_SUPPLY_PROP_CAPACITY, &prop);
		param->bsg_val = prop.intval;
	} else {
		param->bsg_val = 0;
	}

	param->tc_mode = q6fsm->config.tc_mode;
	if (param->tc_mode == 3) {
		PSY_DESC->get_property(psy,
				POWER_SUPPLY_PROP_TEMP, &prop);
		param->tc_val = prop.intval;
	} else {
		param->tc_val = 0;
	}

	fsm_dbg("bsg:%d-%d, tc:%d-%d",
			param->bsg_mode, param->bsg_val,
			param->tc_mode, param->tc_val);

	return 0;
}

static int q6fsm_vbat_monitor_work_v2(struct q6fsm_afe *q6fsm)
{
	struct param_hdr_v3 param_hdr;
	struct bsg_param_v2 param;
	int ret;

	if (q6fsm == NULL)
		return -EINVAL;

	ret = q6fsm_get_bsg_config(q6fsm);
	if (ret) {
		if (ret != 0x1000)
			fsm_err("Get bsg config fail:%d", ret);
		return ret;
	}

	ret = q6fsm_gen_bsg_param(q6fsm, &param);
	if (ret) {
		fsm_err("Get batt voltage fail:%d", ret);
		return ret;
	}

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_BSGV2_PARAM;
	param_hdr.param_size = sizeof(param);
	ret = q6fsm_afe_set_params(&param_hdr,
		(uint8_t *)&param, sizeof(param));
	if (ret) {
		fsm_err("Set params fail:%d", ret);
		return ret;
	}

	return 0;
}

static int q6fsm_vbat_monitor_work(struct q6fsm_afe *q6fsm)
{
	struct param_hdr_v3 param_hdr;
	uint32_t batv;
	int ret;

	if (q6fsm == NULL)
		return -EINVAL;

	if (!q6fsm_get_rx_status()) {
		fsm_info("RX module isn't ready");
		return 0;
	}

	if (q6fsm->bsg_version == 2) {
		ret = q6fsm_vbat_monitor_work_v2(q6fsm);
		if (!ret || ret == 0x1000)
			return ret;
	}

	q6fsm->bsg_version = 1;
	q6fsm->delay_period = 2*HZ;

	ret = q6fsm_get_batv(q6fsm, &batv);
	if (ret) {
		fsm_err("Get batt voltage fail:%d", ret);
		return ret;
	}

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_BSG_VBAT;
	param_hdr.param_size = sizeof(batv);
	ret = q6fsm_afe_set_params(&param_hdr,
		(uint8_t *)&batv, sizeof(batv));
	if (ret) {
		fsm_err("Set params fail:%d", ret);
		return ret;
	}

	return 0;
}

static void q6fsm_vbat_monitor(struct work_struct *work)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;
	int ret;

	if (q6fsm == NULL)
		return;

	ret = q6fsm_vbat_monitor_work(q6fsm);
	if (ret)
		return;

	/* reschedule */
	queue_delayed_work(q6fsm->vbat_wq,
		&q6fsm->vbat_work, q6fsm->delay_period);
}

static void q6fsm_monitor_switch(bool enable)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;

	if (q6fsm == NULL)
		return;

	fsm_info("%s", enable ? "enable" : "disable");
	if (enable) {
		if (q6fsm->monitor_en)
			return;
		q6fsm->monitor_en = true;
		queue_delayed_work(q6fsm->vbat_wq, &q6fsm->vbat_work, 0);
	} else {
		if (!q6fsm->monitor_en)
			return;
		q6fsm->monitor_en = false;
		q6fsm->delay_period = 0;
		cancel_delayed_work_sync(&q6fsm->vbat_work);
	}
}

static ssize_t q6fsm_rx_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct param_hdr_v3 param_hdr;
	int len, enable;
	int ret;

	if (!q6fsm_check_dsp_ready()) {
		fsm_info("DSP not ready yet!");
		return -EINVAL;
	}

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_RX_ENABLE;
	param_hdr.param_size = sizeof(enable);

	ret = q6fsm_afe_get_params(&param_hdr,
		(uint8_t *)&enable, sizeof(enable));
	if (ret) {
		fsm_err("Get params failed: %d", ret);
		return ret;
	}
	len = scnprintf(buf, PAGE_SIZE, "%s\n",
		enable ? "On" : "Off");

	return len;
}

static ssize_t q6fsm_rx_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t len)
{
	struct param_hdr_v3 param_hdr;
	int enable;
	int ret = -1;

	if (buf == NULL)
		return -ENOMEM;

	enable = simple_strtoul(buf, NULL, 0);

	if (!q6fsm_check_dsp_ready()) {
		fsm_info("DSP not ready yet!");
		return ret;
	}

	param_hdr.module_id = AFE_MODULE_ID_FSADSP_RX;
	param_hdr.instance_id = 0;
	param_hdr.reserved = 0;
	param_hdr.param_id = CAPI_V2_PARAM_FSADSP_RX_ENABLE;
	param_hdr.param_size = sizeof(enable);
	enable = !!enable;

	ret = q6fsm_afe_set_params(&param_hdr,
		(uint8_t *)&enable, sizeof(enable));
	if (ret) {
		fsm_err("Set params failed: %d", ret);
		return ret;
	}

	return len;
}

static ssize_t q6fsm_bsg_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;
	int len;

	if (q6fsm == NULL)
		return -EINVAL;

	len = scnprintf(buf, PAGE_SIZE, "%s\n",
		q6fsm->monitor_en ? "On" : "Off");

	return len;
}

static ssize_t q6fsm_bsg_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t len)
{
	int enable;

	if (buf == NULL)
		return -ENOMEM;

	enable = simple_strtoul(buf, NULL, 0);
	q6fsm_monitor_switch(!!enable);

	return len;
}

static CLASS_ATTR_RW(q6fsm_rx);
static CLASS_ATTR_RW(q6fsm_bsg);

static struct attribute *q6fsm_class_attrs[] = {
	&class_attr_q6fsm_rx.attr,
	&class_attr_q6fsm_bsg.attr,
	NULL,
};
ATTRIBUTE_GROUPS(q6fsm_class);

/** Device model classes */
struct class g_q6fsm_class = {
	.name = "q6fsm",
	.class_groups = q6fsm_class_groups,
};

static int __init q6fsm_afe_init(void)
{
	struct q6fsm_afe *q6fsm;

	fsm_info("version: %s", Q6FSM_VERSION);

	q6fsm = kzalloc(sizeof(struct q6fsm_afe), GFP_KERNEL);
	if (q6fsm == NULL) {
		fsm_err("alloc memery failed");
		return -ENOMEM;
	}

	mutex_init(&q6fsm->cmd_lock);
	atomic_set(&q6fsm->bsg_switch, 0);

	/* BSG V1 */
	//q6fsm->delay_period = 2*HZ;
	//q6fsm->bsg_version = 1;
	/* BSG V2 */
	q6fsm->delay_period = 0; /* Get from adsp */
	q6fsm->bsg_version = 2;
	q6fsm->monitor_en = false;
	q6fsm->vbat_wq = create_singlethread_workqueue("q6fsm-vbat");
	INIT_DELAYED_WORK(&q6fsm->vbat_work, q6fsm_vbat_monitor);
	/* path: sys/class/$(CLASS NAME) */
	class_register(&g_q6fsm_class);

	g_fsm_afe = q6fsm;

	return 0;
}

static void q6fsm_afe_deinit(void)
{
	struct q6fsm_afe *q6fsm = g_fsm_afe;

	if (q6fsm == NULL)
		return;

	cancel_delayed_work_sync(&q6fsm->vbat_work);
	destroy_workqueue(q6fsm->vbat_wq);
	mutex_destroy(&q6fsm->cmd_lock);

	kfree(g_fsm_afe);
	g_fsm_afe = NULL;
}

#endif // __Q6FSM_V3_H__
