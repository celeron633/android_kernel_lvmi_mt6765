// SPDX-License-Identifier: GPL-2.0-only
/*
 * S17Promini AXS15260 basic reports reconstructed from its stock kernel.
 * The LCM owns the shared TDDI reset; this driver does not request GPIO 174.
 * Manual stock PRAM download is opt-in; flash upgrades remain separate work.
 */
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include <linux/fb.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/axs15260_s17.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "axs15260_s17.h"
#include "axs15260_s17_pram.h"

#define AXS_DEBUG_HISTORY 16
#define AXS_DEBUG_POLL_SECONDS 30
#define AXS_FLASH_REGION_SIZE 4096U
#define AXS_FLASH_REGIONS 13

struct axs_debug_report {
	u64 time_ns;
	int read_result;
	int decode_result;
	char source;
	u8 raw[AXS_REPORT_SIZE];
};

struct axs_touch {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *supply;
	struct touchscreen_properties props;
	struct notifier_block fb_notifier;
	struct mutex state_lock;
	struct mutex report_lock;
	struct mutex debug_lock;
	struct delayed_work poll_work;
	unsigned int poll_ms;
	unsigned long poll_deadline;
	struct pinctrl *pinctrl;
	struct pinctrl_state *boot_default;
	struct pinctrl_state *debug_pull_up;
	struct pinctrl_state *debug_no_pull;
	const char *pin_profile;
	bool debug_reports;
	u64 poll_reports;
	u64 read_errors;
	u64 decode_errors;
	u64 zero_reports;
	u64 ff_reports;
	u64 contact_reports;
	u64 empty_reports;
	unsigned int history_next;
	unsigned int history_count;
	struct axs_debug_report history[AXS_DEBUG_HISTORY];
	u64 irq_reports;
	u64 manual_reports;
	int last_read;
	int last_decode;
	u8 last_report[AXS_REPORT_SIZE];
	u8 firmware_version[2];
	int firmware_read;
	u64 reset_requests;
	u64 last_reset_ns;
	int last_reset_result;
	bool fw_busy;
	unsigned int fw_stage;
	unsigned int fw_pages;
	u16 fw_version;
	int fw_result;
	unsigned long fw_deadline;
	const struct firmware *pram_fw;
	u8 *pram_packet;
	bool fw_probe;
	bool fw_repair;
	unsigned int fw_written_pages;
	u8 flash_id_tail;
	u8 flash_status;
	u32 flash_crc;
	unsigned int flash_mismatches;
	u8 flash_first[32];
	u8 *flash_dump;
	u8 *flash_backup;
	bool flash_dump_valid;
	bool flash_backup_valid;
	int flash_first_mismatch;
	unsigned int flash_ff_bytes;
	unsigned int flash_trailing_ff;
	u32 flash_region_crc[AXS_FLASH_REGIONS];
	unsigned int flash_region_ff[AXS_FLASH_REGIONS];
	unsigned int flash_region_bad[AXS_FLASH_REGIONS];
	bool fb_suspended;
	bool system_suspended;
	bool irq_disabled;
	bool pressed;
};

static void axs_release(struct axs_touch *touch)
{
	if (!touch->pressed)
		return;
	input_report_key(touch->input, BTN_TOUCH, 0);
	input_mt_sync(touch->input);
	input_sync(touch->input);
	touch->pressed = false;
}

/* report_lock serializes IRQ and explicitly requested diagnostic reads. */
static irqreturn_t axs_read_report_data(struct axs_touch *touch)
{
	struct axs_frame frame;
	u8 buf[AXS_REPORT_SIZE];
	struct i2c_msg msg = {
		.addr = touch->client->addr,
		.flags = I2C_M_RD,
		.len = sizeof(buf),
		.buf = buf,
	};
	unsigned int i, active = 0;
	int ret;

	/* Stock reads one message directly, without a register-address write. */
	ret = i2c_transfer(touch->client->adapter, &msg, 1);
	touch->last_read = ret;
	touch->last_decode = -ENODATA;
	if (ret != 1) {
		dev_err_ratelimited(&touch->client->dev,
				    "report read failed: %d\n", ret);
		axs_release(touch);
		return IRQ_HANDLED;
	}
	memcpy(touch->last_report, buf, sizeof(buf));
	ret = axs_decode_report(buf, sizeof(buf), &frame);
	touch->last_decode = ret;
	if (ret)
		return IRQ_HANDLED;

	for (i = 0; i < frame.count; i++) {
		struct axs_point *point = &frame.points[i];

		/* Vendor type-A reports consider events 0/2 down/contact. */
		if (point->event != 0 && point->event != 2)
			continue;
		input_report_abs(touch->input, ABS_MT_TRACKING_ID, point->id);
		input_report_abs(touch->input, ABS_MT_PRESSURE, point->pressure);
		input_report_abs(touch->input, ABS_MT_TOUCH_MAJOR, point->area);
		touchscreen_report_pos(touch->input, &touch->props,
				       point->x, point->y, true);
		input_mt_sync(touch->input);
		active++;
	}
	if (!active) {
		axs_release(touch);
		return IRQ_HANDLED;
	}
	input_report_key(touch->input, BTN_TOUCH, 1);
	input_sync(touch->input);
	touch->pressed = true;
	return IRQ_HANDLED;
}

/* Called with report_lock held. Preserve even failed reads with their result. */
static irqreturn_t axs_read_report(struct axs_touch *touch, char source)
{
	struct axs_debug_report *record;
	irqreturn_t ret;

	memset(touch->last_report, 0, sizeof(touch->last_report));
	ret = axs_read_report_data(touch);
	if (touch->last_read != 1) {
		touch->read_errors++;
	} else {
		if (!memchr_inv(touch->last_report, 0, AXS_REPORT_SIZE))
			touch->zero_reports++;
		if (!memchr_inv(touch->last_report, 0xff, AXS_REPORT_SIZE))
			touch->ff_reports++;
		if (touch->last_decode)
			touch->decode_errors++;
		else if (touch->pressed)
			touch->contact_reports++;
		else
			touch->empty_reports++;
	}
	record = &touch->history[touch->history_next];
	record->time_ns = ktime_get_ns();
	record->source = source;
	record->read_result = touch->last_read;
	record->decode_result = touch->last_decode;
	memcpy(record->raw, touch->last_report, sizeof(record->raw));
	touch->history_next = (touch->history_next + 1) % AXS_DEBUG_HISTORY;
	if (touch->history_count < AXS_DEBUG_HISTORY)
		touch->history_count++;
	if (touch->debug_reports)
		dev_info_ratelimited(&touch->client->dev,
			"debug source=%c read=%d decode=%d pressed=%d raw=%*ph\n",
			source, touch->last_read, touch->last_decode, touch->pressed,
			AXS_REPORT_SIZE, touch->last_report);
	return ret;
}

static irqreturn_t axs_irq_thread(int irq, void *data)
{
	struct axs_touch *touch = data;
	irqreturn_t ret;

	mutex_lock(&touch->report_lock);
	touch->irq_reports++;
	ret = axs_read_report(touch, 'I');
	mutex_unlock(&touch->report_lock);
	return ret;
}

static ssize_t diagnostics_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	struct irq_data *irq_data = irq_get_irq_data(touch->client->irq);
	ssize_t len;

	mutex_lock(&touch->state_lock);
	mutex_lock(&touch->report_lock);
	len = scnprintf(buf, PAGE_SIZE,
		"irq=%d disabled=%d fb_suspended=%d system_suspended=%d core_disabled=%d\n"
		"irq_reports=%llu manual_reports=%llu firmware_read=%d firmware=%02x%02x\n"
		"last_read=%d last_decode=%d pressed=%d raw=%*ph\n"
		"irq_type=%u profile=%s debug_reports=%d poll_ms=%u poll_reports=%llu\n"
		"read_errors=%llu decode_errors=%llu zero=%llu ff=%llu contact=%llu empty=%llu\n"
		"reset_requests=%llu last_reset_ns=%llu last_reset_result=%d\n",
		touch->client->irq, touch->irq_disabled, touch->fb_suspended,
		touch->system_suspended, irq_data ? irqd_irq_disabled(irq_data) : -1,
		(unsigned long long)touch->irq_reports,
		(unsigned long long)touch->manual_reports,
		touch->firmware_read, touch->firmware_version[0], touch->firmware_version[1],
		touch->last_read, touch->last_decode, touch->pressed,
		AXS_REPORT_SIZE, touch->last_report,
		irq_get_trigger_type(touch->client->irq), touch->pin_profile,
		touch->debug_reports, touch->poll_ms,
		(unsigned long long)touch->poll_reports,
		(unsigned long long)touch->read_errors,
		(unsigned long long)touch->decode_errors,
		(unsigned long long)touch->zero_reports,
		(unsigned long long)touch->ff_reports,
		(unsigned long long)touch->contact_reports,
		(unsigned long long)touch->empty_reports,
		(unsigned long long)touch->reset_requests,
		(unsigned long long)touch->last_reset_ns, touch->last_reset_result);
	mutex_unlock(&touch->report_lock);
	mutex_unlock(&touch->state_lock);
	return len;
}
static DEVICE_ATTR_RO(diagnostics);

/* Same 0x0c query used by the stock driver, serialized with report reads.
 * This selects a read register; it does not erase/write touch firmware.
 */
static ssize_t live_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	u8 cmd = 0x0c, version[2] = { 0xa5, 0xa5 };
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = 1, .buf = &cmd },
		{ .addr = touch->client->addr, .flags = I2C_M_RD,
		  .len = sizeof(version), .buf = version },
	};
	int ret, enabled, voltage;

	mutex_lock(&touch->state_lock);
	if (touch->fb_suspended || touch->system_suspended) {
		mutex_unlock(&touch->state_lock);
		return -EBUSY;
	}
	mutex_lock(&touch->report_lock);
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	mutex_unlock(&touch->report_lock);
	enabled = regulator_is_enabled(touch->supply);
	voltage = regulator_get_voltage(touch->supply);
	mutex_unlock(&touch->state_lock);
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	return scnprintf(buf, PAGE_SIZE,
		"version=%02x%02x supply_enabled=%d supply_uV=%d\n",
		version[0], version[1], enabled, voltage);
}
static DEVICE_ATTR_RO(live_version);

/* Power status without an I2C transaction or changing regulator state. */
static ssize_t supply_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	int enabled, voltage;

	mutex_lock(&touch->state_lock);
	enabled = regulator_is_enabled(touch->supply);
	voltage = regulator_get_voltage(touch->supply);
	mutex_unlock(&touch->state_lock);
	return scnprintf(buf, PAGE_SIZE, "enabled=%d voltage_uV=%d\n",
			 enabled, voltage);
}
static DEVICE_ATTR_RO(supply_state);

/* Explicit touch reset, coordinated with the existing GPIO174 owner.
 * state_lock pauses diagnostic polling; disable_irq drains the IRQ thread.
 */
static ssize_t reset_touch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int settle_ms;
	int ret = kstrtouint(buf, 0, &settle_ms);

	if (ret)
		return ret;
	if (settle_ms < 20 || settle_ms > 200)
		return -EINVAL;
	mutex_lock(&touch->state_lock);
	if (touch->fb_suspended || touch->system_suspended) {
		mutex_unlock(&touch->state_lock);
		return -EBUSY;
	}
	disable_irq(touch->client->irq);
	mutex_lock(&touch->report_lock);
	axs_release(touch);
	touch->reset_requests++;
	ret = s17_axs15260_reset_touch(settle_ms);
	touch->last_reset_result = ret;
	touch->last_reset_ns = ktime_get_ns();
	mutex_unlock(&touch->report_lock);
	enable_irq(touch->client->irq);
	mutex_unlock(&touch->state_lock);
	dev_info(dev, "manual touch reset: settle=%u result=%d\n", settle_ms, ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(reset_touch);

enum axs_fw_stage {
	AXS_FW_IDLE,
	AXS_FW_REQUEST,
	AXS_FW_VALIDATE,
	AXS_FW_SELECT_PRAM,
	AXS_FW_WRITE,
	AXS_FW_FINISH,
	AXS_FW_VERSION,
	AXS_FW_DONE,
	AXS_FW_FAILED,
	AXS_FW_PROBE_READ,
	AXS_FW_SELECT_FLASH,
	AXS_FW_REPAIR_INIT,
	AXS_FW_REPAIR_WRITE,
	AXS_FW_REPAIR_VERIFY,
};

static ssize_t firmware_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	static const char * const stages[] = {
		"idle", "request", "validate", "select-pram", "write",
		"finish", "version", "done", "failed", "read-flash", "select-flash",
		"flash-init", "repair-write", "verify",
	};
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int stage = READ_ONCE(touch->fw_stage);

	/* Approximate progress is intentionally readable during blocking upload. */
	return scnprintf(buf, PAGE_SIZE,
		"busy=%d stage=%s pages=%u/197 result=%d version=%04x mode=%s written_pages=%u\n",
		READ_ONCE(touch->fw_busy),
		stage < ARRAY_SIZE(stages) ? stages[stage] : "unknown",
		READ_ONCE(touch->fw_pages), READ_ONCE(touch->fw_result),
		READ_ONCE(touch->fw_version),
		READ_ONCE(touch->fw_repair) ? "repair-ff" :
		READ_ONCE(touch->fw_probe) ? "read-probe" : "pram",
		READ_ONCE(touch->fw_written_pages));
}
static DEVICE_ATTR_RO(firmware_status);

static ssize_t flash_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len;

	mutex_lock(&touch->report_lock);
	len = scnprintf(buf, PAGE_SIZE,
		"id_tail=%02x status=%02x crc32=%08x expected=%08x mismatch_bytes=%u first=%*ph\n"
		"first_mismatch=%d ff_bytes=%u trailing_ff=%u dump_valid=%d backup_valid=%d\n",
		touch->flash_id_tail, touch->flash_status, touch->flash_crc,
		AXS_S17_FW_CRC32, touch->flash_mismatches,
		(int)sizeof(touch->flash_first), touch->flash_first,
		touch->flash_first_mismatch, touch->flash_ff_bytes, touch->flash_trailing_ff,
		touch->flash_dump_valid, touch->flash_backup_valid);
	mutex_unlock(&touch->report_lock);
	return len;
}
static DEVICE_ATTR_RO(flash_status);

static ssize_t flash_regions_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len = 0;
	unsigned int i, end;

	mutex_lock(&touch->report_lock);
	for (i = 0; i < AXS_FLASH_REGIONS; i++) {
		end = min_t(unsigned int, (i + 1) * AXS_FLASH_REGION_SIZE, AXS_S17_FW_SIZE);
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"%04x..%04x crc32=%08x ff=%u mismatch=%u valid=%d\n",
			i * AXS_FLASH_REGION_SIZE, end, touch->flash_region_crc[i],
			touch->flash_region_ff[i], touch->flash_region_bad[i],
			touch->flash_dump_valid);
	}
	mutex_unlock(&touch->report_lock);
	return len;
}
static DEVICE_ATTR_RO(flash_regions);

static ssize_t flash_binary_read(struct file *file, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	bool backup = !strcmp(attr->attr.name, "flash_backup");
	ssize_t ret;

	if (offset < 0)
		return -EINVAL;
	mutex_lock(&touch->report_lock);
	if (!(backup ? touch->flash_backup_valid : touch->flash_dump_valid)) {
		ret = -ENODATA;
	} else if (offset >= AXS_S17_FW_SIZE) {
		ret = 0;
	} else {
		count = min_t(size_t, count, AXS_S17_FW_SIZE - offset);
		memcpy(buf, (backup ? touch->flash_backup : touch->flash_dump) + offset, count);
		ret = count;
	}
	mutex_unlock(&touch->report_lock);
	return ret;
}

static struct bin_attribute axs_flash_dump_attr = {
	.attr = { .name = "flash_dump", .mode = 0400 },
	.size = AXS_S17_FW_SIZE,
	.read = flash_binary_read,
};
static struct bin_attribute axs_flash_backup_attr = {
	.attr = { .name = "flash_backup", .mode = 0400 },
	.size = AXS_S17_FW_SIZE,
	.read = flash_binary_read,
};
static struct bin_attribute *axs_bin_attrs[] = {
	&axs_flash_dump_attr,
	&axs_flash_backup_attr,
	NULL,
};

static int axs_pram_write(void *data, const u8 *buf, unsigned int len)
{
	struct axs_touch *touch = data;
	int ret;

	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	ret = i2c_master_send(touch->client, buf, len);
	if (ret != (int)len)
		return ret < 0 ? ret : -EIO;
	return time_after_eq(jiffies, touch->fw_deadline) ? -ETIMEDOUT : 0;
}

static int axs_pram_download(void *data)
{
	static const u8 finish[] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 0, 0, 0 };
	struct axs_touch *touch = data;
	u8 command = 0x0c, version[2] = { 0, 0 };
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = 1, .buf = &command },
		{ .addr = touch->client->addr, .flags = I2C_M_RD,
		  .len = sizeof(version), .buf = version },
	};
	unsigned int offset, size, wire_size;
	int ret;

	WRITE_ONCE(touch->fw_stage, AXS_FW_WRITE);
	for (offset = 0; offset < touch->pram_fw->size;
	     offset += AXS_PRAM_PAGE_SIZE) {
		size = min_t(unsigned int, AXS_PRAM_PAGE_SIZE,
			     touch->pram_fw->size - offset);
		wire_size = axs_pram_packet(touch->pram_packet,
					   touch->pram_fw->data, offset, size);
		if (!wire_size)
			return -EINVAL;
		ret = axs_pram_write(touch, touch->pram_packet, wire_size);
		if (ret)
			return ret;
		WRITE_ONCE(touch->fw_pages, offset / AXS_PRAM_PAGE_SIZE + 1);
		if (!(touch->fw_pages % 32))
			dev_info(&touch->client->dev, "PRAM progress: %u/197\n",
				 touch->fw_pages);
	}
	WRITE_ONCE(touch->fw_stage, AXS_FW_FINISH);
	ret = axs_pram_write(touch, finish, sizeof(finish));
	if (ret)
		return ret;
	msleep(100);
	WRITE_ONCE(touch->fw_stage, AXS_FW_VERSION);
	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	WRITE_ONCE(touch->fw_version, (version[0] << 8) | version[1]);
	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	return touch->fw_version == AXS_S17_FW_VERSION ? 0 : -EPROTO;
}

static int axs_flash_read(void *data)
{
	/* Same bridge headers/opcodes as the stock read operations. */
	u8 command[12] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 3, 0, 1, 0x9f, 0, 0, 0 };
	static const u8 read_init[] = {
		0xab, 0xb5, 0xa5, 0x5a, 0, 4, 0, 0, 0, 0, 0x10, 0,
	};
	struct axs_touch *touch = data;
	u8 *rx = touch->pram_packet;
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = 11, .buf = command },
		{ .addr = touch->client->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = rx },
	};
	unsigned int offset, size, i, region, last_non_ff = 0;
	u32 crc = ~0U;
	int ret;

	if (READ_ONCE(touch->fw_stage) != AXS_FW_REPAIR_VERIFY)
		WRITE_ONCE(touch->fw_stage, AXS_FW_PROBE_READ);
	touch->flash_dump_valid = false;
	touch->flash_first_mismatch = -1;
	touch->flash_mismatches = 0;
	touch->flash_ff_bytes = 0;
	touch->flash_trailing_ff = 0;
	touch->flash_crc = 0;
	WRITE_ONCE(touch->fw_pages, 0);
	memset(touch->flash_region_ff, 0, sizeof(touch->flash_region_ff));
	memset(touch->flash_region_bad, 0, sizeof(touch->flash_region_bad));
	for (i = 0; i < AXS_FLASH_REGIONS; i++)
		touch->flash_region_crc[i] = ~0U;
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	touch->flash_id_tail = rx[0];
	command[8] = 5;
	command[10] = 0xff;
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	touch->flash_status = rx[0];
	ret = axs_pram_write(touch, read_init, sizeof(read_init));
	if (ret)
		return ret;
	usleep_range(2000, 2100);
	command[4] = 0;
	command[5] = 4;
	command[8] = 0x0b;
	msgs[0].len = sizeof(command);
	for (offset = 0; offset < AXS_S17_FW_SIZE; offset += AXS_PRAM_PAGE_SIZE) {
		if (time_after_eq(jiffies, touch->fw_deadline))
			return -ETIMEDOUT;
		size = min_t(unsigned int, AXS_PRAM_PAGE_SIZE, AXS_S17_FW_SIZE - offset);
		command[6] = (size + 1) >> 8;
		command[7] = size + 1;
		command[9] = offset >> 16;
		command[10] = offset >> 8;
		command[11] = offset;
		msgs[1].len = size + 1;
		ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret != ARRAY_SIZE(msgs))
			return ret < 0 ? ret : -EIO;
		/* Stock fast-read discards the first dummy byte. */
		crc = crc32_le(crc, rx + 1, size);
		region = offset / AXS_FLASH_REGION_SIZE;
		touch->flash_region_crc[region] =
			crc32_le(touch->flash_region_crc[region], rx + 1, size);
		if (touch->flash_dump)
			memcpy(touch->flash_dump + offset, rx + 1, size);
		if (!offset)
			memcpy(touch->flash_first, rx + 1, sizeof(touch->flash_first));
		for (i = 0; i < size; i++) {
			if (rx[i + 1] == 0xff) {
				touch->flash_ff_bytes++;
				touch->flash_region_ff[region]++;
			} else {
				last_non_ff = offset + i + 1;
			}
			if (rx[i + 1] != touch->pram_fw->data[offset + i]) {
				if (touch->flash_first_mismatch < 0)
					touch->flash_first_mismatch = offset + i;
				touch->flash_mismatches++;
				touch->flash_region_bad[region]++;
			}
		}
		WRITE_ONCE(touch->fw_pages, offset / AXS_PRAM_PAGE_SIZE + 1);
	}
	touch->flash_crc = crc ^ ~0U;
	touch->flash_trailing_ff = AXS_S17_FW_SIZE - last_non_ff;
	for (i = 0; i < AXS_FLASH_REGIONS; i++)
		touch->flash_region_crc[i] ^= ~0U;
	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	touch->flash_dump_valid = !!touch->flash_dump;
	return 0;
}

static int axs_flash_spi_status(struct axs_touch *touch, u8 *status)
{
	u8 command[] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 3, 0, 1, 5, 0, 0xff };
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = sizeof(command), .buf = command },
		{ .addr = touch->client->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = status },
	};
	int ret;

	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	return ret == ARRAY_SIZE(msgs) ? 0 : (ret < 0 ? ret : -EIO);
}

static int axs_flash_wait_ready(struct axs_touch *touch)
{
	unsigned long end = jiffies + msecs_to_jiffies(2000);
	u8 status;
	int ret;

	do {
		ret = axs_flash_spi_status(touch, &status);
		if (ret)
			return ret;
		touch->flash_status = status;
		/* Stock waits for the entire status byte to become zero. */
		if (!status)
			return 0;
		usleep_range(1000, 1100);
	} while (time_before(jiffies, end));
	return -ETIMEDOUT;
}

static int axs_flash_init(struct axs_touch *touch)
{
	static const u8 wren[] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 1, 0, 0, 6 };
	static const u8 write_status[] = {
		0xab, 0xb5, 0xa5, 0x5a, 0, 2, 0, 0, 1, 2,
	};
	u8 command[] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 3, 0, 1, 0x9f, 0, 0 };
	u8 id;
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = sizeof(command), .buf = command },
		{ .addr = touch->client->addr, .flags = I2C_M_RD, .len = 1, .buf = &id },
	};
	int ret;

	WRITE_ONCE(touch->fw_stage, AXS_FW_REPAIR_INIT);
	ret = axs_pram_write(touch, wren, sizeof(wren));
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	if (id != 0x12)
		return -ENODEV;
	usleep_range(1000, 1100);
	ret = axs_pram_write(touch, wren, sizeof(wren));
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	ret = axs_flash_spi_status(touch, &touch->flash_status);
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	ret = axs_pram_write(touch, write_status, sizeof(write_status));
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	return axs_flash_wait_ready(touch);
}

static int axs_flash_repair(void *data)
{
	static const u8 wren[] = { 0xab, 0xb5, 0xa5, 0x5a, 0, 1, 0, 0, 6 };
	struct axs_touch *touch = data;
	unsigned int offset, size, wire_size;
	int ret;

	ret = axs_flash_init(touch);
	if (ret)
		return ret;
	WRITE_ONCE(touch->fw_stage, AXS_FW_REPAIR_WRITE);
	WRITE_ONCE(touch->fw_pages, 0);
	touch->flash_dump_valid = false;
	for (offset = 0; offset < AXS_S17_FW_SIZE; offset += AXS_PRAM_PAGE_SIZE) {
		size = min_t(unsigned int, AXS_PRAM_PAGE_SIZE, AXS_S17_FW_SIZE - offset);
		if (memcmp(touch->flash_backup + offset, touch->pram_fw->data + offset, size)) {
			ret = axs_pram_write(touch, wren, sizeof(wren));
			if (ret)
				return ret;
			wire_size = axs_flash_packet(touch->pram_packet,
						    touch->pram_fw->data, offset, size);
			if (!wire_size)
				return -EINVAL;
			ret = axs_pram_write(touch, touch->pram_packet, wire_size);
			if (ret)
				return ret;
			ret = axs_flash_wait_ready(touch);
			if (ret)
				return ret;
			WRITE_ONCE(touch->fw_written_pages, touch->fw_written_pages + 1);
			dev_info(&touch->client->dev, "repair page=%u written=%u\n",
				 offset / AXS_PRAM_PAGE_SIZE, touch->fw_written_pages);
		}
		WRITE_ONCE(touch->fw_pages, offset / AXS_PRAM_PAGE_SIZE + 1);
	}
	msleep(100);
	WRITE_ONCE(touch->fw_stage, AXS_FW_REPAIR_VERIFY);
	ret = axs_flash_read(touch);
	if (ret)
		return ret;
	if (touch->flash_mismatches || touch->flash_crc != AXS_S17_FW_CRC32)
		return -EIO;
	return 0;
}

static int axs_read_current_version(void *data)
{
	struct axs_touch *touch = data;
	u8 cmd = 0x0c, version[2];
	struct i2c_msg msgs[] = {
		{ .addr = touch->client->addr, .len = 1, .buf = &cmd },
		{ .addr = touch->client->addr, .flags = I2C_M_RD,
		  .len = sizeof(version), .buf = version },
	};
	int ret;

	WRITE_ONCE(touch->fw_stage, AXS_FW_VERSION);
	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	ret = i2c_transfer(touch->client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	WRITE_ONCE(touch->fw_version, (version[0] << 8) | version[1]);
	if (time_after_eq(jiffies, touch->fw_deadline))
		return -ETIMEDOUT;
	return touch->fw_version == AXS_S17_FW_VERSION ? 0 : -EPROTO;
}

static ssize_t axs_firmware_run(struct device *dev, bool probe, bool repair, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	const struct firmware *fw = NULL;
	u8 *packet = NULL;
	int ret;

	if (!mutex_trylock(&touch->debug_lock))
		return -EBUSY;
	WRITE_ONCE(touch->fw_busy, true);
	touch->fw_probe = probe;
	touch->fw_repair = repair;
	WRITE_ONCE(touch->fw_written_pages, 0);
	WRITE_ONCE(touch->fw_stage, AXS_FW_REQUEST);
	WRITE_ONCE(touch->fw_pages, 0);
	WRITE_ONCE(touch->fw_version, 0);
	WRITE_ONCE(touch->fw_result, -EINPROGRESS);
	ret = request_firmware_direct(&fw, "axs15260-s17.bin", dev);
	if (ret)
		goto out;
	WRITE_ONCE(touch->fw_stage, AXS_FW_VALIDATE);
	if (fw->size != AXS_S17_FW_SIZE ||
	    (crc32_le(~0U, fw->data, fw->size) ^ ~0U) != AXS_S17_FW_CRC32 ||
	    fw->data[AXS_S17_FW_VERSION_OFFSET] != 0 ||
	    fw->data[AXS_S17_FW_VERSION_OFFSET + 1] != AXS_S17_FW_VERSION) {
		ret = -EINVAL;
		goto out;
	}
	if ((probe || repair) && (!touch->flash_dump || !touch->flash_backup)) {
		ret = -ENOMEM;
		goto out;
	}
	packet = kmalloc(AXS_FLASH_HEADER_SIZE + AXS_PRAM_PAGE_SIZE, GFP_KERNEL);
	if (!packet) {
		ret = -ENOMEM;
		goto out;
	}
	mutex_lock(&touch->state_lock);
	if (touch->fb_suspended || touch->system_suspended) {
		ret = -EBUSY;
		goto unlock_state;
	}
	disable_irq(touch->client->irq);
	mutex_lock(&touch->report_lock);
	axs_release(touch);
	touch->flash_id_tail = 0;
	touch->flash_status = 0;
	touch->flash_crc = 0;
	touch->flash_mismatches = 0;
	memset(touch->flash_first, 0, sizeof(touch->flash_first));
	touch->pram_fw = fw;
	touch->pram_packet = packet;
	touch->fw_deadline = jiffies + 10 * HZ;
	if (repair)
		touch->fw_deadline = jiffies + 30 * HZ;
	WRITE_ONCE(touch->fw_stage, (probe || repair) ? AXS_FW_SELECT_FLASH : AXS_FW_SELECT_PRAM);
	dev_info(dev, "manual stock firmware %s begin: %zu bytes\n",
		 repair ? "FF repair" : probe ? "read probe" : "PRAM download", fw->size);
	if (probe || repair)
		ret = s17_axs15260_flash_read_session(axs_pram_write, axs_flash_read, touch);
	else
		ret = s17_axs15260_pram_session(axs_pram_write, axs_pram_download, touch);
	if (!ret && repair) {
		/* Validate and preserve every byte before enabling programming. */
		if (!touch->flash_dump_valid || touch->flash_id_tail != 0x12 ||
		    memcmp(touch->flash_first, fw->data, sizeof(touch->flash_first))) {
			ret = -ENODEV;
		} else {
			memcpy(touch->flash_backup, touch->flash_dump, AXS_S17_FW_SIZE);
			touch->flash_backup_valid = true;
			ret = axs_flash_repair_allowed(touch->flash_backup, fw->data,
						       AXS_S17_FW_SIZE);
		}
		if (!ret)
			ret = s17_axs15260_flash_repair_session(axs_pram_write,
				axs_flash_repair, axs_read_current_version, touch);
	}
	touch->pram_fw = NULL;
	touch->pram_packet = NULL;
	mutex_unlock(&touch->report_lock);
	enable_irq(touch->client->irq);
unlock_state:
	mutex_unlock(&touch->state_lock);
out:
	WRITE_ONCE(touch->fw_result, ret);
	WRITE_ONCE(touch->fw_stage, ret ? AXS_FW_FAILED : AXS_FW_DONE);
	WRITE_ONCE(touch->fw_busy, false);
	dev_info(dev, "firmware %s result=%d pages=%u version=%04x\n",
		 repair ? "repair" : probe ? "probe" : "PRAM",
		 ret, touch->fw_pages, touch->fw_version);
	kfree(packet);
	release_firmware(fw);
	mutex_unlock(&touch->debug_lock);
	return ret ? ret : count;
}

static ssize_t firmware_pram_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (!sysfs_streq(buf, "load"))
		return -EINVAL;
	return axs_firmware_run(dev, false, false, count);
}
static DEVICE_ATTR_WO(firmware_pram);

static ssize_t firmware_probe_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	return axs_firmware_run(dev, true, false, count);
}
static DEVICE_ATTR_WO(firmware_probe);

static ssize_t firmware_repair_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	if (!sysfs_streq(buf, "repair-stock-0007"))
		return -EINVAL;
	return axs_firmware_run(dev, false, true, count);
}
static DEVICE_ATTR_WO(firmware_repair);

static ssize_t poll_once_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&touch->state_lock);
	if (touch->fb_suspended || touch->system_suspended) {
		mutex_unlock(&touch->state_lock);
		return -EBUSY;
	}
	mutex_lock(&touch->report_lock);
	touch->manual_reports++;
	axs_read_report(touch, 'M');
	ret = touch->last_read;
	mutex_unlock(&touch->report_lock);
	mutex_unlock(&touch->state_lock);
	return ret == 1 ? count : (ret < 0 ? ret : -EIO);
}
static DEVICE_ATTR_WO(poll_once);

static ssize_t report_history_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int i, start;
	ssize_t len = 0;

	mutex_lock(&touch->report_lock);
	start = (touch->history_next + AXS_DEBUG_HISTORY -
		 touch->history_count) % AXS_DEBUG_HISTORY;
	for (i = 0; i < touch->history_count; i++) {
		struct axs_debug_report *record =
			&touch->history[(start + i) % AXS_DEBUG_HISTORY];

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"ns=%llu source=%c read=%d decode=%d raw=%*ph\n",
			(unsigned long long)record->time_ns, record->source,
			record->read_result, record->decode_result,
			AXS_REPORT_SIZE, record->raw);
	}
	mutex_unlock(&touch->report_lock);
	return len;
}
static DEVICE_ATTR_RO(report_history);

static ssize_t debug_reports_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	bool enabled;

	mutex_lock(&touch->report_lock);
	enabled = touch->debug_reports;
	mutex_unlock(&touch->report_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t debug_reports_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	bool enabled;
	int ret = kstrtobool(buf, &enabled);

	if (ret)
		return ret;
	mutex_lock(&touch->report_lock);
	touch->debug_reports = enabled;
	mutex_unlock(&touch->report_lock);
	return count;
}
static DEVICE_ATTR_RW(debug_reports);

static void axs_poll_work(struct work_struct *work)
{
	struct axs_touch *touch = container_of(to_delayed_work(work),
					     struct axs_touch, poll_work);

	mutex_lock(&touch->state_lock);
	if (touch->poll_ms && time_before(jiffies, touch->poll_deadline)) {
		if (!touch->fb_suspended && !touch->system_suspended) {
			mutex_lock(&touch->report_lock);
			touch->poll_reports++;
			axs_read_report(touch, 'P');
			mutex_unlock(&touch->report_lock);
		}
		schedule_delayed_work(&touch->poll_work,
				      msecs_to_jiffies(touch->poll_ms));
	} else {
		touch->poll_ms = 0;
	}
	mutex_unlock(&touch->state_lock);
}

static ssize_t debug_poll_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int interval;

	mutex_lock(&touch->state_lock);
	interval = touch->poll_ms;
	mutex_unlock(&touch->state_lock);
	return scnprintf(buf, PAGE_SIZE, "%u\n", interval);
}

static ssize_t debug_poll_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int interval;
	int ret = kstrtouint(buf, 0, &interval);

	if (ret)
		return ret;
	if (interval && (interval < 20 || interval > 1000))
		return -EINVAL;
	mutex_lock(&touch->debug_lock);
	/* The worker takes state_lock; never wait for it while holding that lock. */
	cancel_delayed_work_sync(&touch->poll_work);
	mutex_lock(&touch->state_lock);
	touch->poll_ms = interval;
	touch->poll_deadline = jiffies + AXS_DEBUG_POLL_SECONDS * HZ;
	if (interval)
		schedule_delayed_work(&touch->poll_work, 0);
	mutex_unlock(&touch->state_lock);
	mutex_unlock(&touch->debug_lock);
	return count;
}
static DEVICE_ATTR_RW(debug_poll_ms);

static ssize_t irq_trigger_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 irq_get_trigger_type(touch->client->irq));
}

static ssize_t irq_trigger_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int type;
	int ret;

	if (sysfs_streq(buf, "falling"))
		type = IRQ_TYPE_EDGE_FALLING;
	else if (sysfs_streq(buf, "rising"))
		type = IRQ_TYPE_EDGE_RISING;
	else if (sysfs_streq(buf, "both"))
		type = IRQ_TYPE_EDGE_BOTH;
	else
		return -EINVAL;
	mutex_lock(&touch->state_lock);
	disable_irq(touch->client->irq);
	ret = irq_set_irq_type(touch->client->irq, type);
	enable_irq(touch->client->irq);
	mutex_unlock(&touch->state_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(irq_trigger);

static ssize_t pin_profile_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t len;

	mutex_lock(&touch->state_lock);
	len = scnprintf(buf, PAGE_SIZE, "%s\n", touch->pin_profile);
	mutex_unlock(&touch->state_lock);
	return len;
}

static ssize_t pin_profile_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));
	struct pinctrl_state *state;
	const char *name;
	int ret;

	if (sysfs_streq(buf, "boot-default")) {
		state = touch->boot_default;
		name = "boot-default";
	} else if (sysfs_streq(buf, "pull-up")) {
		state = touch->debug_pull_up;
		name = "pull-up";
	} else if (sysfs_streq(buf, "no-pull")) {
		state = touch->debug_no_pull;
		name = "no-pull";
	} else {
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(touch->pinctrl) || IS_ERR_OR_NULL(state))
		return -EOPNOTSUPP;
	mutex_lock(&touch->state_lock);
	disable_irq(touch->client->irq);
	ret = pinctrl_select_state(touch->pinctrl, state);
	if (!ret)
		touch->pin_profile = name;
	enable_irq(touch->client->irq);
	mutex_unlock(&touch->state_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(pin_profile);

static struct attribute *axs_attrs[] = {
	&dev_attr_diagnostics.attr,
	&dev_attr_live_version.attr,
	&dev_attr_supply_state.attr,
	&dev_attr_reset_touch.attr,
	&dev_attr_firmware_status.attr,
	&dev_attr_flash_status.attr,
	&dev_attr_firmware_probe.attr,
	&dev_attr_firmware_repair.attr,
	&dev_attr_flash_regions.attr,
	&dev_attr_firmware_pram.attr,
	&dev_attr_poll_once.attr,
	&dev_attr_report_history.attr,
	&dev_attr_debug_reports.attr,
	&dev_attr_debug_poll_ms.attr,
	&dev_attr_irq_trigger.attr,
	&dev_attr_pin_profile.attr,
	NULL,
};
static const struct attribute_group axs_attr_group = {
	.attrs = axs_attrs,
	.bin_attrs = axs_bin_attrs,
};

/* Callers serialize with state_lock; IRQ never takes it. */
static void axs_update_irq(struct axs_touch *touch)
{
	bool disable = touch->fb_suspended || touch->system_suspended;

	if (disable == touch->irq_disabled)
		return;
	if (disable) {
		disable_irq(touch->client->irq);
		mutex_lock(&touch->report_lock);
		axs_release(touch);
		mutex_unlock(&touch->report_lock);
	} else {
		enable_irq(touch->client->irq);
	}
	touch->irq_disabled = disable;
	dev_info(&touch->client->dev, "IRQ %s: fb=%d system=%d\n",
		 disable ? "paused" : "enabled", touch->fb_suspended,
		 touch->system_suspended);
}

static int axs_fb_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct axs_touch *touch = container_of(nb, struct axs_touch, fb_notifier);
	struct fb_event *fb_event = data;
	int blank;

	if (!fb_event || !fb_event->data)
		return NOTIFY_DONE;
	blank = *(int *)fb_event->data;
	if (event == FB_EARLY_EVENT_BLANK && blank == FB_BLANK_POWERDOWN) {
		mutex_lock(&touch->state_lock);
		touch->fb_suspended = true;
		axs_update_irq(touch);
		mutex_unlock(&touch->state_lock);
	} else if (event == FB_EVENT_BLANK && blank == FB_BLANK_UNBLANK) {
		mutex_lock(&touch->state_lock);
		touch->fb_suspended = false;
		axs_update_irq(touch);
		mutex_unlock(&touch->state_lock);
	}
	return NOTIFY_OK;
}

static int axs_suspend(struct device *dev)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&touch->state_lock);
	touch->system_suspended = true;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static int axs_resume(struct device *dev)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&touch->state_lock);
	touch->system_suspended = false;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(axs_pm_ops, axs_suspend, axs_resume);

static void axs_disable_supply(void *data)
{
	regulator_disable(data);
}

static void axs_remove_irq(void *data)
{
	struct axs_touch *touch = data;

	devm_free_irq(&touch->client->dev, touch->client->irq, touch);
}

static void axs_stop_poll_work(void *data)
{
	struct axs_touch *touch = data;

	cancel_delayed_work_sync(&touch->poll_work);
}

static int axs_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct axs_touch *touch;
	struct regulator *supply;
	u8 cmd = 0x0c, version[2];
	struct i2c_msg version_msgs[] = {
		{ .addr = client->addr, .len = 1, .buf = &cmd },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = 2, .buf = version },
	};
	int ret;

	if (client->addr != 0x3b || client->irq <= 0)
		return -EINVAL;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	touch = devm_kzalloc(&client->dev, sizeof(*touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;
	touch->client = client;
	mutex_init(&touch->state_lock);
	mutex_init(&touch->report_lock);
	mutex_init(&touch->debug_lock);
	INIT_DELAYED_WORK(&touch->poll_work, axs_poll_work);
	touch->pin_profile = "boot-default";
	touch->pinctrl = devm_pinctrl_get(&client->dev);
	if (!IS_ERR(touch->pinctrl)) {
		touch->boot_default = pinctrl_lookup_state(touch->pinctrl,
							"default");
		touch->debug_pull_up = pinctrl_lookup_state(touch->pinctrl,
							 "debug-pull-up");
		touch->debug_no_pull = pinctrl_lookup_state(touch->pinctrl,
							 "debug-no-pull");
	}
	touch->last_read = -ENODATA;
	touch->last_decode = -ENODATA;
	i2c_set_clientdata(client, touch);
	touch->flash_first_mismatch = -1;
	/* Optional debug snapshots; failed allocation only disables firmware tests. */
	touch->flash_dump = devm_kzalloc(&client->dev, AXS_S17_FW_SIZE, GFP_KERNEL);
	touch->flash_backup = devm_kzalloc(&client->dev, AXS_S17_FW_SIZE, GFP_KERNEL);
	supply = devm_regulator_get(&client->dev, "vtouch");
	if (IS_ERR(supply))
		return PTR_ERR(supply);
	touch->supply = supply;
	ret = regulator_enable(supply);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&client->dev, axs_disable_supply, supply);
	if (ret)
		return ret;

	/* Stock touch probe resets independently of the LK/LCM display handoff. */
	msleep(100);
	ret = s17_axs15260_reset_touch(20);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;
	if (ret)
		return ret;
	touch->reset_requests = 1;
	touch->last_reset_ns = ktime_get_ns();
	touch->last_reset_result = ret;
	dev_info(&client->dev, "stock probe touch reset completed\n");

	/* Read-only startup: log existing firmware; leave flash writes for later. */
	ret = i2c_transfer(client->adapter, version_msgs, ARRAY_SIZE(version_msgs));
	touch->firmware_read = ret;
	if (ret == ARRAY_SIZE(version_msgs))
		memcpy(touch->firmware_version, version, sizeof(version));
	if (ret == ARRAY_SIZE(version_msgs))
		dev_info(&client->dev, "existing firmware version %02x%02x\n",
			 version[0], version[1]);
	else
		dev_info(&client->dev, "firmware version not ready: %d\n", ret);

	touch->input = devm_input_allocate_device(&client->dev);
	if (!touch->input)
		return -ENOMEM;
	touch->input->name = "axs_ts";
	touch->input->id.bustype = BUS_I2C;
	input_set_capability(touch->input, EV_KEY, BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, touch->input->propbit);
	input_set_abs_params(touch->input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_POSITION_X, 0, 384, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_POSITION_Y, 0, 854, 0, 0);
	touchscreen_parse_properties(touch->input, true, &touch->props);
	ret = input_register_device(touch->input);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					axs_irq_thread, IRQF_ONESHOT, "axs_ts", touch);
	if (ret)
		return ret;
	/* Release IRQ before the managed input device on probe failure/removal. */
	ret = devm_add_action_or_reset(&client->dev, axs_remove_irq, touch);
	if (ret)
		return ret;
	/* Stop asynchronous diagnostics before releasing IRQ/input resources. */
	ret = devm_add_action_or_reset(&client->dev, axs_stop_poll_work, touch);
	if (ret)
		return ret;
	touch->fb_notifier.notifier_call = axs_fb_event;
	ret = fb_register_client(&touch->fb_notifier);
	if (ret)
		return ret;
	ret = devm_device_add_group(&client->dev, &axs_attr_group);
	if (ret) {
		fb_unregister_client(&touch->fb_notifier);
		return ret;
	}
	dev_info(&client->dev, "basic reports ready: addr=0x3b irq=%d\n", client->irq);
	return 0;
}

static int axs_remove(struct i2c_client *client)
{
	struct axs_touch *touch = i2c_get_clientdata(client);

	devm_device_remove_group(&client->dev, &axs_attr_group);
	cancel_delayed_work_sync(&touch->poll_work);
	fb_unregister_client(&touch->fb_notifier);
	mutex_lock(&touch->state_lock);
	touch->system_suspended = true;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static void axs_shutdown(struct i2c_client *client)
{
	struct axs_touch *touch = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&touch->poll_work);
	mutex_lock(&touch->state_lock);
	touch->system_suspended = true;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
}

static const struct of_device_id axs_of_match[] = {
	{ .compatible = "lvmi,s17promini-axs15260" },
	{ }
};
MODULE_DEVICE_TABLE(of, axs_of_match);

static const struct i2c_device_id axs_ids[] = {
	{ "axs15260-s17", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axs_ids);

static struct i2c_driver axs_driver = {
	.driver = {
		.name = "axs15260-s17",
		.of_match_table = axs_of_match,
		.pm = &axs_pm_ops,
	},
	.probe = axs_probe,
	.remove = axs_remove,
	.shutdown = axs_shutdown,
	.id_table = axs_ids,
};
module_i2c_driver(axs_driver);

MODULE_DESCRIPTION("S17Promini AXS15260 basic touch reports");
MODULE_LICENSE("GPL");
