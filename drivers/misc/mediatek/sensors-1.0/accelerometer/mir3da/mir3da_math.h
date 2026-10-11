/* SPDX-License-Identifier: GPL-2.0 */
/* S17 stock Image 0x632de0/0x634030/0x6316d8, reconstructed offline. */
#ifndef S17_MIR3DA_MATH_H
#define S17_MIR3DA_MATH_H

struct mir3da_calibration {
	int z_offset;
	bool history_valid;
	s16 history[3][30];
};

static bool mir3da_static_sample(struct mir3da_calibration *c, int v[3])
{
	int axis, i, lo, hi, spread = 0;

	for (axis = 0; axis < 3; axis++) {
		if (!c->history_valid) {
			for (i = 0; i < 30; i++)
				c->history[axis][i] = v[axis];
		} else {
			lo = c->history[axis][0];
			hi = lo;
			for (i = 1; i < 30; i++) {
				lo = min_t(int, lo, c->history[axis][i]);
				hi = max_t(int, hi, c->history[axis][i]);
				c->history[axis][i - 1] = c->history[axis][i];
			}
			c->history[axis][29] = v[axis];
			spread += hi - lo;
		}
	}
	c->history_valid = true;
	return spread < 60;
}

static int mir3da_signed(int magnitude, int sign)
{
	return sign < 0 ? -magnitude : magnitude;
}

static void mir3da_temperature_compensate(struct mir3da_calibration *c,
					 int v[3])
{
	int x = v[0], y = v[1], z = v[2] + c->z_offset;
	int ax = abs(x), ay = abs(y), remainder = z % 10;
	int square = 1024 * 1024 - x * x - y * y;
	int target = mir3da_signed((square < 0 ? 0 : int_sqrt(square)) +
				  remainder, z);
	int sample[3] = { x, y, z - c->z_offset };
	bool stable;

	if (ax < 200 && ay < 200) {
		stable = mir3da_static_sample(c, sample);
		if (!c->z_offset) {
			if (stable)
				c->z_offset = target - z;
			z = target;
		} else if (stable) {
			if (abs(abs(z) - 1024) > 130)
				c->z_offset = 0;
			z = target;
		}
		x = x * 130 / 200;
		y = y * 130 / 200;
	} else if (abs(ax - 1024) < 200 && c->z_offset && ay < 200) {
		x += mir3da_signed((1024 - ax) * 70 / 200, x);
		y = y * 130 / 200;
	} else if (ax < 200 && c->z_offset && abs(ay - 1024) < 200) {
		y += mir3da_signed((1024 - ay) * 70 / 200, y);
		x = x * 130 / 200;
	} else if (!c->z_offset) {
		if (ax < 200 && abs(ay - 1024) < 200)
			z = mir3da_signed(ax * 130 / 200, z);
		else if (ay < 200 && abs(ax - 1024) < 200)
			z = mir3da_signed(ay * 130 / 200, z);
		else
			z = target;
	}
	v[0] = x;
	v[1] = y;
	v[2] = z;
}

static int mir3da_proximity_sample(int raw[3], unsigned int *count)
{
	int px = raw[0] * 9807, py = raw[1] * 9807, pz = raw[2] * 9807;
	int ax = abs(px / 1024), ay = abs(py / 1024);
	bool near = ax > 7500 || (pz < -1023 && ay > 7000) ||
		(ax > 5000 && ay > 5000) ||
		(((u32)(py - 0x4e2400) >> 14) < 125 &&
		 (u32)(pz + 0x4e1fff) < 0x4e1c00);

	if (!near)
		*count = 0;
	else if (*count < 4)
		(*count)++;
	return *count >= 4 ? 0 : 1;
}
#endif
