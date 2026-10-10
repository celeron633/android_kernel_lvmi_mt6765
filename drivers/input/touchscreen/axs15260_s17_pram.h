/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _AXS15260_S17_PRAM_H
#define _AXS15260_S17_PRAM_H

#define AXS_S17_FW_SIZE 50192U
#define AXS_S17_FW_CRC32 0x1655298bU
#define AXS_S17_FW_VERSION 7U
#define AXS_S17_FW_VERSION_OFFSET 0x7634U
#define AXS_PRAM_PAGE_SIZE 256U
#define AXS_PRAM_HEADER_SIZE 11U
#define AXS_FLASH_HEADER_SIZE 12U

/* Stock Image 0x9fca64: size excludes the four-byte signature and
 * the two-byte length/two reserved bytes, and includes the 3-byte address.
 */
static inline unsigned int axs_pram_packet(u8 *packet, const u8 *data,
					 unsigned int offset, unsigned int len)
{
	unsigned int wire_len = len + 3;

	if (!len || len > AXS_PRAM_PAGE_SIZE ||
	    offset > AXS_S17_FW_SIZE || len > AXS_S17_FW_SIZE - offset ||
	    offset % AXS_PRAM_PAGE_SIZE)
		return 0;
	packet[0] = 0xab;
	packet[1] = 0xb5;
	packet[2] = 0xa5;
	packet[3] = 0x5a;
	packet[4] = wire_len >> 8;
	packet[5] = wire_len;
	packet[6] = 0;
	packet[7] = 0;
	packet[8] = offset >> 16;
	packet[9] = offset >> 8;
	packet[10] = offset;
	memcpy(packet + AXS_PRAM_HEADER_SIZE, data + offset, len);
	return AXS_PRAM_HEADER_SIZE + len;
}

/* Stock SPI page-program bridge: opcode 02 + three address bytes + payload. */
static inline unsigned int axs_flash_packet(u8 *packet, const u8 *data,
					  unsigned int offset, unsigned int len)
{
	unsigned int wire_len = len + 4;

	if (!len || len > AXS_PRAM_PAGE_SIZE ||
	    offset > AXS_S17_FW_SIZE || len > AXS_S17_FW_SIZE - offset ||
	    offset % AXS_PRAM_PAGE_SIZE)
		return 0;
	packet[0] = 0xab;
	packet[1] = 0xb5;
	packet[2] = 0xa5;
	packet[3] = 0x5a;
	packet[4] = wire_len >> 8;
	packet[5] = wire_len;
	packet[6] = 0;
	packet[7] = 0;
	packet[8] = 2;
	packet[9] = offset >> 16;
	packet[10] = offset >> 8;
	packet[11] = offset;
	memcpy(packet + AXS_FLASH_HEADER_SIZE, data + offset, len);
	return AXS_FLASH_HEADER_SIZE + len;
}

/* Guard before any write: every differing stored byte must still be erased. */
static inline int axs_flash_repair_allowed(const u8 *stored, const u8 *expected,
					   unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		if (stored[i] != expected[i] && stored[i] != 0xff)
			return -EUCLEAN;
	return 0;
}

#endif
