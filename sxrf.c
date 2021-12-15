/******************************************************************************
 * @file sxrf.c
 * @author Nikos Tsiligaridis
 * @brief C library for Semtech SX127x, RFM9x
 * @version 0.1.0
 * @date 2021-11-21
 * 
 * @copyright Copyright (c) 2021
******************************************************************************/
#include <inttypes.h>
#include "sxrf.h"
#include "esp_log.h"
#include <math.h>

/** CHeck if expr() returns SXRF_OK, else return ret. Used in functions where multiple
 * registers are written with write_verify and any single failure must cause abort. */
#define ASSERT_RET_ERROR(ret, expr) do { \
	if((ret = expr) != SXRF_OK) return ret; } \
while(0) \

// Temp
#include <stdio.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/** Masks for reading/writing from various multifunction registers */
const u8
	SXRF_MODULATION_MASK = 0b01100000,

	SXRF_OP_MODE_MASK = 0b00000111,

	SXRF_PACKET_FORMAT_MASK = 0b10000000,
	SXRF_CRC_ON_MASK = 0b00010000,

	SXRF_SYNC_WORD_ON_MASK = 0b00010000,
	SXRF_ADDR_FILTERING_MASK = 0b00000110,

	SXRF_DIO0_MAPPING_MASK = 0b11000000,

	SXRF_IRQ_FLAGS2_FIFO_OVERRUN_MASK = 0b00010000
;

/** Bits for multifunction registers */
const u8
	// Sync word generation detection on true/false
	SXRF_SYNC_CONFIG_ON_TRUE = SXRF_SYNC_WORD_ON_MASK,
	SXRF_SYNC_CONFIG_ON_FALSE = 0,

	// CrcOn true/false
	SXRF_PACKET_CONFIG_CRC_ON_TRUE = SXRF_CRC_ON_MASK,
	SXRF_PACKET_CONFIG_CRC_ON_FALSE = 0

	// Packet format
	
;

/** SX127x OSC freq */
const u32 SXRF_FOSC = 32e6;

/** Freq step = Fxosc / 2^19 */
const float SXRF_F_STEP = ((float)SXRF_FOSC / (1 << 19));

/** Min allowed freq deviation */
const u32 SXRF_FREQ_DEV_MIN = 600;

/** Max allowed freq deviation */
const u32 SXRF_FREQ_DEV_MAX = 200e3;

/** Time to wait after a write operation to verify written value */
const u16 SXRF_WRITE_VERIFY_TIMEOUT_MS = 50;

const u16 SXRF_WRITE_VERIFY_RETRY_INT_MS = 10;

/******************************************************************************
* Registers
******************************************************************************/
const uint8_t
	// Operation mode
	REG_OP_MODE = 0x01,

	// Bitrate
	REG_BITRATE_MSB = 0x02,
	REG_BITRATE_LSB = 0x03,

	// Freq dev.
	REG_FDEV_MSB = 0x04,
	REG_FDEV_LSB = 0x05,

	// Carrier frequency
	REG_FRF_MSB = 0x06,
	REG_FRF_MID = 0x07,
	REG_FRF_LSB = 0x08,

	// RSSI
	REG_RSSI_VALUE = 0x11,

	// RX bandwidth and AFC
	REG_RX_BW = 0x12,
	REG_RX_BW_AFC = 0x13,

	REG_PREAMBLE_MSB = 0x25,
	REG_PREAMBLE_LSB = 0x26,

	REG_SYNC_CONFIG = 0x27,

	// Sync word reg 1 of 8. The rest 7 are addressed
	// sequentially
	REG_SYNC_VAL1 = 0x28,

	REG_PACKET_CONFIG1 = 0x30,

	// Node address
	REG_NODE_ADDR = 0x33,

	// Node address
	REG_SYNC_WORD = 0x39,

	// DIOx IRQ FLags
	REG_IRQ_FLAGS1 = 0x3e,
	REG_IRQ_FLAGS2 = 0x3f,

	// DIOx Mapping
	REG_DIO_MAPPING1 = 0x40,

	// Silion version
	REG_VERSION = 0x42,

	// Bitrate fractional part
	REG_BITRATE_FRAC = 0x5D
;

///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////
u8 sxrf_get_reg_by_irq_flag(SXRF_IRQ_FLAG flag);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
* Get node address
* @return Node address
******************************************************************************/
u8 sxrf_get_node_addr()
{
	return sxrf_read_byte(REG_NODE_ADDR);;
}

/******************************************************************************
* Set node address
* @param addr Node address
* @return SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_set_node_addr(u8 addr)
{
	return sxrf_write_byte_verify(REG_NODE_ADDR, addr);
}

/******************************************************************************
* Set address filtering. When enabled packets without matching node addr or 
* broadcast addr. are discarded.
* @param addr Addr. filtering configuration
* @return SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_set_addr_filtering(SXRF_ADDR_FILTERING_MODE mode)
{
	return sxrf_set_reg_bits(REG_PACKET_CONFIG1, SXRF_ADDR_FILTERING_MASK, mode);;
}

/******************************************************************************
* Get preamble size
* @return Preamble size (bytes)
******************************************************************************/
u16 sxrf_get_preamble_size()
{
	u16 reg_val = sxrf_read_byte(REG_PREAMBLE_MSB) << 8;
	reg_val |= sxrf_read_byte(REG_PREAMBLE_LSB);

	return reg_val;
}

/******************************************************************************
* Set preamble size
* @param size Preamble size (bytes)
* @return SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_set_preamble_size(u16 size)
{
	SXRF_RET ret = SXRF_OK;
	
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_PREAMBLE_MSB, size >> 8));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_PREAMBLE_LSB, size));

	return SXRF_OK;
}

/******************************************************************************
* Get sync word. Sync words are 8 bytes long max.
* @param buff		Output buffer
* @param buff_size	Output buffer size
* @return Sync word size as configured in config reg
******************************************************************************/
u8 sxrf_get_sync_word(u8 *buff, u8 buff_size)
{
	u8 sw_size = sxrf_read_byte(REG_SYNC_CONFIG) & 0b111;
	sw_size++;	

	sxrf_read(REG_SYNC_VAL1, buff, sw_size);

	return sw_size;
}

/******************************************************************************
* Set sync word and sync word size. Max 8 bytes long.
* @param addr Sync word
* @return
*	SXRF_OK on success
*	SXRF_INVALID_INPUT when invalid params provided
******************************************************************************/
SXRF_RET sxrf_set_sync_word(const u8 *sync_word_buff, u8 size)
{
	// Min/max lengths
	if(size < 1 || size > 8)
	{
		return SXRF_ERR_INVALID_INPUT;
	}
	// Cant contain 0x00
	for (size_t i = 0; i < size; i++)
	{
		if(sync_word_buff[i] == 0x00)
		{
			return SXRF_ERR_INVALID_INPUT;
		}
	}	

	SXRF_RET ret = SXRF_OK;

	// Update size in config reg
	// Size is stored as size - 1
	u8 reg_conf = sxrf_read_byte(REG_SYNC_CONFIG);
	reg_conf = (reg_conf & ~0b111) | (size - 1);
	
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_SYNC_CONFIG, reg_conf));

	// Write sync word
	// TODO: Needs verify function
	sxrf_write(REG_SYNC_VAL1, sync_word_buff, size);

	// Set sync ON
	ASSERT_RET_ERROR(ret, sxrf_set_reg_bits(REG_SYNC_CONFIG, SXRF_SYNC_WORD_ON_MASK, SXRF_SYNC_CONFIG_ON_TRUE));
	
	return SXRF_OK;
}

/******************************************************************************
* Get modulation
******************************************************************************/
SXRF_MODULATION sxrf_get_modulation()
{
	SXRF_MODULATION mod = sxrf_read_byte(REG_OP_MODE) & SXRF_MODULATION_MASK;

	return mod;
}

/******************************************************************************
* Set RF modulation
* @param Modulation (OOK/FSK)
* @return SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_set_modulation(SXRF_MODULATION mod)
{
	// Get cur val
	u8 val = sxrf_read_byte(REG_OP_MODE);

	switch (mod)
	{
	case SXRF_MODULATION_FSK:
		val = (val & ~SXRF_MODULATION_MASK) | SXRF_MODULATION_FSK;
		break;
	case SXRF_MODULATION_OOK:
		val = (val & ~SXRF_MODULATION_MASK) | SXRF_MODULATION_OOK;
		break;
	default:
		return SXRF_ERR_INVALID_INPUT;
	}

	return sxrf_write_byte_verify(REG_OP_MODE, val);
}

/******************************************************************************
* Get carrier frequency
* @return Carrier frequency in Hz
******************************************************************************/
u32 sxrf_get_freq()
{
	u32 freq = 0;

	freq = sxrf_read_byte(REG_FRF_MSB) << 16;
	freq |= sxrf_read_byte(REG_FRF_MID) << 8;
	freq |= sxrf_read_byte(REG_FRF_LSB);

	return freq * SXRF_F_STEP;
}

/******************************************************************************
* Set carrier frequency
* @param Frequency in Hz
* @return 
*	SXRF_OK on success
*	SXRF_ERR_INVALID_INPUT when given frequency unsupported
******************************************************************************/
SXRF_RET sxrf_set_freq(u32 freq)
{
	if(!
		(freq >= 862e6 && freq <= 1020e6) ||
		(freq >= 410e6 && freq <= 525e6) ||
		(freq >= 137e6 && freq <= 175e6)
	)
	{
		return SXRF_ERR_INVALID_INPUT;
	}

	u32 frf = freq / SXRF_F_STEP;

	SXRF_RET ret = SXRF_OK;
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_FRF_MSB, frf >> 16));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_FRF_MID, frf >> 8));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_FRF_LSB, frf));

	return SXRF_OK;
}

/******************************************************************************
* Get bitrate
******************************************************************************/
float sxrf_get_bitrate()
{
	u32 reg_whole = 0;

	reg_whole = sxrf_read_byte(REG_BITRATE_MSB) << 8;
	reg_whole |= sxrf_read_byte(REG_BITRATE_LSB);

	u16 reg_frac = sxrf_read_byte(REG_BITRATE_FRAC);

	return SXRF_FOSC / (reg_whole + ((float)reg_frac / 16));
}

/******************************************************************************
* Set bitrate (will be approximated)
* @param Bitrate in bps
******************************************************************************/
SXRF_RET sxrf_set_bitrate(float bitrate)
{
	// TODO: Check ranges fro FSK and OOK mode separately
	float reg = SXRF_FOSC / bitrate;

	// Whole part of register value will go to bitrate regs
	u16 reg_whole = (int)reg;

	// Fraction part goes to frac reg
	u8 frac = 16 * (reg - (int)reg);

	SXRF_RET ret = SXRF_OK;
	
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_BITRATE_MSB, reg_whole >> 8));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_BITRATE_LSB, reg_whole));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_BITRATE_FRAC, frac));

	return SXRF_OK;
}

/******************************************************************************
* Get frequency deviation
******************************************************************************/
int sxrf_get_freq_dev()
{
	float dev = 0;

	// 2 MSbs of MSB are reserved, mask them out
	dev = (0x3F & sxrf_read_byte(REG_FDEV_MSB)) << 8;
	dev = (u32)dev | sxrf_read_byte(REG_FDEV_LSB);
	dev *= SXRF_F_STEP;

	return dev;
}

/******************************************************************************
* Set frequency deviation in Hz
* @param Frequency deviation (0.6 - 200KHz)
* @return
*	SXRF_OK on success
*	SXRF_ERR_INVALID_INPUT when given value out of range
******************************************************************************/
SXRF_RET sxrf_set_freq_dev(int dev)
{
	float bitrate = sxrf_get_bitrate();

	// Validate input
	if(dev + bitrate / 2 > 250e3)
		return SXRF_ERR_INVALID_INPUT;
	if(dev < SXRF_FREQ_DEV_MIN || dev > SXRF_FREQ_DEV_MAX)
		return SXRF_ERR_INVALID_INPUT;
	
	// Calc and round
	u16 reg = (dev / SXRF_F_STEP) + 0.5;

	// 2 MSbs of MSB are reserved, mask them out
	SXRF_RET ret = SXRF_OK;
	
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_FDEV_MSB, reg >> 8));
	ASSERT_RET_ERROR(ret, sxrf_write_byte_verify(REG_FDEV_LSB, reg));

	return SXRF_OK;
}

/******************************************************************************
* Get RX bandwidth (Hz)
******************************************************************************/
int sxrf_get_rx_bw()
{
	// Mantissa/exponent
	u8 m = 0, e = 0;

	u8 reg = sxrf_read_byte(REG_RX_BW);

	m = reg & 0b00011000;
	m = (m >> 1) | 16;
	e = reg & 0b00000111;

	int bw = (float)SXRF_FOSC / (float)(m * (2 << (e + 1)));

	return bw;
}

/******************************************************************************
* Set RX bandwidth. Must be one of supported values (see datasheet)
* @param bw Rx BW in HZ
* @return
*	SXRF_OK on success
*	SXRF_ERR_INVALID_INPUT when given bw not supported
******************************************************************************/
SXRF_RET sxrf_set_rx_bw(int bw)
{
	bool found = false;
	
	// Mantissa/exponent
	u8 m = 0, e = 0;

	// Walk through suported values and find values for mantissa and exp regs
	// Allow for 1% in user input and snap to nearest
	for (e = 1; e <= 7; ++e)
    {
        for (m = 16; m <= 24; m += 4)
        {
            float calced_bw = SXRF_FOSC / (m * (2 << (e + 1)));
            if(abs(calced_bw - bw) <= calced_bw * 0.01)
			{
				found = true;

                break;
			}
        }

		if(found) break;
    }

	if(found)
	{
		// Update rxbw register
		u8 cur_val = sxrf_read_byte(REG_RX_BW);

		cur_val = (cur_val & 0b11100111) | (((m & 1100) << 1));
		cur_val = (cur_val & 0b11111000) | e;
		
		return sxrf_write_byte_verify(REG_RX_BW, cur_val);
	}

	return SXRF_ERR_INVALID_INPUT;
}

/******************************************************************************
* Get operation mode
* @return Mode
******************************************************************************/
SXRF_OP_MODE sxrf_get_op_mode()
{
	u8 cur_val = sxrf_read_byte(REG_OP_MODE) & 0b111;

	return cur_val;
}

/******************************************************************************
* Set operation mode
* @return RET_OK on success
******************************************************************************/
SXRF_RET sxrf_set_op_mode(SXRF_OP_MODE mode)
{
	u8 reg_val = sxrf_read_byte(REG_OP_MODE);

	reg_val = (reg_val & ~SXRF_OP_MODE_MASK) | (mode & SXRF_OP_MODE_MASK);

	

	return sxrf_write_byte_verify(REG_OP_MODE, reg_val);;
}


/******************************************************************************
* Set DIO0 mapping
* @return RET_OK on success
******************************************************************************/
SXRF_RET sxrf_set_dio0_mapping(SXRF_DIO0_MAPPING map)
{
	return sxrf_set_reg_bits(REG_DIO_MAPPING1, SXRF_DIO0_MAPPING_MASK, map);
}

/******************************************************************************
* Check if IRQ flag set
* @return IRQ flag value
******************************************************************************/
bool sxrf_is_irq_flag_set(SXRF_IRQ_FLAG flag)
{
	u8 addr = sxrf_get_reg_by_irq_flag(flag);
	u8 mask = 0;

	// Invalid input
	if(addr == 0)
		return 0;

	mask = addr == REG_IRQ_FLAGS1 ? (1 << flag) : (1 << (flag - 8));

	return sxrf_read_byte(addr) & mask;
}

/******************************************************************************
* Clear IRQ flag
* @return
*	SXRF_OK on success
*	SXRF_ERR_INVALID_INPUT on invalid flag
******************************************************************************/
SXRF_RET sxrf_clear_irq_flag(SXRF_IRQ_FLAG flag)
{
	u8 addr = sxrf_get_reg_by_irq_flag(flag);
	u8 mask = 0;

	if(addr == 0)
		return SXRF_ERR_INVALID_INPUT;

	mask = addr == REG_IRQ_FLAGS1 ? (1 << flag) : (1 << (flag - 8));

	// Flags cleared by setting to 1
	return addr != 0 ? sxrf_set_reg_bits(addr, mask, mask) : 0;
}

/******************************************************************************
* Get reg to which the provided IRQ flag belongs. Helper function.
* @return RegIrqFlags1/2 address
******************************************************************************/
u8 sxrf_get_reg_by_irq_flag(SXRF_IRQ_FLAG flag)
{
	switch (flag)
	{
	case SXRF_IRQ_MODE_READY:
    case SXRF_IRQ_RX_READY:
    case SXRF_IRQ_TX_READY:
    case SXRF_IRQ_PLL_LOCK:
    case SXRF_IRQ_RSSI:
    case SXRF_IRQ_TIMEOUT:
    case SXRF_IRQ_PREAMBLE_DETECT:
    case SXRF_IRQ_SYNC_ADDRESS_MATCH:
		return REG_IRQ_FLAGS1;
	case SXRF_IRQ_FIFO_FULL: 
	case SXRF_IRQ_FIFO_EMPTY: 
	case SXRF_IRQ_FIFO_LEVEL: 
	case SXRF_IRQ_FIFO_OVERRUN: 
	case SXRF_IRQ_PACKET_SENT: 
	case SXRF_IRQ_PAYLOAD_READY: 
	case SXRF_IRQ_CRC_OK:
	case SXRF_IRQ_LOW_BAT:
		return REG_IRQ_FLAGS2;
	default:
		return 0;
	}
}

/******************************************************************************
* Set packet format
* @return RET_OK on success
******************************************************************************/
SXRF_RET sxrf_set_packet_format(SXRF_PACKET_FORMAT format)
{
	return sxrf_set_reg_bits(REG_PACKET_CONFIG1, SXRF_PACKET_FORMAT_MASK, (u8)format);
}

/******************************************************************************
* Set CRC on
* @return RET_OK on success
******************************************************************************/
SXRF_RET sxrf_set_crc_on(bool on)
{
	return sxrf_set_reg_bits(REG_PACKET_CONFIG1, SXRF_CRC_ON_MASK,
		on ? SXRF_PACKET_CONFIG_CRC_ON_TRUE : SXRF_PACKET_CONFIG_CRC_ON_FALSE);
}

/******************************************************************************
* Read fifo up to buff_size bytes into buffer. FIFO is 256bytes long.
* @param buff Output buffer
* @param buff_size Output buffer size
* @return Bytes read
******************************************************************************/
u32 sxrf_read_fifo(u8 *buff, u32 buff_size)
{
	// First byte is length
	int length = sxrf_read_byte(0x00);

	if(length == 0)
		return length;

	// Read FIFO into buffer
	sxrf_read(0x00, buff, length);

	return length;
}

/******************************************************************************
* Clear FIFO.
* FIFO can be cleared by setting FifoOverrun IRQ flag
* @return
*	SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_clear_fifo()
{
	// After writing the flag is cleared automatically
	SXRF_RET ret = sxrf_set_reg_bits(REG_IRQ_FLAGS2, SXRF_IRQ_FLAGS2_FIFO_OVERRUN_MASK, 
		1 << (SXRF_IRQ_FIFO_OVERRUN - 8));

	return ret;
}

/******************************************************************************
* Get RSSI in dBm. RSSI is returned in 0.5dBm steps.
* @return RSSI in dBm
******************************************************************************/
float sxrf_get_rssi()
{
	return -(sxrf_read_byte(REG_RSSI_VALUE) / 2);
}

/******************************************************************************
* Get silicon version
* @return Version
******************************************************************************/
u8 sxrf_get_version()
{
	return sxrf_read_byte(REG_VERSION);
}

/******************************************************************************
* Read register, clear bits using mask, or bits using val
* @param address	Register address
* @param mask		Used to mask out the relevant bits
* @param val		Value to set to masked out bits
* @return SXRF_OK on success
******************************************************************************/
SXRF_RET sxrf_set_reg_bits(u8 addr, u8 mask, u8 val)
{
	u8 cur_val = sxrf_read_byte(addr) & ~mask;

	// Make sure provided val doesn't set other bits
	val = val & mask;
	
	cur_val |= val;

	return sxrf_write_byte_verify(addr, cur_val);;
}

/******************************************************************************
* Write byte to register. Uses provided HAL function.
* @param addr	Target address
******************************************************************************/
void sxrf_write_byte(const u8 addr, u8 val)
{
    sxrf_write(addr, &val, 1);
}

/******************************************************************************
* Read byte from register
* @param addr Register address
* @return Register value
******************************************************************************/
u8 sxrf_read_byte(const u8 addr)
{
    uint8_t res = 0;
    sxrf_read(addr,  &res, 1);
    
    return res;
}

/******************************************************************************
* Write single byte and read back to validate. If write fails after multiple
* attempts fail with timeout
* @param addr	Target addres
* @param val	Value
* @param
*	SXRF_OK on success
*	SXRF_TIMEOUT on timeout
******************************************************************************/
SXRF_RET sxrf_write_byte_verify(const u8 addr, u8 val)
{
	u32 start_tick = sxrf_get_tick_ms();
	bool success = false;

	sxrf_write_byte(addr, val);

	do
	{
		u8 read_back = sxrf_read_byte(addr);
		if(read_back == val)
		{
			success = true;
			break;
		}
		sxrf_wait_ms(SXRF_WRITE_VERIFY_RETRY_INT_MS);
	}while(sxrf_get_tick_ms() - start_tick < SXRF_WRITE_VERIFY_TIMEOUT_MS);
	
	return success ? SXRF_OK : SXRF_ERR_TIMEOUT;
}