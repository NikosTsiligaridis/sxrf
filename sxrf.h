/******************************************************************************
 * @file sxrf.h
 * @author Nikos Tsiligaridis
 * @brief C library for Semtech SX127x, RFM9x
 * @version 0.1.0
 * @date 2021-11-21
 * 
 * @copyright Copyright (c) 2021
******************************************************************************/
#ifndef SXRF_H
#define SXRF_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/** Modulations */
typedef enum SXRF_MODULATION_T
{
    SXRF_MODULATION_UNKNOWN = -1,
    SXRF_MODULATION_FSK = 0b00000000,
    SXRF_MODULATION_OOK = 0b00100000,
} SXRF_MODULATION;

/** Operation modes */
typedef enum SXRF_OP_MODE_T
{
    SXRF_OP_MODE_SLEEP     = 0b000,
    SXRF_OP_MODE_STANDBY   = 0b001,
    SXRF_OP_MODE_FSTX      = 0b010,
    SXRF_OP_MODE_TX        = 0b011,
    SXRF_OP_MODE_FSRX      = 0b100,
    SXRF_OP_MODE_RX        = 0b101
} SXRF_OP_MODE;

/** Address filtering, RegPacketConfig1 reg */
typedef enum SXRF_ADDR_FILTERING_MODE_T
{
    SXRF_ADDR_FILTERING_NONE = 0,
    SXRF_ADDR_FILTERING_NODE_ADDR = 0b00000010,
    SXRF_ADDR_FILTERING_NODE_OR_BROADCAST_ADDR = 0b00000100
} SXRF_ADDR_FILTERING_MODE;

/** Packet formats */
typedef enum SXRF_PACKET_FORMAT_T
{
    SXRF_PACKET_FORMAT_FIXED = 0,
    SXRF_PACKET_FORMAT_VARIABLE = 0b10000000
} SXRF_PACKET_FORMAT;

/** DIO0 IRQ mapping */
typedef enum SXRF_DIO0_MAPPING_T
{
    SXRF_DIO0_RX_DONE = 0,
    SXRF_DIO0_TX_DONE = 1,
    SXRF_DIO0_CAD_DONE = 2
} SXRF_DIO0_MAPPING;

/** IRQ Flags
 * Broken into 2 registers, in exact order, labelled for clarity
 */
typedef enum SXRF_IRQ_FLAG_T
{
    // RegIrqFlags1 - Ordered [7:0]
    SXRF_IRQ_MODE_READY = 7,
    SXRF_IRQ_RX_READY = 6,
    SXRF_IRQ_TX_READY = 5,
    SXRF_IRQ_PLL_LOCK = 4,
    SXRF_IRQ_RSSI = 3,
    SXRF_IRQ_TIMEOUT = 2,
    SXRF_IRQ_PREAMBLE_DETECT = 1,
    SXRF_IRQ_SYNC_ADDRESS_MATCH = 0,

    // RegIrqFlags2 - Ordered [7:0]
    SXRF_IRQ_FIFO_FULL = 15,
    SXRF_IRQ_FIFO_EMPTY = 14,
    SXRF_IRQ_FIFO_LEVEL = 13,
    SXRF_IRQ_FIFO_OVERRUN = 12,
    SXRF_IRQ_PACKET_SENT = 11,
    SXRF_IRQ_PAYLOAD_READY = 10,
    SXRF_IRQ_CRC_OK = 9,
    SXRF_IRQ_LOW_BAT = 8
} SXRF_IRQ_FLAG;

/** Generic return values */
typedef enum SXRF_RET_T
{
    SXRF_OK = 0,
    SXRF_ERROR = -1,
    SXRF_ERR_INVALID_INPUT = -2,
    SXRF_ERR_TIMEOUT = -3
} SXRF_RET;

SXRF_RET sxrf_set_addr_filtering(SXRF_ADDR_FILTERING_MODE mode);

u8 sxrf_get_node_addr();
SXRF_RET sxrf_set_node_addr(u8 addr);

u8 sxrf_get_sync_word(u8 *buff, u8 buff_size);
SXRF_RET sxrf_set_sync_word(const u8 *sync_word, u8 size);

u16 sxrf_get_preamble_size();
SXRF_RET sxrf_set_preamble_size(u16 size);

SXRF_RET sxrf_set_modulation(SXRF_MODULATION mod);
SXRF_MODULATION sxrf_get_modulation();

u32 sxrf_get_freq();
SXRF_RET sxrf_set_freq(u32 freq_hz);

int sxrf_get_freq_dev();
SXRF_RET sxrf_set_freq_dev(int dev);

SXRF_RET sxrf_set_rx_bw(int bw);
int sxrf_get_rx_bw();

float sxrf_get_bitrate();
SXRF_RET sxrf_set_bitrate(float bitrate);

SXRF_OP_MODE sxrf_get_op_mode();
SXRF_RET sxrf_set_op_mode(SXRF_OP_MODE mode);

SXRF_RET sxrf_set_dio0_mapping(SXRF_DIO0_MAPPING val);

bool sxrf_is_irq_flag_set(SXRF_IRQ_FLAG flag);
SXRF_RET sxrf_clear_irq_flag(SXRF_IRQ_FLAG flag);

SXRF_RET sxrf_set_packet_format(SXRF_PACKET_FORMAT format);

SXRF_RET sxrf_set_crc_on(bool on);

float sxrf_get_rssi();

u8 sxrf_get_version();

SXRF_RET sxrf_set_reg_bits(u8 addr, u8 mask, u8 val);

SXRF_RET sxrf_write_byte_verify(const u8 addr, u8 val);
u8 sxrf_read_byte(const u8 addr);
void sxrf_write_byte(const u8 addr, u8 val);

u32 sxrf_read_fifo(u32 buff, u32 buff_size);
SXRF_RET sxrf_clear_fifo();

//
// Must be implemented by HAL
//
/** Read data from SPI */
extern void sxrf_read(u8 addr, u8 *buff, int read_len);
/** Write data from SPI */
extern void sxrf_write(u8 addr, const u8 *buff, int read_len);
/** Wait X mS */
extern void sxrf_wait_ms(u32 ms);
/** Get system tick (mS) */
extern u32 sxrf_get_tick_ms();

#endif
