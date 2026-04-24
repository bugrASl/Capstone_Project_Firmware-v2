/**
 *  @file       wireless_packet.h
 *  @brief      Wireless packet structure and 12-bit compression
 *  @author     bugrASl
 *  @date       10.04.2026
 *  @version    2.1
 *
 *  @details
 *      ─────────────── Wire Layout (32-Byte, Fixed Payload) ───────────────────────────
 *      
 *      Byte    Fields          Size        Description
 *      ────────────────────────────────────────────────────────────────────────────────
 *      [0]     seq             1B          Packet Sequence Number(0-255, wraps)
 *      [1]     flags           1B          Status Flags, 2-bit Battery Level
 *      [2]     tx_retry        1B          ARC_CNT: Retransmitions for this packet
 *      [3]     pkt_loss        1B          PLOS_CNT: Cumulative lost packets
 *      [4-5]   timestamp       2B          TIM2 us counter, little-endian
 *      [6-7]   vbat_raw        2B          12-bit Battery ADC reading, high-nibble-aligned
 *      [8-19]  sample[0]       12B         8 Channels x 12-bit ADC reading, packed
 *      [20-31] sample[1]       12B         8 Channels x 12-bit ADC reading, packed
 *      ────────────────────────────────────────────────────────────────────────────────
 *      Total:  32B
 *
 *      ─────────────── 12-bit Packing ─────────────────────────────────────────────────
 *      Two 12-bit values A, B -> 3 bytes:
 *          byte[0] =   A[7:0]
 *          byte[1] =   A[11:8] | (B[3:0] << 4)
 *          byte[2] =   B[11:4]
 *      ────────────────────────────────────────────────────────────────────────────────
 *
 *      ─────────────── vbat_raw Encoding ──────────────────────────────────────────────
 *          byte[0] =   vbat_raw[11:4]
 *          byte[1] =   (vbat_raw[3:0] << 4)   
 *      ────────────────────────────────────────────────────────────────────────────────
 */

#ifndef WIRELESS_PACKET_H
#define WIRELESS_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============= CONFIGURABLE CONSTANTS ====================================================*/

#define WL_PAYLOAD_SIZE         32
#define WL_NUM_CHANNELS         8
#define WL_SAMPLES_PER_PACKET   2

/*============= BYTE OFFSETS ==============================================================*/

#define WL_OFF_SEQ              0
#define WL_OFF_FLAGS            1
#define WL_OFF_TX_RETRY         2
#define WL_OFF_PKT_LOSS         3
#define WL_OFF_TIMESTAMP        4
#define WL_OFF_VBAT_RAW         6
#define WL_OFF_SAMPLES          8    

/*============= DYNAMIC CONSTANTS =========================================================*/

#define WL_BYTES_PER_SAMPLE     ( (WL_NUM_CHANNELS / 2) * 3 )
#define WL_OFF_SAMPLE(s)        ( WL_OFF_SAMPLES + ( (s) * WL_BYTES_PER_SAMPLE ) )

/*============= COMPILE ASSERTS ===========================================================*/

_Static_assert(
        WL_OFF_SAMPLES + ( WL_SAMPLES_PER_PACKET * WL_BYTES_PER_SAMPLE ) == WL_PAYLOAD_SIZE,
        "Packet layout does not fill 32-bytes payload"
        );

/*============= FLAG BYTES ================================================================*/
/*
 *  bit 7:      FIRST_PACKET_FLAG   - resets expected_seq on receiver
 *  bit 6:      CLIPPING_FLAG       - any channel saturated (0x000 or 0xFFF)
 *  bit 5:      ELEC_OFF_FLAG       - electrode contact impedence too high
 *  bit 4:      ADC_OVRN_FLAG       - ADC overrun detected
 *  bit 3:      TX_SAT_FLAG         - TX_FIFO was full when packet submitted
 *  bit 2:      CAL_FLAG            - calibration frame
 *  bit [1:0]:  BATT_LVL            - 2-bit battery level
 */

#define WL_FLAG_FIRST_PACKET    (1u << 7)
#define WL_FLAG_CLIPPING        (1u << 6)
#define WL_FLAG_ELEC_OFF        (1u << 5)
#define WL_FLAG_ADC_OVRN        (1u << 4)
#define WL_FLAG_TX_SAT          (1u << 3)
#define WL_FLAG_CAL             (1u << 2)
                        
/*----- BATT-LVL ------*/

#define WL_BATT_MASK            (0x03u)
#define WL_BATT_OK              (0x00u)
#define WL_BATT_LOW             (0x01u)
#define WL_BATT_CRIT            (0x02u)
#define WL_BATT_CHARG           (0x03u)

/** Write battery level into flags byte */
#define WL_BATT_SET(flags, level)   ( ( (flags) & ~WL_BATT_MASK ) | ( (level) & WL_BATT_MASK ) )
/** Extract battery level from flags byte */
#define WL_BATT_GET(flags)          ( (flags) & WL_BATT_MASK )

/*============= VBAT_RAW HELPERS ==========================================================*/
/**
 *  Encoding:
 *      byte[0] =   val[11:4]
 *      byte[1] =   (val[3:0] << 4)
 */

/** Pack 12-bit vbat into wire bytes */
#define WL_VBAT_ENCODE(out, val)\
    do\
    {\
        (out)[0]  =   (uint8_t)( ( (val) >> 4 ) & 0xFF );\
        (out)[1]  =   (uint8_t)( ( (val) & 0x0F ) << 4 );\
    }\
    while(0)

/** Unpack 12-bit vbat from wire bytes */
#define WL_VBAT_DECODE(in)\
    ( (uint16_t)( (uint16_t)((in)[0]) << 4 ) | (uint16_t)( ( (in)[1] >> 4 ) & 0x0F ) )

/*============= STRUCTURES ================================================================*/

typedef struct
{
    uint16_t    ch[WL_NUM_CHANNELS];
} WL_SampleSet;

typedef struct
{
    uint8_t         seq;
    uint8_t         flags;
    uint8_t         tx_retry;
    uint8_t         pkt_loss;
    uint16_t        timestamp;
    uint16_t        vbat_raw;
    WL_SampleSet    samples[WL_SAMPLES_PER_PACKET];
} WL_Packet;    

/*============= API =======================================================================*/

void WL_Pack(const WL_Packet *in, uint8_t *out);
void WL_Unpack(const uint8_t *in, WL_Packet *out);

/*=========================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_PACKET_H */
