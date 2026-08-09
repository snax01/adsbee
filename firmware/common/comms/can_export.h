/*
 * can_export.h
 *
 *  Created on: Jul 17, 2025
 *      Author: snax
 *
 * 29-bit extended CAN ID layout (CANfd + BRS):
 *   Bits 28..27  unused (0)
 *   Bits 26..16  11-bit message type (CAN_IDS_t / CANMSG_*)
 *   Bits 15..8   Source UID
 *   Bits  7..0   Destination UID
 *
 * COMMS_MSG_t.can_ID holds the full 29-bit arbitration ID on the wire
 * and over USB HID passthrough.
 */

#ifndef EFIS_SHARED_CAN_EXPORT_H_
#define EFIS_SHARED_CAN_EXPORT_H_

#include <stdint.h>

/* Reserved UIDs */
static const uint8_t UID_BROADCAST = 0x00;	/* Destination for bus-wide messages */
static const uint8_t UID_MASTER    = 0x01;	/* RADbus AE master */
static const uint8_t UID_PC        = 0xFF;	/* PC / config application */

/* 29-bit ID field masks / shifts */
static const uint32_t CAN_MSG_MASK  = 0x7FFu;
static const uint32_t CAN_MSG_SHIFT = 16u;
static const uint32_t CAN_SRC_SHIFT = 8u;
static const uint32_t CAN_DEST_MASK = 0xFFu;
static const uint32_t CAN_SRC_MASK  = 0xFFu;

static inline uint32_t can_pack_id(uint16_t msg, uint8_t src, uint8_t dest)
{
	return ((uint32_t)(msg & CAN_MSG_MASK) << CAN_MSG_SHIFT)
	     | ((uint32_t)src << CAN_SRC_SHIFT)
	     | (uint32_t)dest;
}

static inline uint16_t can_msg_of(uint32_t arb_id)
{
	return (uint16_t)((arb_id >> CAN_MSG_SHIFT) & CAN_MSG_MASK);
}

static inline uint8_t can_src_of(uint32_t arb_id)
{
	return (uint8_t)((arb_id >> CAN_SRC_SHIFT) & CAN_SRC_MASK);
}

static inline uint8_t can_dest_of(uint32_t arb_id)
{
	return (uint8_t)(arb_id & CAN_DEST_MASK);
}

/* Message types — full 11-bit space (0x000..0x7FF); no low-nibble reservation */
typedef enum	{

	CANMSG_WHOAMI_REQUEST = 0x010,

	CANMSG_ACK = 0x0E0,
	CANMSG_EXTFLASH_ACK = 0x0F0,	/* deprecated TX: use CANMSG_ACK with flash trailer */

	CANMSG_EFIS_LOG = 0x100,	// TODO - probably remove

	CANMSG_ADSB_AIRCRAFT_IN = 0x120,
	CANMSG_ADSB_OWNSHIP_STATE = 0x130,
	CANMSG_ADSB_OWNSHIP_IDENT = 0x140,
	CANMSG_ADSB_WAKEUP = 0x150,

	// Using for sensor calibration
	// TODO - Make one CANMSG to be used to send
	CANMSG_ACC_UNCAL = 0x200,
	CANMSG_ACC	= 0x210,

	CANMSG_GYRO_UNCAL = 0x220,
	CANMSG_GYRO = 0x230,

	CANMSG_MAG_MMC_UNCAL = 0x240,
	CANMSG_MAG_MMC	= 0x250,

	CANMSG_MAG_LIS_UNCAL = 0x260,
	CANMSG_MAG_LIS	= 0x270,

	// TODO - Currently only sending pressure value without offset.
	CANMSG_ST_PRES	= 0x280,
	CANMSG_PT_PRES	= 0x290,
	CANMSG_LPS_STATIC = 0x2A0,
	// Using for sensor calibration

	// Data for aux screens
	CANMSG_AHRS_DATA = 0x2B0,
	CANMSG_AIRDATA_PITOT = 0x2C0,
	CANMSG_AIRDATA_STATIC = 0x2D0,
	CANMSG_GNSS_DATA = 0x2E0,
	CANMSG_MISC_DATA = 0x2F0,
	CANMSG_QNH_DATA = 0x300,
	CANMSG_DIR_GYRO_DATA = 0x310,
	// Data for aux screens

	CANMSG_EN_MSG = 0x320,		// Used to enable sensor output

	CANMSG_REQUEST_SENSOR_CALIB = 0x330,
	CANMSG_SEND_SENSOR_CALIB = 0x340,
	CANMSG_SET_SENSOR_CALIB = 0x350,
	CANMSG_SAVE_SENSOR_CALIB = 0x360,

	CANMSG_REQUEST_SETTINGS_VALUE = 0x370,
	CANMSG_SEND_SETTINGS_VALUE = 0x380,
	CANMSG_RECEIVE_SETTINGS_VALUE = 0x380,
	CANMSG_GLOBAL_SET_SETTING = 0x390,
	CANMSG_SAVE_USER_SETTINGS = 0x3A0,

	CANMSG_ENTER_BL = 0x3B0,

	CANMSG_FACTORY_RESET = 0x400,
	CANMSG_READ_EXTFLASH = 0x410,
	CANMSG_WRITE_EXTFLASH = 0x420,
	CANMSG_ERASE_EXTFLASH = 0x430,
	CANMSG_SET_ADDR_EXTFLASH = 0x440,

	CANMSG_AIRPORT_DB_OP = 0x450,

	CANMSG_REQUEST_RADBUS_NUM_CLIENTS = 0x600,
	CANMSG_REQUEST_RADBUS_CLIENT = 0x600,

	// TODO - update ID's for this in can_auth.py
	CANMSG_NONCE = 0x710,
	CANMSG_RS = 0x720,

	CANMSG_PREPARE_FW_UPDATE = 0x730,

	CANMSG_HEARTBEAT = 0x750,
	CANMSG_UID_REQUEST = 0x760,
	CANMSG_UID_ASSIGN = 0x770,
	CANMSG_DISCONNECT_CLIENT = 0x780,
	CANMSG_RADBUS_CLIENTS = 0x790,

	CANMSG_IDRIVE_SETBRIGHTNESS = 0x202,
	CANMSG_IDRIVE_ENCODER = 0x264,
	CANMSG_IDRIVE_BUTTON_STATUS = 0x267,
	CANMSG_IDRIVE_INIT_RESPONSE = 0x273,
	CANMSG_IDRIVE_KEEPALIVE = 0x563,
	CANMSG_IDRIVE_STATUS = 0x5E7,

	CANMSG_MAXID = 0x7FF
}CAN_IDS_t;

/** Unified ACK / NACK status (payload byte 1). */
typedef enum {
	ACK_STATUS_FAIL = 0,
	ACK_STATUS_OK   = 1,
	ACK_STATUS_BUSY = 2	/* accepted / in progress; completion follows */
} ack_status_t;

/*
 * Unified ACK payload (CANMSG_ACK):
 *   [0]     UID (must match arb src)
 *   [1]     ack_status_t
 *   [2-3]   11-bit msg type being ACKed
 *   [4-6]   sequence (0 if unused)
 *   [7]     FlashOpType (OP_NONE if not a flash completion)
 *   [8-11]  address (flash completion only)
 * DLC 7  = no flash trailer (bytes 7+ absent / unused)
 * DLC 12 = flash completion with op + addr
 */

enum can_msg_outputs{
	CB_ACC_UNCAL = 0,
	CB_ACC_CAL,
	CB_GYRO_UNCAL,
	CB_GYRO_CAL,
	CB_MMC_UNCAL,
	CB_MMC_CAL,
	CB_LIS_UNCAL,
	CB_LIS_CAL,
	CB_STATIC,
	CB_PITOT,
	CB_LPS_STATIC,

	CB_COUNT
};

#endif /* EFIS_SHARED_CAN_EXPORT_H_ */
