#pragma once

#include "mcp251863/MCP251XFD.h"

const uint16_t aircraft_msg_id = 0x1AC;

enum adsb_packet_type_t : uint8_t	{
	MODE_S_IDENT 			= 1,		// TC 1 - 4
	MODE_S_SURFACE 			= 5,		// TC 5 - 8
	MODE_S_POSITION_BARO 	= 9,		// TC 9 - 18
	MODE_S_VELOCITY 		= 19,		// TC 19
	MODE_S_POSITION_GNSS 	= 20,		// TC 20 - 22
	MODE_S_RESERVED 		= 23,		// TC 23 - 27
	MODE_S_AIRCRAFT_STATUS 	= 28,		// TC 28
	MODE_S_TARGET_STATE 	= 29,		// TC 29
	MODE_S_OPERATION_STATUS = 31,		// TC 31

	UAT_IDENT				= 40,
	UAT_STATE_VECTOR		= 41,
	UAT_AUXILIARY_STATE_VECTOR = 42,
	UAT_TARGET_STATE		= 43,
	UAT_TRAJECTORY_CHANGE	= 44,

	ADSBEE_STATUS_MSG		= 50
};


enum can_termination_t	{
	CAN_TERM_ON = 0,		// GPIO low to enable termination resistors
	CAN_TERM_OFF = 1		// GPIO high to disable termination resistors
};

struct queue_msg_t	{
	uint32_t uid;
	adsb_packet_type_t packet_type;
};

extern TaskHandle_t canbus_task_handle;
void canbus_task(void* pvParameters);

extern bool volatile efis_connected;

extern uint32_t time_since_zulu;

#include "gdl90_utils.hh"
extern GDL90Reporter::GDL90TargetReportData ownship_data;



bool CanbusInit(can_termination_t term_res_enable);
void transmit_can(uint32_t canID, eMCP251XFD_DataLength DLC, uint8_t *data);
void check_can_errors();
void send_heartbeat();
void request_UID();

/**
 * @brief Initialize SPI bus for MCP251XFD
 * 
 * This function is called by the MCP251XFD driver to initialize the SPI interface.
 * 
 * @param pIntDev Pointer to the SPI device handle (unused, but required by interface)
 * @param chipSelect Chip select pin (unused, we use the one in devcfg)
 * @param sckFreq SPI clock frequency in Hz
 * @return eERRORRESULT Error code
 */
eERRORRESULT MCP251XFD_SPI_Init(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq);

/**
 * @brief SPI transfer function for MCP251XFD
 * 
 * This function performs SPI transactions for the MCP251XFD driver.
 * 
 * @param pIntDev Pointer to the SPI device handle (unused, but required by interface)
 * @param chipSelect Chip select pin (unused, we use the one in devcfg)
 * @param txData Pointer to transmit data buffer
 * @param rxData Pointer to receive data buffer (can be NULL for write-only)
 * @param size Number of bytes to transfer
 * @return eERRORRESULT Error code
 */
eERRORRESULT MCP251XFD_SPI_Transfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size);


static inline uint16_t MCP251XFD_ComputeCRC16(const uint8_t* data, size_t size)
{
    // Not used when CRC mode is disabled
    (void)data;
    (void)size;
    return 0;
}

/**
 * @brief Get current time in milliseconds
 * 
 * @return Current time in milliseconds
 */
static inline uint32_t MCP251XFD_GetCurrentms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}