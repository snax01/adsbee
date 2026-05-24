#pragma once

#include "esp_timer.h"
#include "mcp251863/MCP251XFD.h"
#include "settings.hh"
#include "mode_s_packet.hh"
#include "uat_packet.hh"

enum adsb_packet_type_t : uint8_t {
    MODE_S_IDENT = 1,               // TC 1 - 4
    MODE_S_SURFACE = 5,             // TC 5 - 8
    MODE_S_POSITION_BARO = 9,       // TC 9 - 18
    MODE_S_VELOCITY = 19,           // TC 19
    MODE_S_POSITION_GNSS = 20,      // TC 20 - 22
    MODE_S_RESERVED = 23,           // TC 23 - 27
    MODE_S_AIRCRAFT_STATUS = 28,    // TC 28
    MODE_S_TARGET_STATE = 29,       // TC 29
    MODE_S_OPERATION_STATUS = 31,   // TC 31

    UAT_IDENT = 40,
    UAT_STATE_VECTOR = 41,
    UAT_AUXILIARY_STATE_VECTOR = 42,
    UAT_TARGET_STATE = 43,
    UAT_TRAJECTORY_CHANGE = 44,

    ADSBEE_STATUS_MSG = 50
};

enum can_termination_t {
    CAN_TERM_ON = 0,   // GPIO low to enable termination resistors
    CAN_TERM_OFF = 1   // GPIO high to disable termination resistors
};

extern bool volatile efis_connected;

extern uint32_t time_since_zulu;

/** Receiver position updated from RADbus CAN ownship messages (ESP32 only). */
SettingsManager::RxPosition& GetRadbusRxPosition();

#include "gdl90_utils.hh"

bool CanbusInit(can_termination_t term_res_enable);
bool CanbusIsInitialized();

/** Poll RADbus RX and send heartbeat / UID request. Call from ADSBeeServer::Update(). */
void CanbusUpdate();

/**
 * Transmit traffic for a Mode S packet that was just ingested into the aircraft dictionary.
 * No-op if CAN is not initialized or no EFIS client is connected.
 */
void ReportCANFromIngestedModeSPacket(const DecodedModeSPacket& decoded_packet);

/**
 * Transmit traffic for a UAT ADS-B packet that was just ingested into the aircraft dictionary.
 */
void ReportCANFromIngestedUATPacket(const DecodedUATADSBPacket& decoded_packet);

void transmit_can(uint32_t canID, eMCP251XFD_DataLength DLC, uint8_t* data);
void check_can_errors();
void send_heartbeat();
void request_UID();

eERRORRESULT MCP251XFD_SPI_Init(void* pIntDev, uint8_t chipSelect, const uint32_t sckFreq);
eERRORRESULT MCP251XFD_SPI_Transfer(void* pIntDev, uint8_t chipSelect, uint8_t* txData, uint8_t* rxData, size_t size);

static inline uint16_t MCP251XFD_ComputeCRC16(const uint8_t* data, size_t size) {
    (void)data;
    (void)size;
    return 0;
}

static inline uint32_t MCP251XFD_GetCurrentms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}
