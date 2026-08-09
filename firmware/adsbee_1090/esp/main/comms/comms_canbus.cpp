#include "comms.hh"

#include "comms_canbus.hh"
#include "can_export.h"
#include "mcp251863/fifo_conf.h"
#include "adsbee_server.hh"
#include "aircraft_dictionary.hh"
#include "device_info.hh"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal.hh"
#include "object_dictionary.hh"

extern GDL90Reporter gdl90;



#define CAN_STANDBY_PIN     GPIO_NUM_12
#define CAN_TERM_ENABLE     GPIO_NUM_21

#define CAN_RX_INT_PIN      GPIO_NUM_47 // Msg received interrupt
#define CAN_INT_PIN         GPIO_NUM_48 // General Interrupts from MCP251863

#define CAN_SPI_HOST        SPI3_HOST
#define SPI_MOSI_PIN        GPIO_NUM_14
#define SPI_MISO_PIN        GPIO_NUM_13
#define SPI_SCLK_PIN        GPIO_NUM_17
#define SPI_CS_PIN          GPIO_NUM_18

#define SPI_CLOCK           8000000    // 8 MHz - TODO test higher SPI clock rates

// MCP251XFD SPI frames are small (see MCP251XFD_TRANS_BUF_SIZE). Keep this low to limit
// SPI master DMA heap reserved at spi_bus_initialize() time on memory-tight builds.
#define CAN_SPI_MAX_TRANSFER_BYTES 128

/* Local-only UID sentinels (never assigned on the bus). Matches EFIS radbus.h. */
static const uint8_t kRadbusUidUnassigned = 0xFE;
static const uint8_t kClientAddErrorCode = 0x99;

/* Ephemeral src range for unassigned TX (arbitration uniqueness). */
static const uint8_t kProvisionalSrcMin = 0x80;
static const uint8_t kProvisionalSrcMax = 0xFD;

/** Match only the 11-bit message type in a packed 29-bit extended ID. */
static const uint32_t kCanMsgTypeFilterMask = CAN_MSG_MASK << CAN_MSG_SHIFT;

static spi_device_handle_t  can_spi_handle = NULL;

// Shared DMA staging buffer for MCP251XFD SPI (stack buffers are not cache/DMA-safe on ESP32-S3).
alignas(64) static uint8_t s_can_spi_dma_buf[CAN_SPI_MAX_TRANSFER_BYTES];

// Buffer for received message data
uint8_t rx_data[64];
MCP251XFD_CANMessage rx_message = {
    .PayloadData = rx_data,
};

// Buffer for transmit message data — CAN-FD + BRS + extended 29-bit ID
MCP251XFD_CANMessage tx_message = {
    .MessageID = 0,
    .MessageSEQ = 0,
    .ControlFlags = (setMCP251XFD_MessageCtrlFlags)(
        MCP251XFD_CANFD_FRAME | MCP251XFD_SWITCH_BITRATE | MCP251XFD_EXTENDED_MESSAGE_ID),
    .DLC = MCP251XFD_DLC_2BYTE,
    .PayloadData = NULL,
};

bool volatile efis_connected = false;
uint8_t RADbus_UID = kRadbusUidUnassigned;
uint8_t provisional_src = kRadbusUidUnassigned;
uint32_t last_heartbeat = 0;
// High byte = Device_type_FF (0xFF); low 24 bits = device unique ID. Set in CanbusInit().
uint32_t serial_num = 0;

uint32_t time_since_zulu;

static SettingsManager::RxPosition radbus_rx_position = {};

SettingsManager::RxPosition& GetRadbusRxPosition() { return radbus_rx_position; }

static bool canbus_initialized = false;

// Set from GPIO ISR when MCP251863 INT1 (active low) asserts; drained in CanbusUpdate().
static volatile bool can_rx_pending = false;

static void IRAM_ATTR can_rx_gpio_isr(void* /*arg*/) { 
    can_rx_pending = true;
}

/**
 * Build RADbus serial: 0xFFXXXXXX.
 * Prefer a 24-bit fold of the manufacturing feed receiver ID (synced from Pico DeviceInfo).
 * Fall back to ESP32 base MAC[3..5] if the feed ID is unset.
 */
static uint32_t build_radbus_serial_num() {
    static const uint32_t kDeviceTypeFF = 0xFFu << 24;
    static const uint32_t kUnique24Mask = 0x00FFFFFFu;

    uint32_t unique24 = 0;
    const uint8_t* rid = settings_manager.settings.feed_receiver_ids[0];
    // feed_receiver_ids: [0]=0xBE [1]=0xE0 [2..7]=6-byte manufacturing UID (MSB first).
    const bool rid_valid = (rid[0] == 0xBE && rid[1] == 0xE0)
                           && (rid[2] | rid[3] | rid[4] | rid[5] | rid[6] | rid[7]) != 0;
    if (rid_valid) {
        // Fold 48-bit manufacturing UID into 24 bits so date + VVXXXX both contribute.
        unique24 = SettingsManager::DeviceInfo::FoldReceiverIdToUnique24(rid);
    } else {
        ObjectDictionary::ESP32DeviceInfo esp_info = GetESP32DeviceInfo();
        unique24 = ((uint32_t)esp_info.base_mac[3] << 16) | ((uint32_t)esp_info.base_mac[4] << 8)
                   | (uint32_t)esp_info.base_mac[5];
        CONSOLE_WARNING("RADbus", "feed_receiver_id unset; using ESP MAC for serial unique bits.");
    }

    return kDeviceTypeFF | (unique24 & kUnique24Mask);
}

/** FNV-1a fold of serial → provisional src in 0x80..0xFD while unassigned. */
static uint8_t radbus_serial_src_hash(uint32_t serial) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < 4; i++) {
        h ^= (serial >> (i * 8)) & 0xFFu;
        h *= 16777619u;
    }
    h ^= h >> 16;
    const uint8_t span = (uint8_t)(kProvisionalSrcMax - kProvisionalSrcMin + 1);
    return (uint8_t)(kProvisionalSrcMin + (h % span));
}

/** Wire source UID: assigned UID when connected, else serial-derived provisional src. */
static uint8_t radbus_tx_src() {
    if (efis_connected) {
        return RADbus_UID;
    }
    return provisional_src;
}


// MCP251XFD Device Configuration
MCP251XFD MCP251XFD_Ext1 = 
{ 
  .UserDriverData  = NULL, 
  //--- Driver configuration --- 
  .DriverConfig    = MCP251XFD_DRIVER_NORMAL_USE, 
  //--- IO configuration --- 
  .GPIOsOutLevel   = MCP251XFD_GPIO0_HIGH | MCP251XFD_GPIO1_HIGH, 
  //--- Interface driver call functions --- 
  .SPI_ChipSelect  = 0,                    // Chip select index (not used with ESP-IDF SPI driver)
  .InterfaceDevice = &can_spi_handle,          // Pointer to SPI device handle
  //--- Interface clocks --- 
  .SPIClockSpeed   = SPI_CLOCK,
  
  .fnSPI_Init      = MCP251XFD_SPI_Init, 
  .fnSPI_Transfer  = MCP251XFD_SPI_Transfer, 
  //--- Time call function --- 
  .fnGetCurrentms  = MCP251XFD_GetCurrentms, 
  //--- CRC16-CMS call function --- 
  .fnComputeCRC16  = MCP251XFD_ComputeCRC16, 
};

// SPI Bus Configuration Structure
static spi_bus_config_t buscfg = {
    .mosi_io_num = SPI_MOSI_PIN,
    .miso_io_num = SPI_MISO_PIN,
    .sclk_io_num = SPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = CAN_SPI_MAX_TRANSFER_BYTES,
};

// SPI Device Configuration Structure
static spi_device_interface_config_t devcfg = {
    .mode = 0,
    .clock_speed_hz = SPI_CLOCK,
                        // SPI mode 0 (CPOL=0, CPHA=0)
    .spics_io_num = SPI_CS_PIN,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL
};


static bool check_rx_fifo(void)
{
    setMCP251XFD_FIFOstatus st = MCP251XFD_RX_FIFO_EMPTY;
    if (MCP251XFD_GetFIFOStatus(&MCP251XFD_Ext1, MCP251XFD_FIFO1, &st) != ERR_OK) {
        return false;
    }
    return (st & MCP251XFD_RX_FIFO_NOT_EMPTY) != 0;
}

/** Discard any frames already in RX FIFO1 (noise / traffic before we are ready). */
static void drain_rx_fifo(void)
{
    uint32_t ts;
    while (check_rx_fifo()) {
        (void)MCP251XFD_ReceiveMessageFromFIFO(
            &MCP251XFD_Ext1,
            &rx_message,
            MCP251XFD_PAYLOAD_64BYTE,
            &ts,
            MCP251XFD_FIFO1);
    }
    can_rx_pending = false;
}

void process_rx_msg()   {
    uint32_t ts;
    eERRORRESULT e = MCP251XFD_ReceiveMessageFromFIFO(
        &MCP251XFD_Ext1,
        &rx_message,
        MCP251XFD_PAYLOAD_64BYTE,   // must match fifo_conf.h for FIFO1
        &ts,                        // NULL if you don't want timestamp, need to change in fifo_conf if i want to remove timestamp
        MCP251XFD_FIFO1);
    if (e != ERR_OK) {
        return;
    }

    const uint32_t arb_id = rx_message.MessageID;
    const uint16_t msg_id = can_msg_of(arb_id);
    const uint8_t dest = can_dest_of(arb_id);

    // Soft dest filter: accept broadcast and frames addressed to our assigned UID.
    if (dest != UID_BROADCAST && dest != RADbus_UID) {
        return;
    }

    switch (msg_id)   {
        case (CANMSG_ADSB_OWNSHIP_STATE):   {
            memcpy(&time_since_zulu, &rx_data[0], sizeof(uint32_t));
            radbus_rx_position.source = SettingsManager::RxPosition::kPositionSourceRADbus;
            memcpy(&radbus_rx_position.latitude_deg, &rx_data[4], sizeof(float));
            memcpy(&radbus_rx_position.longitude_deg, &rx_data[8], sizeof(float));
            memcpy(&radbus_rx_position.baro_altitude_ft, &rx_data[12], sizeof(int32_t));
            float speed_kts = 0.0f;
            memcpy(&speed_kts, &rx_data[16], sizeof(float));
            radbus_rx_position.speed_kts = static_cast<int32_t>(speed_kts);
            memcpy(&radbus_rx_position.heading_deg, &rx_data[20], sizeof(float));
            memcpy(&gdl90.ownship_data.vertical_rate_fpm, &rx_data[24], sizeof(int32_t));
            if (time_since_zulu != 0) {
                gdl90.utc_timing_is_valid = true;
                gdl90.gnss_position_valid = true;
            }
            break;
        }

        case (CANMSG_ADSB_OWNSHIP_IDENT):    {
            radbus_rx_position.source = SettingsManager::RxPosition::kPositionSourceRADbus;
            memcpy(&gdl90.ownship_data.callsign, &rx_data[0], 8);
            memcpy(&radbus_rx_position.icao_address, &rx_data[8], sizeof(uint32_t));
            memcpy(&gdl90.ownship_data.participant_address, &rx_data[8], sizeof(uint32_t));
            memcpy(&gdl90.ownship_data.address_type, &rx_data[12], 1);
            memcpy(&gdl90.ownship_data.emitter_category, &rx_data[13], 1);
            memcpy(&gdl90.ownship_data.navigation_integrity_category, &rx_data[14], 1);
            memcpy(&gdl90.ownship_data.navigation_accuracy_category_position, &rx_data[15], 1);
            memcpy(&gdl90.ownship_data.misc_indicators, &rx_data[16], 1);
            break;
        }

        // DLC 5: [0]=UID (or 0x99 error), [1..4]=serial of assignee
        case (CANMSG_UID_ASSIGN):    {
            if (rx_message.DLC != MCP251XFD_DLC_5BYTE) {
                break;
            }
            uint32_t assigned_serial = 0;
            memcpy(&assigned_serial, &rx_data[1], sizeof(uint32_t));
            if (assigned_serial != serial_num) {
                break;
            }
            if (rx_data[0] != kClientAddErrorCode && rx_data[0] != UID_BROADCAST
                && rx_data[0] != UID_MASTER && rx_data[0] != UID_PC
                && rx_data[0] != kRadbusUidUnassigned) {
                RADbus_UID = rx_data[0];
                efis_connected = true;
                radbus_rx_position.source = SettingsManager::RxPosition::kPositionSourceRADbus;
                CONSOLE_INFO("RADbus", "UID assigned %i.", RADbus_UID);
            }   else    {
                CONSOLE_ERROR("RADbus", "RADbus reached max clients, no client ID given.");
            }
            break;
        }

        case (CANMSG_DISCONNECT_CLIENT):    {
            if (rx_data[0] == RADbus_UID)   {
                efis_connected = false;
                RADbus_UID = kRadbusUidUnassigned;
                radbus_rx_position.source = SettingsManager::RxPosition::kPositionSourceNone;
                gdl90.gnss_position_valid = false;
                gdl90.utc_timing_is_valid = false;
                CONSOLE_INFO("RADbus", "Disconnected by master.");
            }
            break;
        }

        default:
            break;
    }
}

void request_UID()  {
    uint8_t UID_request[8];
    memcpy(&UID_request[0], &serial_num, sizeof(uint32_t));
    memcpy(&UID_request[4], &ObjectDictionary::kFirmwareVersion, sizeof(uint32_t));

    transmit_can(CANMSG_UID_REQUEST, UID_MASTER, MCP251XFD_DLC_8BYTE, &UID_request[0]);
}

void send_heartbeat() {
    if (efis_connected) {
        transmit_can(CANMSG_HEARTBEAT, UID_MASTER, MCP251XFD_DLC_1BYTE, &RADbus_UID);
    } else {
        request_UID();
    }
    last_heartbeat = get_time_since_boot_ms();
}


bool CanbusIsInitialized() { return canbus_initialized; }

void CanbusSetTermination(can_termination_t term_res_enable) {
    gpio_set_level(CAN_TERM_ENABLE, term_res_enable);
    CONSOLE_INFO("CAN", "Termination resistor %s.", term_res_enable == CAN_TERM_ON ? "ENABLED" : "DISABLED");
}

static const uint32_t kCANHeartbeatIntervalMs = 1000;

void CanbusUpdate() {
    if (!canbus_initialized) {
        return;
    }

    if (can_rx_pending) {
        can_rx_pending = false;
        while (check_rx_fifo()) {
            process_rx_msg();
        }
    }

    uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - last_heartbeat >= kCANHeartbeatIntervalMs) {
        send_heartbeat();
    }
}

static void ReportCANModeSTraffic(uint32_t uid, uint8_t packet_type, const ModeSAircraft& aircraft) {
    switch (packet_type) {
        case MODE_S_IDENT:
        case MODE_S_IDENT + 1:
        case MODE_S_IDENT + 2:
        case MODE_S_IDENT + 3: {
            uint8_t tx_buf[16] = {0};
            tx_buf[0] = packet_type;
            memcpy(&tx_buf[1], &uid, 4);
            memcpy(&tx_buf[5], &aircraft.callsign, 8);
            tx_buf[13] = aircraft.emitter_category;
            transmit_can(CANMSG_ADSB_AIRCRAFT_IN, UID_BROADCAST, MCP251XFD_DLC_16BYTE, &tx_buf[0]);
            break;
        }

        case MODE_S_SURFACE:
        case MODE_S_SURFACE + 1:
        case MODE_S_SURFACE + 2:
        case MODE_S_SURFACE + 3:
            break;

        case MODE_S_POSITION_BARO:
        case MODE_S_POSITION_BARO + 1:
        case MODE_S_POSITION_BARO + 2:
        case MODE_S_POSITION_BARO + 3:
        case MODE_S_POSITION_BARO + 4:
        case MODE_S_POSITION_BARO + 5:
        case MODE_S_POSITION_BARO + 6:
        case MODE_S_POSITION_BARO + 7:
        case MODE_S_POSITION_BARO + 8:
        case MODE_S_POSITION_BARO + 9:
        case MODE_S_POSITION_GNSS:
        case MODE_S_POSITION_GNSS + 1:
        case MODE_S_POSITION_GNSS + 2: {
            uint8_t tx_buf[20] = {0};
            tx_buf[0] = packet_type;
            memcpy(&tx_buf[1], &uid, 4);
            memcpy(&tx_buf[5], &aircraft.latitude_deg, 4);
            memcpy(&tx_buf[9], &aircraft.longitude_deg, 4);
            memcpy(&tx_buf[13], &aircraft.baro_altitude_ft, 4);
            transmit_can(CANMSG_ADSB_AIRCRAFT_IN, UID_BROADCAST, MCP251XFD_DLC_20BYTE, &tx_buf[0]);
            break;
        }

        case MODE_S_VELOCITY: {
            uint8_t tx_buf[20] = {0};
            tx_buf[0] = packet_type;
            memcpy(&tx_buf[1], &uid, 4);
            memcpy(&tx_buf[5], &aircraft.direction_deg, 4);
            memcpy(&tx_buf[9], &aircraft.baro_vertical_rate_fpm, 4);
            memcpy(&tx_buf[13], &aircraft.speed_kts, 4);
            transmit_can(CANMSG_ADSB_AIRCRAFT_IN, UID_BROADCAST, MCP251XFD_DLC_20BYTE, &tx_buf[0]);
            break;
        }

        case MODE_S_RESERVED:
        case MODE_S_RESERVED + 1:
        case MODE_S_RESERVED + 2:
        case MODE_S_RESERVED + 3:
        case MODE_S_RESERVED + 4:
        case MODE_S_AIRCRAFT_STATUS:
        case MODE_S_TARGET_STATE:
        case MODE_S_OPERATION_STATUS:
            break;

        default:
            break;
    }
}

static void ReportCANUATTraffic(uint32_t uid, adsb_packet_type_t packet_type, const UATAircraft& aircraft) {
    switch (packet_type) {
        case UAT_IDENT: {
            uint8_t tx_buf[16] = {0};
            tx_buf[0] = packet_type;
            memcpy(&tx_buf[1], &uid, 4);
            memcpy(&tx_buf[5], &aircraft.callsign, 8);
            tx_buf[13] = aircraft.emitter_category;
            transmit_can(CANMSG_ADSB_AIRCRAFT_IN, UID_BROADCAST, MCP251XFD_DLC_16BYTE, &tx_buf[0]);
            break;
        }

        case UAT_STATE_VECTOR: {
            uint8_t tx_buf[30] = {0};
            tx_buf[0] = packet_type;
            memcpy(&tx_buf[1], &uid, 4);
            memcpy(&tx_buf[5], &aircraft.latitude_deg, 4);
            memcpy(&tx_buf[9], &aircraft.longitude_deg, 4);
            memcpy(&tx_buf[13], &aircraft.baro_altitude_ft, 4);
            memcpy(&tx_buf[17], &aircraft.direction_deg, 4);
            memcpy(&tx_buf[21], &aircraft.baro_vertical_rate_fpm, 4);
            memcpy(&tx_buf[25], &aircraft.speed_kts, 4);
            transmit_can(CANMSG_ADSB_AIRCRAFT_IN, UID_BROADCAST, MCP251XFD_DLC_32BYTE, &tx_buf[0]);
            break;
        }

        case UAT_AUXILIARY_STATE_VECTOR:
        case UAT_TARGET_STATE:
        case UAT_TRAJECTORY_CHANGE:
            break;

        default:
            break;
    }
}

void ReportCANFromIngestedModeSPacket(const DecodedModeSPacket& decoded_packet) {
    if (!canbus_initialized || !efis_connected) {
        return;
    }
    if (decoded_packet.raw.buffer_len_bytes != RawModeSPacket::kExtendedSquitterPacketLenBytes) {
        return;
    }

    ModeSADSBPacket ads_b_packet(decoded_packet);
    if (!ads_b_packet.is_valid) {
        return;
    }

    switch (ads_b_packet.downlink_format) {
        case ModeSADSBPacket::kDownlinkFormatExtendedSquitter:
        case ModeSADSBPacket::kDownlinkFormatExtendedSquitterNonTransponder:
            break;
        default:
            return;
    }

    if (ads_b_packet.type_code == ModeSADSBPacket::kTypeCodeInvalid) {
        return;
    }

    uint32_t uid = Aircraft::ICAOToUID(ads_b_packet.icao_address, Aircraft::kAircraftTypeModeS);
    ModeSAircraft aircraft;
    if (!adsbee_server.aircraft_dictionary.GetAircraft(uid, aircraft)) {
        return;
    }

    ReportCANModeSTraffic(uid, static_cast<uint8_t>(ads_b_packet.type_code), aircraft);
}

void ReportCANFromIngestedUATPacket(const DecodedUATADSBPacket& decoded_packet) {
    if (!canbus_initialized || !efis_connected) {
        return;
    }

    uint32_t uid = Aircraft::ICAOToUID(decoded_packet.GetICAOAddress(), Aircraft::kAircraftTypeUAT);
    UATAircraft aircraft;
    if (!adsbee_server.aircraft_dictionary.GetAircraft(uid, aircraft)) {
        return;
    }

    if (decoded_packet.has_state_vector) {
        ReportCANUATTraffic(uid, UAT_STATE_VECTOR, aircraft);
    }
    if (decoded_packet.has_mode_status) {
        ReportCANUATTraffic(uid, UAT_IDENT, aircraft);
    }
}

bool CanbusInit(can_termination_t term_res_enable) {
    CONSOLE_INFO("CAN_INIT", "Initializing MCP251863...");

    serial_num = build_radbus_serial_num();
    provisional_src = radbus_serial_src_hash(serial_num);
    RADbus_UID = kRadbusUidUnassigned;
    efis_connected = false;
    CONSOLE_INFO("CAN_INIT", "RADbus serial=0x%08lX provisional_src=0x%02X",
                 (unsigned long)serial_num, provisional_src);

    // Configure selectable termination resistor GPIO control
    gpio_config_t term_res_gpio = {
        .pin_bit_mask = (1ULL << CAN_TERM_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&term_res_gpio);
    CanbusSetTermination(term_res_enable);
    
    // Configure and drive standby pin to wake device
    gpio_config_t standby_gpio = {
        .pin_bit_mask = (1ULL << CAN_STANDBY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&standby_gpio);
    // Drive standby pin LOW to activate device (XSTBY is active low)
    gpio_set_level(CAN_STANDBY_PIN, 0);

    // INT1 is active-low push-pull (no external pull). Idle high: ESP pull-up + MCP drives high.
    gpio_config_t can_rx_int = {
        .pin_bit_mask = (1ULL << CAN_RX_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&can_rx_int);

    vTaskDelay(pdMS_TO_TICKS(5));

    // Initialize MCP251XFD device with CAN-FD configuration
    // Note: SPI bus initialization is handled by MCP251XFD_SPI_Init callback
    MCP251XFD_Config mcp_config = {
        // Controller clocks
        .XtalFreq = 40000000,  // 40MHz crystal
        .OscFreq = 0,          // Not using external oscillator
        .SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN,  // SYSCLK = CLKIN (no PLL) = 40MHz
        .ClkoPinConfig = MCP251XFD_CLKO_DivBy10,
        .SYSCLK_Result = NULL,
        
        // CAN-FD configuration
        .NominalBitrate = 1000000,  // 1 Mbps nominal bitrate
        .DataBitrate = 4000000,     // 4 Mbps data bitrate (CAN-FD)
        .BitTimeStats = NULL,
        .Bandwidth = MCP251XFD_NO_DELAY,
        .ControlFlags = MCP251XFD_CANFD_USE_ISO_CRC,  // ISO CAN-FD (ISO 11898-1:2015 compliant)
        
        // GPIOs and Interrupts pins
        .GPIO0PinMode = MCP251XFD_PIN_AS_INT0_TX,
        .GPIO1PinMode = MCP251XFD_PIN_AS_INT1_RX,
        .INTsOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
        .TXCANOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
        
        // Interrupts
        .SysInterruptFlags = MCP251XFD_INT_RX_EVENT,
    };
    
    CONSOLE_INFO("CAN_INIT", "Initializing MCP251XFD with CAN-FD (1 Mbps / 4 Mbps, ISO)...");
    eERRORRESULT init_result = Init_MCP251XFD(&MCP251XFD_Ext1, &mcp_config);
    
    if (init_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to initialize MCP251XFD: Error code %d", init_result);
        return false;
    }
    
    CONSOLE_INFO("CAN_INIT", "MCP251863 initialized successfully");
    
    // Configure timestamp (must be done before FIFO configuration)
    // Enable timestamps with SOF sample point, prescaler = 1 (40MHz / 1 = 40MHz timestamp clock)
    eERRORRESULT ts_result = MCP251XFD_ConfigureTimeStamp(
        &MCP251XFD_Ext1,
        true,  // Enable timestamps
        MCP251XFD_TS_CAN20_SOF_CANFD_SOF,  // Timestamp at SOF for both CAN2.0 and CAN-FD
        1,     // Prescaler = 1 (timestamp clock = 40MHz / 1 = 40MHz)
        false  // Don't use interrupt-based counter
    );
    if (ts_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to configure timestamp: Error code %d", ts_result);
        return false;
    }
    CONSOLE_INFO("CAN_INIT", "Timestamp configured");
    
    // Configure FIFOs using list from fifo_conf.h (includes TEF, FIFO1, FIFO2)
    eERRORRESULT fifo_list_result = MCP251XFD_ConfigureFIFOList(&MCP251XFD_Ext1, Ext1_FIFOlist, EXT1_FIFO_COUNT);
    if (fifo_list_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to configure FIFO list: Error code %d", fifo_list_result);
        return false;
    }
    CONSOLE_INFO("CAN_INIT", "FIFOs configured - TEF at 0x%04X, RX FIFO1 at 0x%04X, TX FIFO2 at 0x%04X",
        Ext1_TEF_RAMInfos.RAMStartAddress,
        Ext1_FIFO1_RAMInfos.RAMStartAddress,
        Ext1_FIFO2_RAMInfos.RAMStartAddress);
    
    // Hardware RX filter: match message type only in packed 29-bit EXT IDs (src/dest ignored).
    // Destination filtering is done in software in process_rx_msg().
    MCP251XFD_Filter radbus_rx_filters[] = {
        {
            .Filter = MCP251XFD_FILTER0,
            .EnableFilter = true,
            .Match = MCP251XFD_MATCH_ONLY_EID,
            .PointTo = MCP251XFD_FIFO1,
            .AcceptanceID = can_pack_id(CANMSG_ADSB_OWNSHIP_STATE, 0, 0),
            .AcceptanceMask = kCanMsgTypeFilterMask,
            .ExtendedID = true,
        },
        {
            .Filter = MCP251XFD_FILTER1,
            .EnableFilter = true,
            .Match = MCP251XFD_MATCH_ONLY_EID,
            .PointTo = MCP251XFD_FIFO1,
            .AcceptanceID = can_pack_id(CANMSG_ADSB_OWNSHIP_IDENT, 0, 0),
            .AcceptanceMask = kCanMsgTypeFilterMask,
            .ExtendedID = true,
        },
        {
            .Filter = MCP251XFD_FILTER2,
            .EnableFilter = true,
            .Match = MCP251XFD_MATCH_ONLY_EID,
            .PointTo = MCP251XFD_FIFO1,
            .AcceptanceID = can_pack_id(CANMSG_UID_ASSIGN, 0, 0),
            .AcceptanceMask = kCanMsgTypeFilterMask,
            .ExtendedID = true,
        },
        {
            .Filter = MCP251XFD_FILTER3,
            .EnableFilter = true,
            .Match = MCP251XFD_MATCH_ONLY_EID,
            .PointTo = MCP251XFD_FIFO1,
            .AcceptanceID = can_pack_id(CANMSG_DISCONNECT_CLIENT, 0, 0),
            .AcceptanceMask = kCanMsgTypeFilterMask,
            .ExtendedID = true,
        },
    };

    eERRORRESULT filter_result = MCP251XFD_ConfigureFilterList(
        &MCP251XFD_Ext1,
        MCP251XFD_D_NET_FILTER_DISABLE,
        radbus_rx_filters,
        sizeof(radbus_rx_filters) / sizeof(radbus_rx_filters[0]));
    if (filter_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to configure filter list: Error code %d", filter_result);
        return false;
    }
    CONSOLE_INFO("CAN_INIT", "RX EXT filters: msg 0x%03X/0x%03X/0x%03X/0x%03X (provisional_src=0x%02X)",
                 CANMSG_ADSB_OWNSHIP_STATE, CANMSG_ADSB_OWNSHIP_IDENT, CANMSG_UID_ASSIGN,
                 CANMSG_DISCONNECT_CLIENT, provisional_src);
    
    // Start CAN bus in CAN-FD mode
    eERRORRESULT start_result = MCP251XFD_StartCANFD(&MCP251XFD_Ext1);
    if (start_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to start CAN bus: Error code %d", start_result);
        return false;
    }

    drain_rx_fifo();

    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret == ESP_ERR_INVALID_STATE) {
        // Already installed (e.g. by EthernetInit on the shared aux SPI bus).
    } else if (isr_ret != ESP_OK) {
        CONSOLE_ERROR("CAN_INIT", "GPIO ISR service install failed: %s", esp_err_to_name(isr_ret));
        return false;
    }

    gpio_set_intr_type(CAN_RX_INT_PIN, GPIO_INTR_NEGEDGE);
    isr_ret = gpio_isr_handler_add(CAN_RX_INT_PIN, can_rx_gpio_isr, nullptr);
    if (isr_ret != ESP_OK) {
        CONSOLE_ERROR("CAN_INIT", "GPIO ISR handler add failed: %s", esp_err_to_name(isr_ret));
        return false;
    }

    canbus_initialized = true;
    last_heartbeat = 0;  // Force first CanbusUpdate() to send heartbeat / UID request.
    CONSOLE_INFO("CAN_INIT", "MCP251863 ready");
    return true;
}



void transmit_can(uint16_t msg_type, uint8_t dest, eMCP251XFD_DataLength DLC, uint8_t *data) {

    tx_message.MessageID = can_pack_id(msg_type, radbus_tx_src(), dest);
    tx_message.DLC = DLC;
    tx_message.PayloadData = data;
    tx_message.ControlFlags = (setMCP251XFD_MessageCtrlFlags)(
        MCP251XFD_CANFD_FRAME | MCP251XFD_SWITCH_BITRATE | MCP251XFD_EXTENDED_MESSAGE_ID);

    eERRORRESULT tx_result = MCP251XFD_TransmitMessageToFIFO(
        &MCP251XFD_Ext1,
        &tx_message,
        MCP251XFD_FIFO2,
        true  // Flush immediately
    );

    if (tx_result != ERR_OK) {
        CONSOLE_ERROR("CAN", "Failed to transmit message: Error code %d", tx_result);
    }
}

void check_can_errors() {
    uint8_t tx_err = 0, rx_err = 0;
    eMCP251XFD_TXRXErrorStatus err_status = MCP251XFD_RX_NO_ERROR;
    eERRORRESULT err_result = MCP251XFD_GetTransmitReceiveErrorCountAndStatus(&MCP251XFD_Ext1, &tx_err, &rx_err, &err_status);
    if (err_result != ERR_OK) {
        CONSOLE_ERROR("CAN", "Error counters: TX=%d, RX=%d, Status=0x%02X", tx_err, rx_err, err_status);
    }
}



eERRORRESULT MCP251XFD_SPI_Init(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq)
{
    esp_err_t ret;
    
    // Update clock speed if different
    bool need_reinit = false;
    if (devcfg.clock_speed_hz != sckFreq) {
        devcfg.clock_speed_hz = sckFreq;
        need_reinit = true;
    }
    
    // If device exists and we need to change clock, remove it first
    if (can_spi_handle != NULL && need_reinit) {
        spi_bus_remove_device(can_spi_handle);
        can_spi_handle = NULL;
    }
    
    // Initialize SPI bus if not already initialized (e.g. by W5500 Ethernet on the same aux bus).
    if (can_spi_handle == NULL) {
        ret = spi_bus_initialize(CAN_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            CONSOLE_ERROR("CAN_INIT", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return ERR__SPI_FREQUENCY_ERROR;
        }

        ret = spi_bus_add_device(CAN_SPI_HOST, &devcfg, &can_spi_handle);
        if (ret != ESP_OK) {
            CONSOLE_ERROR("CAN_INIT", "Failed to add SPI device: %s", esp_err_to_name(ret));
            return ERR__SPI_FREQUENCY_ERROR;
        }
    }
    
    return ERR_OK;
}



eERRORRESULT MCP251XFD_SPI_Transfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size)
{
    if (can_spi_handle == NULL) {
        CONSOLE_ERROR("CAN_INIT", "SPI handle is NULL, bus not initialized");
        return ERR__SPI_FREQUENCY_ERROR;
    }
    
    if (txData == NULL) {
        CONSOLE_ERROR("CAN_INIT", "SPI transfer: txData is NULL");
        return ERR__SPI_PARAMETER_ERROR;
    }

    if (size == 0 || size > sizeof(s_can_spi_dma_buf)) {
        CONSOLE_ERROR("CAN_INIT", "SPI transfer size %u out of range (max %u)", (unsigned)size,
                      (unsigned)sizeof(s_can_spi_dma_buf));
        return ERR__SPI_PARAMETER_ERROR;
    }

    memcpy(s_can_spi_dma_buf, txData, size);

    // ESP-IDF SPI driver handles CS automatically based on devcfg.spics_io_num
    // The chipSelect parameter from MCP251XFD is ignored since we use fixed CS pin
    spi_transaction_t trans = {
        .length = size * 8,  // Length in bits
        .tx_buffer = s_can_spi_dma_buf,
        .rx_buffer = s_can_spi_dma_buf,
    };
    
    esp_err_t ret = spi_device_transmit(can_spi_handle, &trans);
    if (ret != ESP_OK) {
        CONSOLE_ERROR("CAN_INIT", "SPI transfer failed: %s", esp_err_to_name(ret));
        return ERR__SPI_FREQUENCY_ERROR;
    }

    uint8_t *readback = rxData != nullptr ? rxData : txData;
    memcpy(readback, s_can_spi_dma_buf, size);
    
    return ERR_OK;
}
