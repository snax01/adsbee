
#include "comms.hh"

#include "comms_canbus.hh"
#include "mcp251863/fifo_conf.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

// ADSBEE INCLUDES
#include "adsbee_server.hh"         // This gives us access to adsbee_server.aircraft_dictionary...
#include "aircraft_dictionary.hh"
#include "task_priorities.hh"
#include "object_dictionary.hh"
extern GDL90Reporter gdl90;
// ADSBEE INCLUDES

// Buffer for received message data
uint8_t rx_data[64];
MCP251XFD_CANMessage rx_message = {
    .PayloadData = rx_data,
};

// Buffer for transmit message data
uint8_t tx_data[64] = {0};
MCP251XFD_CANMessage tx_message = {
    .MessageID = 0x123,
    .MessageSEQ = 0,
    .ControlFlags = (setMCP251XFD_MessageCtrlFlags)(MCP251XFD_CANFD_FRAME | MCP251XFD_SWITCH_BITRATE),  // CAN-FD frame with BRS enabled
    .DLC = MCP251XFD_DLC_2BYTE,
    .PayloadData = tx_data,
};


#define SPI_HOST_ID         SPI2_HOST  // Using SPI2 (HSPI) - SPI0/SPI1 are typically used for flash/PSRAM

static spi_device_handle_t  can_spi_handle = NULL;

#define CAN_STANDBY_PIN     GPIO_NUM_12
#define CAN_TERM_ENABLE     GPIO_NUM_21

#define CAN_RX_INT_PIN      GPIO_NUM_47 // Msg received interrupt
#define CAN_INT_PIN         GPIO_NUM_48 // General Interrupts from MCP251863

#define SPI_MOSI_PIN        GPIO_NUM_14
#define SPI_MISO_PIN        GPIO_NUM_13
#define SPI_SCLK_PIN        GPIO_NUM_17
#define SPI_CS_PIN          GPIO_NUM_18

#define SPI_CLOCK           8000000    // 8 MHz (slowed down for 20MHz crystal)

#define CANMSG_ADSB_AIRCRAFT_OUT    0x120
#define CANMSG_ADSB_OWNSHIP_STATE   0x130
#define CANMSG_ADSB_OWNSHIP_IDENT   0x140
#define CANMSG_ADSB_WAKEUP          0x150

#define CANMSG_HEARTBEAT            0x750
#define CANMSG_UID_REQUEST          0x760
#define CANMSG_UID_ASSIGN           0x770

// Buffer for received message data
uint8_t rx_data[64];
MCP251XFD_CANMessage rx_message = {
    .PayloadData = rx_data,
};

// Buffer for transmit message data
uint8_t tx_data[64] = {0};
MCP251XFD_CANMessage tx_message = {
    .MessageID = 0x123,
    .MessageSEQ = 0,
    .ControlFlags = (setMCP251XFD_MessageCtrlFlags)(MCP251XFD_CANFD_FRAME | MCP251XFD_SWITCH_BITRATE),  // CAN-FD frame with BRS enabled
    .DLC = MCP251XFD_DLC_2BYTE,
    .PayloadData = tx_data,
};

bool volatile efis_connected = false;
uint8_t RADbus_UID = 0xFF;
uint32_t last_heartbeat = 0;
uint32_t serial_num = 0xFF123456;

uint32_t time_since_zulu;

extern QueueHandle_t CAN_msg_tx_queue;
TaskHandle_t canbus_task_handle = NULL;


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
    .max_transfer_sz = 4096,
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



GDL90Reporter::GDL90TargetReportData ownship_data = { .traffic_alert_status = 0,
                                                      .address_type = GDL90Reporter::GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress, 
                                                      .participant_address = 0x111111,
                                                      .misc_indicators = 0,
                                                      .emergency_priority_code = GDL90Reporter::GDL90TargetReportData::kEmergencyPriorityCodeNoEmergency,
                                                     };


static bool check_rx_fifo(void)
{
    setMCP251XFD_FIFOstatus st = MCP251XFD_RX_FIFO_EMPTY;
    if (MCP251XFD_GetFIFOStatus(&MCP251XFD_Ext1, MCP251XFD_FIFO1, &st) != ERR_OK) {
        return false;
    }
    return (st & MCP251XFD_RX_FIFO_NOT_EMPTY) != 0;
}

void process_rx_msg()   {
    uint32_t ts;
    eERRORRESULT e = MCP251XFD_ReceiveMessageFromFIFO(
        &MCP251XFD_Ext1,
        &rx_message,
        MCP251XFD_PAYLOAD_64BYTE,   // must match fifo_conf.h for FIFO1
        &ts,                        // NULL if you don't want timestamp, need to change in fifo_conf if i want to remove timestamp
        MCP251XFD_FIFO1);


    if (rx_message.MessageID == CANMSG_ADSB_OWNSHIP_STATE)  {
        memcpy(&time_since_zulu, &rx_data[0], sizeof(uint32_t));
        memcpy(&ownship_data.latitude_deg, &rx_data[4], sizeof(float));
        memcpy(&ownship_data.longitude_deg, &rx_data[8], sizeof(float));
        memcpy(&ownship_data.altitude_ft, &rx_data[12], sizeof(int32_t));
        memcpy(&ownship_data.speed_kts, &rx_data[16], sizeof(float));
        memcpy(&ownship_data.direction_deg, &rx_data[20], sizeof(float));
        memcpy(&ownship_data.vertical_rate_fpm, &rx_data[24], sizeof(int32_t));
        if (time_since_zulu != 0)   {   // Should I have a better way to identify when these flags should flip?
            gdl90.utc_timing_is_valid = true;
            gdl90.gnss_position_valid = true;
        }
        CONSOLE_INFO("RADbus", "STATE ownship received.");
        return;
    }

    // TODO - need to set gnss_position_valid and utc_timing_is_valid if RADbus data is valid.
    if (rx_message.MessageID == CANMSG_ADSB_OWNSHIP_IDENT)  {
        memcpy(&ownship_data.callsign, &rx_data[0], 8);
        memcpy(&ownship_data.participant_address, &rx_data[8], 4);
        memcpy(&ownship_data.address_type, &rx_data[12], 1);
        memcpy(&ownship_data.emitter_category, &rx_data[13], 1);
        memcpy(&ownship_data.navigation_integrity_category, &rx_data[14], 1);
        memcpy(&ownship_data.navigation_accuracy_category_position, &rx_data[15], 1);
        memcpy(&ownship_data.misc_indicators, &rx_data[16], 1);
        CONSOLE_INFO("RADbus", "IDENT ownship received.");
        return;
    }

    if (rx_message.MessageID == CANMSG_UID_ASSIGN)  {
        RADbus_UID = rx_data[0];
        efis_connected = true;
        CONSOLE_WARNING("RADbus", "UID assign received.");
        return;
    }
}


void canbus_task(void* pvParameters)    {
    CanbusInit(CAN_TERM_ON);

    queue_msg_t queue_rx_buf = { 0 };

    while (1)   {

        vTaskDelay(pdMS_TO_TICKS(10));

        if (last_heartbeat + 1000 <=  get_time_since_boot_ms()) {
            transmit_can(CANMSG_HEARTBEAT, MCP251XFD_DLC_1BYTE, &RADbus_UID);
            last_heartbeat = get_time_since_boot_ms();
        }

        if (gpio_get_level(CAN_RX_INT_PIN) == 0)    {
            while (check_rx_fifo())    {
                process_rx_msg();
            }
        }

        if (efis_connected) {
            if (xQueueReceive(CAN_msg_tx_queue, &queue_rx_buf, 0) == pdPASS) {
            ModeSAircraft aircraft_mode_s;
            UATAircraft aircraft_uat;

            if ( ( static_cast<uint8_t>(queue_rx_buf.packet_type) ) >= 1 && ( static_cast<uint8_t>(queue_rx_buf.packet_type) <= 31 ) )  {
                adsbee_server.aircraft_dictionary.GetAircraft(queue_rx_buf.uid, aircraft_mode_s);
            }   else if ( ( static_cast<uint8_t>(queue_rx_buf.packet_type) ) >= 40 && ( static_cast<uint8_t>(queue_rx_buf.packet_type) <= 44 ) ) {
                adsbee_server.aircraft_dictionary.GetAircraft(queue_rx_buf.uid, aircraft_uat);
            }   else {
                break;
            }
            

            switch (static_cast<uint8_t>(queue_rx_buf.packet_type)) {
                case MODE_S_IDENT:      // TC = 1 (Aircraft Identification)
                case MODE_S_IDENT + 1:  // TC = 2 (Aircraft Identification)
                case MODE_S_IDENT + 2:  // TC = 3 (Aircraft Identification)
                case MODE_S_IDENT + 3: {// TC = 4 (Aircraft Identification)
                    uint8_t tx_buf[16] = { 0 };
                    tx_buf[0] = queue_rx_buf.packet_type;
                    memcpy(&tx_buf[1], &queue_rx_buf.uid, 4);
                    memcpy(&tx_buf[5], &aircraft_mode_s.callsign, 8);
                    tx_buf[13] = aircraft_mode_s.emitter_category;

                    transmit_can(CANMSG_ADSB_AIRCRAFT_OUT, MCP251XFD_DLC_16BYTE, &tx_buf[0]);
                    break;
                }

                case MODE_S_SURFACE:      // TC = 5 (Surface Position)
                case MODE_S_SURFACE + 1:  // TC = 6 (Surface Position)
                case MODE_S_SURFACE + 2:  // TC = 7 (Surface Position)
                case MODE_S_SURFACE + 3:  // TC = 8 (Surface Position)

                    break;

                case MODE_S_POSITION_BARO:      // TC = 9 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 1:  // TC = 10 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 2:  // TC = 11 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 3:  // TC = 12 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 4:  // TC = 13 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 5:  // TC = 14 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 6:  // TC = 15 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 7:  // TC = 16 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 8:  // TC = 17 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_BARO + 9:  // TC = 18 (Airborne Position w/ Baro Altitude)
                case MODE_S_POSITION_GNSS:      // TC = 20 (Airborne Position w/ GNSS Altitude)
                case MODE_S_POSITION_GNSS + 1:  // TC = 21 (Airborne Position w/ GNSS Altitude)
                case MODE_S_POSITION_GNSS + 2: { // TC = 22 (Airborne Position w/ GNSS Altitude)
                    uint8_t tx_buf[20] = { 0 };
                    tx_buf[0] = queue_rx_buf.packet_type;
                    memcpy(&tx_buf[1], &queue_rx_buf.uid, 4);
                    memcpy(&tx_buf[5], &aircraft_mode_s.latitude_deg, 4);
                    memcpy(&tx_buf[9], &aircraft_mode_s.longitude_deg, 4);
                    memcpy(&tx_buf[13], &aircraft_mode_s.baro_altitude_ft, 4);

                    // TODO - Transmit GNSS Altitude if TC == 20 - 22
                    transmit_can(CANMSG_ADSB_AIRCRAFT_OUT, MCP251XFD_DLC_20BYTE, &tx_buf[0]);
                    break;
                }

                case MODE_S_VELOCITY: {  // TC = 19 (Airborne Velocities)
                    uint8_t tx_buf[20] = { 0 };
                    tx_buf[0] = queue_rx_buf.packet_type;
                    memcpy(&tx_buf[1], &queue_rx_buf.uid, 4);
                    memcpy(&tx_buf[5], &aircraft_mode_s.direction_deg, 4);
                    memcpy(&tx_buf[9], &aircraft_mode_s.baro_vertical_rate_fpm, 4);
                    memcpy(&tx_buf[13], &aircraft_mode_s.speed_kts, 4);
                    transmit_can(CANMSG_ADSB_AIRCRAFT_OUT, MCP251XFD_DLC_20BYTE, &tx_buf[0]);
                    break;
                }

                case MODE_S_RESERVED:      // TC = 23 (Reserved)
                case MODE_S_RESERVED + 1:  // TC = 24 (Reserved)
                case MODE_S_RESERVED + 2:  // TC = 25 (Reserved)
                case MODE_S_RESERVED + 3:  // TC = 26 (Reserved)
                case MODE_S_RESERVED + 4:  // TC = 27 (Reserved)

                    break;

                case MODE_S_AIRCRAFT_STATUS:  // TC = 28 (Aircraft Status)

                    break;

                case MODE_S_TARGET_STATE:  // TC = 29 (Target state and status info)

                    break;

                case MODE_S_OPERATION_STATUS:  // TC = 31 (Aircraft operation status)

                    break;

                

                case UAT_IDENT: {
                    uint8_t tx_buf[16] = { 0 };
                    tx_buf[0] = queue_rx_buf.packet_type;
                    memcpy(&tx_buf[1], &queue_rx_buf.uid, 4);
                    memcpy(&tx_buf[5], &aircraft_uat.callsign, 8);
                    tx_buf[13] = aircraft_uat.emitter_category;

                    transmit_can(CANMSG_ADSB_AIRCRAFT_OUT, MCP251XFD_DLC_16BYTE, &tx_buf[0]);
                    break;
                }

                case UAT_STATE_VECTOR:  {
                    uint8_t tx_buf[30] = { 0 };
                    tx_buf[0] = queue_rx_buf.packet_type;
                    memcpy(&tx_buf[1], &queue_rx_buf.uid, 4);
                    memcpy(&tx_buf[5], &aircraft_uat.latitude_deg, 4);
                    memcpy(&tx_buf[9], &aircraft_uat.longitude_deg, 4);
                    memcpy(&tx_buf[13], &aircraft_uat.baro_altitude_ft, 4);
                    memcpy(&tx_buf[17], &aircraft_uat.direction_deg, 4);
                    memcpy(&tx_buf[21], &aircraft_uat.baro_vertical_rate_fpm, 4);
                    memcpy(&tx_buf[25], &aircraft_uat.speed_kts, 4);

                    // TODO - Transmit GNSS Altitude if valid
                    transmit_can(CANMSG_ADSB_AIRCRAFT_OUT, MCP251XFD_DLC_32BYTE, &tx_buf[0]);
                    break;
                }
                
                case UAT_AUXILIARY_STATE_VECTOR:
                    break;
                case UAT_TARGET_STATE:
                    break;
                case UAT_TRAJECTORY_CHANGE:
                    break;


                default:
                    CONSOLE_WARNING("CAN::IngestModeSADSBPacket",
                                    "Received ADSB message with unsupported type code %d for UID 0x%lX.", queue_rx_buf.packet_type,
                                    queue_rx_buf.uid);

                }
            }
        }

        // TODO - check canbus error status at 1hz
        //check_can_errors();

    }   // End of can loop
}


bool CanbusInit(can_termination_t term_res_enable)   {
    CONSOLE_INFO("CAN_INIT", "Initializing MCP251863...");

    // Configure selectable termination resistor GPIO control
    gpio_config_t term_res_gpio = {
        .pin_bit_mask = (1ULL << CAN_TERM_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&term_res_gpio);
    gpio_set_level(CAN_TERM_ENABLE, term_res_enable);
    
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

    // Configure CAN RX Interrupt pin
    gpio_config_t can_rx_int = {
        .pin_bit_mask = (1ULL << CAN_RX_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&can_rx_int);

/*     esp_err_t ret = gpio_install_isr_service(0);
    if (ret == ESP_ERR_INVALID_STATE) {
        // ISR handler has been already installed so no issues
        CONSOLE_INFO("Comms_canbus", "GPIO ISR handler has been already installed");
    } else if (ret != ESP_OK) {
        CONSOLE_ERROR("Comms_canbus", "GPIO ISR handler install failed");
        return false;
    } */


    CAN_msg_tx_queue = xQueueCreate(100, sizeof(queue_msg_t));
    

    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Initialize MCP251XFD device with CAN-FD configuration
    // Note: SPI bus initialization is handled by MCP251XFD_SPI_Init callback
    MCP251XFD_Config mcp_config = {
        // Controller clocks
        .XtalFreq = 20000000,  // 40MHz crystal // TODO - Change to 40MHz crystal and update this in v2
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
    
    // Configure filter list to accept all messages
    MCP251XFD_Filter filter_list[] = {
        {
            .Filter = MCP251XFD_FILTER0,
            .EnableFilter = true,
            .Match = MCP251XFD_MATCH_SID_EID,
            .PointTo = MCP251XFD_FIFO1,
            .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES,
            .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES,
            .ExtendedID = false,
        },
    };
    
    eERRORRESULT filter_result = MCP251XFD_ConfigureFilterList(
        &MCP251XFD_Ext1,
        MCP251XFD_D_NET_FILTER_DISABLE,  // Disable DeviceNet filter
        filter_list,
        sizeof(filter_list) / sizeof(filter_list[0])
    );
    if (filter_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to configure filter list: Error code %d", filter_result);
        return false;
    }
    CONSOLE_INFO("CAN_INIT", "Filters configured");
    
    // Start CAN bus in CAN-FD mode
    eERRORRESULT start_result = MCP251XFD_StartCANFD(&MCP251XFD_Ext1);
    if (start_result != ERR_OK) {
        CONSOLE_ERROR("CAN_INIT", "Failed to start CAN bus: Error code %d", start_result);
        return false;
    }

    //gpio_isr_handler_add(CAN_RX_INT_PIN, CANRX_isr_handler, NULL);

    while(!efis_connected)  {

        uint8_t UID_request[8];
        memcpy(&UID_request[0], &serial_num, sizeof(uint32_t));
        memcpy(&UID_request[4], &ObjectDictionary::kFirmwareVersion, sizeof(uint32_t));
        
        vTaskDelay(pdMS_TO_TICKS(100));

        if (last_heartbeat + 1000 <=  get_time_since_boot_ms()) {
            transmit_can(CANMSG_UID_REQUEST, MCP251XFD_DLC_8BYTE, &UID_request[0]);
            last_heartbeat = get_time_since_boot_ms();
        }

        if (gpio_get_level(CAN_RX_INT_PIN) == 0)    {
            while (check_rx_fifo())    {
                process_rx_msg();
            }
        }
    }
    
    return true;
}

void transmit_can(uint32_t canID, eMCP251XFD_DataLength DLC, uint8_t *data) {

    tx_message.MessageID = canID;
    tx_message.DLC = DLC;
    tx_message.PayloadData = data;

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
    
    // Initialize SPI bus if not already initialized
    if (can_spi_handle == NULL) {
        ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret == ESP_ERR_INVALID_STATE) {
            // Bus already initialized
        } else if (ret != ESP_OK) {
            CONSOLE_ERROR("CAN_INIT", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return ERR__SPI_FREQUENCY_ERROR;
        }
        
        // Add device to SPI bus
        ret = spi_bus_add_device(SPI3_HOST, &devcfg, &can_spi_handle);
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
    
    // ESP-IDF SPI driver handles CS automatically based on devcfg.spics_io_num
    // The chipSelect parameter from MCP251XFD is ignored since we use fixed CS pin
    spi_transaction_t trans = {
        .length = size * 8,  // Length in bits
        .tx_buffer = txData,
        .rx_buffer = rxData,
    };
    
    esp_err_t ret = spi_device_transmit(can_spi_handle, &trans);
    if (ret != ESP_OK) {
        CONSOLE_ERROR("CAN_INIT", "SPI transfer failed: %s", esp_err_to_name(ret));
        return ERR__SPI_FREQUENCY_ERROR;
    }
    
    return ERR_OK;
}