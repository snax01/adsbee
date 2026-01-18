
#include "comms.hh"

#include "canbus.hh"
#include "driver/fifo_conf.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

// Buffer for received message data
uint8_t rx_data[64];
MCP251XFD_CANMessage rx_message = {
    .PayloadData = rx_data,
};

// Buffer for transmit message data
uint8_t tx_data[2] = {0xEF, 0x15};
MCP251XFD_CANMessage tx_message = {
    .MessageID = 0x123,
    .MessageSEQ = 0,
    .ControlFlags = (setMCP251XFD_MessageCtrlFlags)(MCP251XFD_CANFD_FRAME | MCP251XFD_SWITCH_BITRATE),  // CAN-FD frame with BRS enabled
    .DLC = MCP251XFD_DLC_2BYTE,
    .PayloadData = tx_data,
};

static spi_device_handle_t spi_handle = NULL;

#define CAN_STANDBY_PIN GPIO_NUM_48

// SPI Bus Configuration Structure
static spi_bus_config_t buscfg = {
    .mosi_io_num = GPIO_NUM_14,
    .miso_io_num = GPIO_NUM_13,
    .sclk_io_num = GPIO_NUM_17,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
};

// SPI Device Configuration Structure
static spi_device_interface_config_t devcfg = {
    .mode = 0,
    .clock_speed_hz = 10000000,  // 10 MHz (slowed down for testing)
                        // SPI mode 0 (CPOL=0, CPHA=0)
    .spics_io_num = GPIO_NUM_18,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL
};


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
    if (spi_handle != NULL && need_reinit) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    
    // Initialize SPI bus if not already initialized
    if (spi_handle == NULL) {
        ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret == ESP_ERR_INVALID_STATE) {
            // Bus already initialized
        } else if (ret != ESP_OK) {
            CONSOLE_ERROR("CanbusInit", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return ERR__SPI_FREQUENCY_ERROR;
        }
        
        // Add device to SPI bus
        ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle);
        if (ret != ESP_OK) {
            CONSOLE_ERROR("CanbusInit", "Failed to add SPI device: %s", esp_err_to_name(ret));
            return ERR__SPI_FREQUENCY_ERROR;
        }
    }
    
    return ERR_OK;
}


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
eERRORRESULT MCP251XFD_SPI_Transfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size)
{
    if (spi_handle == NULL) {
        CONSOLE_ERROR("CanbusInit", "SPI handle is NULL, bus not initialized");
        return ERR__SPI_FREQUENCY_ERROR;
    }
    
    if (txData == NULL) {
        CONSOLE_ERROR("CanbusInit", "SPI transfer: txData is NULL");
        return ERR__SPI_PARAMETER_ERROR;
    }
    
    // ESP-IDF SPI driver handles CS automatically based on devcfg.spics_io_num
    // The chipSelect parameter from MCP251XFD is ignored since we use fixed CS pin
    spi_transaction_t trans = {
        .length = size * 8,  // Length in bits
        .tx_buffer = txData,
        .rx_buffer = rxData,
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        CONSOLE_ERROR("CanbusInit", "SPI transfer failed: %s", esp_err_to_name(ret));
        return ERR__SPI_FREQUENCY_ERROR;
    }
    
    return ERR_OK;
}

/**
 * @brief Get current time in milliseconds
 * 
 * @return Current time in milliseconds
 */
uint32_t MCP251XFD_GetCurrentms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint16_t MCP251XFD_ComputeCRC16(const uint8_t* data, size_t size)
{
    // Not used when CRC mode is disabled
    (void)data;
    (void)size;
    return 0;
}

// MCP251XFD Device Configuration
MCP251XFD MCP251XFD_Ext1 = 
{ 
  .UserDriverData  = NULL, 
  //--- Driver configuration --- 
  .DriverConfig    = MCP251XFD_DRIVER_NORMAL_USE, 
  //--- IO configuration --- 
  .GPIOsOutLevel   = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH, 
  //--- Interface driver call functions --- 
  .SPI_ChipSelect  = 0,                    // Chip select index (not used with ESP-IDF SPI driver)
  .InterfaceDevice = &spi_handle,         // Pointer to SPI device handle
  //--- Interface clocks --- 
  .SPIClockSpeed   = 10000000, // 10MHz 
  
  .fnSPI_Init      = MCP251XFD_SPI_Init, 
  .fnSPI_Transfer  = MCP251XFD_SPI_Transfer, 
  //--- Time call function --- 
  .fnGetCurrentms  = MCP251XFD_GetCurrentms, 
  //--- CRC16-CMS call function --- 
  .fnComputeCRC16  = MCP251XFD_ComputeCRC16, 
  
};


bool CanbusInit()   {
    CONSOLE_INFO("CanbusInit", "Initializing MCP251863...");
    
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
    vTaskDelay(pdMS_TO_TICKS(10));
    
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
        .SysInterruptFlags = MCP251XFD_INT_NO_EVENT,
    };
    
    CONSOLE_INFO("CanbusInit", "Initializing MCP251XFD with CAN-FD (1 Mbps / 4 Mbps, ISO)...");
    eERRORRESULT init_result = Init_MCP251XFD(&MCP251XFD_Ext1, &mcp_config);
    
    if (init_result != ERR_OK) {
        CONSOLE_ERROR("CanbusInit", "Failed to initialize MCP251XFD: Error code %d", init_result);
        return false;
    }
    
    CONSOLE_INFO("CanbusInit", "MCP251863 initialized successfully");
    
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
        CONSOLE_ERROR("CanbusInit", "Failed to configure timestamp: Error code %d", ts_result);
        return false;
    }
    CONSOLE_INFO("CanbusInit", "Timestamp configured");
    
    // Configure FIFOs using list from fifo_conf.h (includes TEF, FIFO1, FIFO2)
    eERRORRESULT fifo_list_result = MCP251XFD_ConfigureFIFOList(&MCP251XFD_Ext1, Ext1_FIFOlist, EXT1_FIFO_COUNT);
    if (fifo_list_result != ERR_OK) {
        CONSOLE_ERROR("CanbusInit", "Failed to configure FIFO list: Error code %d", fifo_list_result);
        return false;
    }
    CONSOLE_INFO("CanbusInit", "FIFOs configured - TEF at 0x%04X, RX FIFO1 at 0x%04X, TX FIFO2 at 0x%04X",
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
        CONSOLE_ERROR("CanbusInit", "Failed to configure filter list: Error code %d", filter_result);
        return false;
    }
    CONSOLE_INFO("CanbusInit", "Filters configured");
    
    // Start CAN bus in CAN-FD mode
    eERRORRESULT start_result = MCP251XFD_StartCANFD(&MCP251XFD_Ext1);
    if (start_result != ERR_OK) {
        CONSOLE_ERROR("CanbusInit", "Failed to start CAN bus: Error code %d", start_result);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    CONSOLE_INFO("CanbusInit", "CAN bus started - listening for messages...");



    eERRORRESULT tx_result = MCP251XFD_TransmitMessageToFIFO(
        &MCP251XFD_Ext1,
        &tx_message,
        MCP251XFD_FIFO2,
        true  // Flush immediately
    );
    if (tx_result == ERR_OK) {
        CONSOLE_INFO("CanbusInit", "Transmitted: ID=0x%03lX, DLC=%d, Data=[0x%02X 0x%02X] (BRS enabled)",
            tx_message.MessageID, tx_message.DLC, tx_data[0], tx_data[1]);
    } else {
        CONSOLE_ERROR("CanbusInit", "Failed to transmit message: Error code %d", tx_result);
    }



    
    return true;
}