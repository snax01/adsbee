The 1700-byte failure is almost certainly SPI DMA staging, not the CAN stack directly.

What’s going on
Caps 0x1800 = internal + DMA-capable RAM. On ESP32-S3, the SPI driver calls heap_caps_aligned_alloc() when a transfer buffer or length isn’t cache-line aligned (64 bytes).

1700 matches a typical settings SPI response:

sizeof(Settings) ≈ 1697
1 byte command + 2 byte CRC ≈ 1700
That lines up with settings sync and other kAddrSettingsData reads/writes.

You had ~6 KB free but largest block ~1588 B — classic fragmentation from many short-lived DMA allocations during SPI traffic and MCP251 init, then a ~1700 B (or 1728 B aligned) alloc fails.

Fixes applied
Pico SPI buffers — 8 KB heap DMA buffers replaced with static 64-byte-aligned spi_rx_buf_ / spi_tx_buf_ (no boot-time DMA heap, no per-transfer alloc for full 4096-byte polls).

CAN MCP251 SPI — single static DMA buffer for all MCP251 transfers (no stack → DMA staging malloc on every register access).

64-byte SPI wire length — GetSpiWireLenBytes() pads transaction length to 64 on both ESP slave and Pico master so the IDF driver doesn’t malloc staging buffers for odd sizes like 1700. Master and slave must agree on wire length (Pico esp32.cc + ESP pico.cpp + PartialRead).


Verifying 12288 bytes of data with CRC 0xf1f68679.
Writing 12288 Bytes to partition 1 at offset 0x1f3000.
Verifying flash with CRC 0xf1f68679.
OK
READY
CoProcessor: ESP32 >> [heap_caps_alloc_failed_hook] heap_caps_malloc was called but failed to allocate 12289 bytes with 0x1800 capabilities.
        free heap: 24040 bytes
        largest free block: 9204 bytes
        DRAM: 16264 bytes
        IRAM: 0 bytes

CoProcessor: ESP32 >> [WebSocketServer::Handler] [Network Console] Failed to calloc memory for buf.
ERROR Timed out after 5000 ms. Received 0 Bytes.
Erasing 12288 Bytes at offset 0x1f6000 in partition 1.
Erasing 3 sector(s) starting at 503/2025 (12288 Bytes at 0x10a0b000).
OK
READY
Verifying 12288 bytes of data with CRC 0xf1f68679.
Writing 12288 Bytes to partition 1 at offset 0x1f6000.
Verifying flash with CRC 0xf1f68679.
OK



/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "adsbee_server.hh"
#include "bsp.hh"
#include "comms.hh"
#include "cpu_utils.hh"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_debug_helpers.h"  // For esp_backtrace_print
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_unit_tests.hh"
#include "pico.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"
#include "task_priorities.hh"

#define HARDWARE_UNIT_TESTS
// #define PRINT_HEAP_USAGE

static const uint32_t kHeapUsagePrintIntervalMs = 100;
static const uint32_t kDeviceStatusUpdateIntervalMs = 1000;

BSP bsp = BSP();
ObjectDictionary object_dictionary;
Pico pico_ll = Pico({});
SPICoprocessor pico = SPICoprocessor({.interface = pico_ll});
ADSBeeServer adsbee_server = ADSBeeServer();
SettingsManager settings_manager = SettingsManager();
CommsManager comms_manager = CommsManager({});
CPUMonitor cpu_monitor = CPUMonitor({});

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char* function_name) {
    CONSOLE_ERROR("heap_caps_alloc_failed_hook",
                  "%s was called but failed to allocate %d bytes with 0x%lX capabilities.\r\n"
                  "\tfree heap: %d bytes\r\n"
                  "\tlargest free block: %d bytes\r\n"
                  "\tDRAM: %d bytes\r\n"
                  "\tIRAM: %d bytes\r\n",
                  function_name, requested_size, caps, heap_caps_get_free_size(MALLOC_CAP_8BIT),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                  heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT));
    printf("Stack trace at allocation failure:\n");
    esp_backtrace_print(20);  // Print up to 20 stack frames
}

void device_status_update_task(void* pvParameters) {
    while (1) {
        cpu_monitor.ReadCPUUsage(object_dictionary.device_status.core_0_usage_percent,
                                 object_dictionary.device_status.core_1_usage_percent);
        object_dictionary.device_status.temperature_deg_c = CPUMonitor::ReadTemperatureDegC();
        object_dictionary.device_status.heap_free_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        object_dictionary.device_status.heap_largest_free_block_bytes =
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

        vTaskDelay(pdMS_TO_TICKS(kDeviceStatusUpdateIntervalMs));  // Delay 1 second.
    }
}

// Main application
extern "C" void app_main(void) {
    heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

    ESP_LOGI("app_main", "Beginning ADSBee Server Application.");
    ESP_LOGI("app_main", "Default task priority: %d", uxTaskPriorityGet(NULL));

    CPUMonitor::Init();
    xTaskCreate(device_status_update_task, "DeviceStatusUpdate", kDeviceStatusUpdateTaskStackSizeBytes, NULL,
                kDeviceStatusUpdateTaskPriority, NULL);
    adsbee_server.Init();

#ifdef HARDWARE_UNIT_TESTS
    RunHardwareUnitTests();
#endif

#ifdef PRINT_HEAP_USAGE
    uint32_t last_heap_print_timestamp_ms = 0;
#endif
    while (1) {
        adsbee_server.Update();

        // Yield to the idle task to avoid a watchdog trigger. Note: Delay must be >= 10ms since 100Hz tick is typical.
        vTaskDelay(1);  // Delay 1 tick (10ms).

#ifdef PRINT_HEAP_USAGE
        if (get_time_since_boot_ms() - last_heap_print_timestamp_ms > kHeapUsagePrintIntervalMs) {
            CONSOLE_INFO("heap", "Free heap: %d, largest block: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT),
                         heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            last_heap_print_timestamp_ms = get_time_since_boot_ms();
        }
#endif
    }
}
