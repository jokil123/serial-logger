#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Include for Queues
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_vfs_fat.h" // Include for FAT FS
#include "sdmmc_cmd.h"   // Include for SD card commands

// UART Config
#define UART_PORT_NUM UART_NUM_2
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16

#define UART_BAUD_RATE 115200
#define TASK_STACK_SIZE 2048
#define RX_BUF_SIZE 1024

// SD Card Config
#define MOUNT_POINT "/sdcard"
#define LOG_FILENAME MOUNT_POINT "/uart_log.txt"
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5

#define SD_WRITE_TASK_STACK_SIZE 4096 // SD card + FATFS needs more stack
// #define SD_WRITE_BUF_SIZE 512         // How many bytes to buffer before writing
#define SD_WRITE_BUF_SIZE 2048  // How many bytes to buffer before writing
#define SHARED_QUEUE_SIZE 1024  // Max bytes to hold in queue between tasks
#define SD_WRITE_TIMEOUT_MS 500 // Write to SD if no new data for this long

static const char *TAG_UART = "UART";
static const char *TAG_SD = "SD";
static QueueHandle_t uart_to_sd_queue; // Queue to pass data between tasks

// Init uart
void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG_UART, "Configuring UART%d...", UART_PORT_NUM);
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    ESP_LOGI(TAG_UART, "Setting UART%d pins: TX=%d, RX%d", UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN);

    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG_UART, "UART%d initialized sucessfully.", UART_PORT_NUM);
}

static void uart_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG_UART, "UART RX task started on core %d", xPortGetCoreID());
    uint8_t received_char;

    while (1)
    {
        int len = uart_read_bytes(UART_PORT_NUM, &received_char, 1, portMAX_DELAY);

        if (len > 0)
        {
            BaseType_t result = xQueueSend(uart_to_sd_queue, &received_char, pdMS_TO_TICKS(100));

            if (result != pdPASS)
            {
                ESP_LOGW(TAG_UART, "Failed to queue character! SD card writing might be too slow");
            }
        }
        else if (len < 0)
        {
            ESP_LOGE(TAG_UART, "uart_read_bytes error: %d", len);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    vTaskDelete(NULL);
}

esp_err_t init_sd_card(void)
{
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG_SD, "Initializing SD card");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = 5000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SD, "Failed to initialize SPI bus (%s)", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG_SD, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG_SD, "Failed to mount filesystem.");
        }
        else
        {
            ESP_LOGE(TAG_SD, "Failed to initialize the sd card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    ESP_LOGI(TAG_SD, "Filesystem mointed");
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

static void sd_write_task(void *pvParameters)
{
    ESP_LOGI(TAG_UART, "sd_write_task started on core %d", xPortGetCoreID());
    uint8_t write_buffer[SD_WRITE_BUF_SIZE];
    TickType_t last_sd_write_time = xTaskGetTickCount();
    size_t buffer_byte_cursor = 0;
    FILE *f = NULL;

    ESP_LOGI(TAG_SD, "Opening file %s for appending...", LOG_FILENAME);
    f = fopen(LOG_FILENAME, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG_SD, "Failed to open file for writing");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_SD, "File opened sucessfully.");

    while (1)
    {
        bool writeTimeoutExceeded = xTaskGetTickCount() - last_sd_write_time > pdMS_TO_TICKS(SD_WRITE_TIMEOUT_MS);

        if (buffer_byte_cursor >= SD_WRITE_BUF_SIZE || writeTimeoutExceeded)
        {
            ESP_LOGI(TAG_SD, "Write buffer full (%d of %d) or timeout (%ld), writing to SD card...", buffer_byte_cursor, SD_WRITE_BUF_SIZE, pdTICKS_TO_MS(xTaskGetTickCount() - last_sd_write_time));
            size_t written = fwrite(write_buffer, 1, buffer_byte_cursor, f);
            if (written != buffer_byte_cursor)
            {
                ESP_LOGE(TAG_SD, "File write failed! Only wrote %d of %d bytes", written, buffer_byte_cursor);
            }
            else
            {
                if (fsync(fileno(f)) != 0)
                {

                    ESP_LOGE(TAG_SD, "fsync failed");
                }
            }
            last_sd_write_time = xTaskGetTickCount();
            buffer_byte_cursor = 0;
        }

        TickType_t wait_time = pdMS_TO_TICKS(SD_WRITE_TIMEOUT_MS);

        if (buffer_byte_cursor == 0)
        {
            wait_time = portMAX_DELAY;
        }

        // ESP_LOGI(TAG_SD, "Waiting %ld ms", pdTICKS_TO_MS(wait_time));
        BaseType_t received = xQueueReceive(uart_to_sd_queue, &write_buffer[buffer_byte_cursor], wait_time);
        // ESP_LOGI(TAG_SD, "Received character through queue or timeout (%d)", received);

        if (received != pdPASS)
        {
            // ESP_LOGI(TAG_SD, "No character received");
            continue;
        }

        buffer_byte_cursor++;
    }

    ESP_LOGI(TAG_SD, "SD Write task finishing...");
    if (f != NULL)
    {
        fflush(f);
        fclose(f);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG_UART, "Starting logger");

    uart_to_sd_queue = xQueueCreate(SHARED_QUEUE_SIZE, sizeof(uint8_t));
    if (uart_to_sd_queue == NULL)
    {
        ESP_LOGE("APP_MAIN", "Failed to create shared queue! Terminating.");
        return;
    }

    uart_init();
    if (init_sd_card() != ESP_OK)
    {
        ESP_LOGE("APP_MAIN", "SD card init failed. Terminating task.");
        vTaskDelete(NULL);
        return;
    }

    xTaskCreate(uart_rx_task, "uart_rx_task", TASK_STACK_SIZE, NULL, 6, NULL);
    xTaskCreate(sd_write_task, "sd_write_task", SD_WRITE_TASK_STACK_SIZE, NULL, 5, NULL);

    ESP_LOGI(TAG_UART, "app_main finished setup.");
}