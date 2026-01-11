/**
 * @file sdcard.c
 * @brief SD Card driver for M5Paper
 *
 * Uses SPI interface to communicate with SD card.
 * M5Paper uses specific pins for SD card.
 */

#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "sdcard.h"

static const char *TAG = "SDCARD";

static gpio_num_t M5PAPER_TOUCH_INT_PIN = GPIO_NUM_36;
static gpio_num_t M5PAPER_MAIN_PWR_PIN = GPIO_NUM_2;
static gpio_num_t M5PAPER_SD_CS_PIN = GPIO_NUM_4;
static gpio_num_t M5PAPER_SD_MOSI_PIN = GPIO_NUM_12;
static gpio_num_t M5PAPER_SD_MISO_PIN = GPIO_NUM_13;
static gpio_num_t M5PAPER_SD_CLK_PIN = GPIO_NUM_14;

static gpio_num_t M5PAPERS3_SD_MOSI_PIN = GPIO_NUM_38;
static gpio_num_t M5PAPERS3_SD_MISO_PIN = GPIO_NUM_40;
static gpio_num_t M5PAPERS3_SD_CLK_PIN = GPIO_NUM_39;
static gpio_num_t M5PAPERS3_SD_CS_PIN = GPIO_NUM_47;

#define CONFIG_SD_MOUNT_POINT "/sdcard"

static bool mountSDCard(void);

// M5Paper SD card SPI pins
#define SD_PIN_MOSI GPIO_NUM_12
#define SD_PIN_MISO GPIO_NUM_13
#define SD_PIN_CLK GPIO_NUM_14
#define SD_PIN_CS GPIO_NUM_4

// SD card mount point
#define MOUNT_POINT "/sdcard"

// SPI host to use
#define SD_SPI_HOST SPI3_HOST

// Card handle
static sdmmc_card_t *card = NULL;
static bool mounted = false;
static bool spi_bus_initialized = false;

/**
 * Initialize SD card
 */
esp_err_t sdcard_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card...");

    if (!mountSDCard())
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

#define CONFIG_EBOOK_ENABLE_SD_CARD 1
static bool mountSDCard(void)
{
#ifdef CONFIG_EBOOK_ENABLE_SD_CARD
    if (mounted)
    {
        ESP_LOGI(TAG, "SD card already mounted");
        return true;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {};
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    int mosi, miso, clk, cs;
    spi_host_device_t host_id;

#ifdef CONFIG_EBOOK_DEVICE_M5PAPERS3
    mosi = M5PAPERS3_SD_MOSI_PIN;
    miso = M5PAPERS3_SD_MISO_PIN;
    clk = M5PAPERS3_SD_CLK_PIN;
    cs = M5PAPERS3_SD_CS_PIN;
    host_id = SPI2_HOST;
    host.max_freq_khz = 20000;
#else
    mosi = M5PAPER_SD_MOSI_PIN;
    miso = M5PAPER_SD_MISO_PIN;
    clk = M5PAPER_SD_CLK_PIN;
    cs = M5PAPER_SD_CS_PIN;
    host_id = SPI3_HOST; // Share VSPI with EPD to avoid pin mux conflicts.
    host.max_freq_khz = 20000;
#endif

    ESP_LOGI(TAG, "Mounting SD card on pins CS=%d, CLK=%d, MOSI=%d, MISO=%d, Host=%d",
             cs, clk, mosi, miso, host_id);

    host.slot = host_id;

    bus_cfg.mosi_io_num = mosi;
    bus_cfg.miso_io_num = miso;
    bus_cfg.sclk_io_num = clk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4096;
    bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;
    bus_cfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    bus_cfg.intr_flags = 0;

    size_t max_transfer_sz = 0;
    esp_err_t ret = spi_bus_get_max_transaction_len(host_id, &max_transfer_sz);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SPI bus already initialized");
    }
    else
    {
        if (ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "SPI bus query failed: %s. Attempting to initialize anyway...", esp_err_to_name(ret));
        }
        
        ret = spi_bus_initialize(host_id, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return false;
        }
        spi_bus_initialized = true;
        ESP_LOGI(TAG, "SPI bus initialized");
    }

    slot_config.gpio_cs = (gpio_num_t)cs;
    slot_config.host_id = host_id;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false};

    ESP_LOGI(TAG, "Attempting SD card mount...");
    ret = esp_vfs_fat_sdspi_mount(
        CONFIG_SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        if (spi_bus_initialized)
        {
            spi_bus_free(host_id);
            spi_bus_initialized = false;
        }
        return false;
    }

    mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s", CONFIG_SD_MOUNT_POINT);

    if (card)
    {
        sdmmc_card_print_info(stdout, card);
    }

    return true;
#else
    return false;
#endif
}
/**
 * Deinitialize SD card
 */
void sdcard_deinit(void)
{
    if (mounted)
    {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        ESP_LOGI(TAG, "Card unmounted");

        if (spi_bus_initialized)
        {
            spi_bus_free(SD_SPI_HOST);
            spi_bus_initialized = false;
        }
        mounted = false;
        card = NULL;
    }
}

/**
 * Check if mounted
 */
bool sdcard_is_mounted(void)
{
    return mounted;
}

/**
 * Get total size
 */
uint64_t sdcard_get_total_size(void)
{
    if (!mounted || !card)
        return 0;

    return (uint64_t)card->csd.capacity * card->csd.sector_size;
}

/**
 * Get free size
 */
uint64_t sdcard_get_free_size(void)
{
    if (!mounted)
        return 0;

    FATFS *fs;
    DWORD fre_clust;

    if (f_getfree(MOUNT_POINT, &fre_clust, &fs) != FR_OK)
    {
        return 0;
    }

    uint64_t free_bytes = (uint64_t)fre_clust * fs->csize * 512;
    return free_bytes;
}
