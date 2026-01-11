/**
 * @file speaker.cpp
 * @brief PC Speaker emulation for M5PaperS3
 * 
 * Emulates the PC speaker using ESP32 LEDC PWM hardware.
 * 
 * The PC speaker is controlled by:
 * 1. PIT Channel 2 (ports 0x42/0x43) - generates the tone frequency
 * 2. Port 0x61 bits 0-1 - gate and enable controls
 * 
 * For sound output:
 * - Bit 0 of 0x61 (timer gate) must be 1 to enable PIT counting
 * - Bit 1 of 0x61 (speaker enable) must be 1 to connect PIT to speaker
 * - PIT channel 2 must be programmed with a valid frequency divisor
 * 
 * Most DOS games use PIT Mode 3 (square wave generator) for speaker output.
 * The LEDC hardware generates the square wave, so we just need to set
 * the correct frequency when conditions are met.
 */

#include "sdkconfig.h"
#include "config_defs.h"
#include "speaker.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <string.h>

static const char *TAG = "SPK";

// Hardware configuration for M5PaperS3 buzzer
#define SPEAKER_PIN         21
#define SPEAKER_LEDC_TIMER  LEDC_TIMER_0
#define SPEAKER_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define SPEAKER_LEDC_CH     LEDC_CHANNEL_0
#define SPEAKER_LEDC_RES    LEDC_TIMER_10_BIT  // 10-bit resolution

// PIT base frequency: 1.193182 MHz
#define PIT_FREQUENCY       1193182UL

// Frequency limits for the speaker
#define MIN_FREQUENCY_HZ    20
#define MAX_FREQUENCY_HZ    20000

/**
 * Internal speaker state
 * 
 * We track both the "requested" state (what the emulated software wants)
 * and the "hardware" state (what we've actually configured in LEDC).
 */
static struct {
    // Initialization flag
    bool initialized;
    
    // Port 0x61 state
    bool gate_enabled;      // Bit 0: PIT channel 2 gate
    bool speaker_enabled;   // Bit 1: Speaker output enable
    
    // PIT state
    uint8_t pit_mode;       // Current PIT mode (0-5)
    uint16_t pit_reload;    // PIT channel 2 reload value (divisor)
    bool pit_counting;      // True if PIT has been triggered and is counting
    
    // Calculated values
    uint32_t target_freq;   // Desired frequency in Hz
    
    // Hardware state
    bool hw_active;         // True if LEDC is currently outputting
    uint32_t hw_freq;       // Frequency currently set in LEDC hardware
    
    // Volume (maps to duty cycle)
    uint8_t volume;         // 0-255, where 255 = 50% duty (max for square wave)
} spk = {
    .initialized = false,
    .gate_enabled = false,
    .speaker_enabled = false,
    .pit_mode = PIT_MODE_3,
    .pit_reload = 0,
    .pit_counting = false,
    .target_freq = 0,
    .hw_active = false,
    .hw_freq = 0,
    .volume = 128,
};

/**
 * Calculate the duty cycle value for LEDC based on volume.
 * For a square wave, max duty is 50% (half of the resolution range).
 * Volume scales this linearly.
 */
static uint32_t calc_duty(uint8_t volume)
{
    // Max duty for square wave is 50% of full range
    // With 10-bit resolution, full range is 1024 (0-1023)
    // So 50% = 512
    const uint32_t max_duty = (1U << SPEAKER_LEDC_RES) / 2U;  // 512
    return (max_duty * (uint32_t)volume) / 255U;
}

/**
 * Calculate frequency from PIT reload value.
 * Handles the special case where reload=0 means 65536.
 */
static uint32_t calc_frequency(uint16_t reload)
{
    uint32_t divisor = reload ? reload : 0x10000;
    return PIT_FREQUENCY / divisor;
}

/**
 * Update LEDC hardware to match the desired speaker state.
 * This is the central function that actually controls the hardware.
 */
static void speaker_update_hw(void)
{
    if (!spk.initialized) {
        return;
    }
    
    // Determine if speaker should be active
    // Requirements:
    // 1. Gate enabled (bit 0 of 0x61)
    // 2. Speaker enabled (bit 1 of 0x61)  
    // 3. Valid frequency (non-zero reload value)
    // 4. For modes that need triggering (mode 1, 5), must be triggered
    bool should_play = spk.gate_enabled && 
                       spk.speaker_enabled && 
                       spk.target_freq >= MIN_FREQUENCY_HZ &&
                       spk.target_freq <= MAX_FREQUENCY_HZ;
    
    // For mode 3 (square wave), speaker is active whenever gate and enable are set
    // For mode 0 (interrupt on terminal count), output goes low initially then high
    // For mode 1 (one-shot), requires gate rising edge to trigger
    // For mode 2 (rate generator), output goes high immediately when gate is set
    // Mode 3 is by far the most common for PC speaker
    
    if (!should_play) {
        // Turn off speaker
        if (spk.hw_active) {
            ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH, 0);
            ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH);
            spk.hw_active = false;
            ESP_LOGI(TAG, "Speaker STOP");
        }
        return;
    }
    
    // Speaker should be playing
    // Check if we need to update frequency
    if (spk.hw_freq != spk.target_freq) {
        // Configure timer with new frequency
        // Using ledc_set_freq is faster but may have limitations
        // Full timer reconfig is more reliable
        esp_err_t err = ledc_set_freq(SPEAKER_LEDC_MODE, SPEAKER_LEDC_TIMER, spk.target_freq);
        if (err != ESP_OK) {
            // ledc_set_freq failed, try full reconfiguration
            ledc_timer_config_t timer_cfg = {
                .speed_mode = SPEAKER_LEDC_MODE,
                .duty_resolution = SPEAKER_LEDC_RES,
                .timer_num = SPEAKER_LEDC_TIMER,
                .freq_hz = spk.target_freq,
                .clk_cfg = LEDC_AUTO_CLK,
                .deconfigure = false,
            };
            err = ledc_timer_config(&timer_cfg);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set frequency %lu Hz: %s", 
                         (unsigned long)spk.target_freq, esp_err_to_name(err));
                return;
            }
        }
        
        uint32_t old_freq = spk.hw_freq;
        spk.hw_freq = spk.target_freq;
        
        if (!spk.hw_active) {
            ESP_LOGI(TAG, "Speaker START: %lu Hz (div=%u)", 
                     (unsigned long)spk.hw_freq, (unsigned)spk.pit_reload);
        } else {
            ESP_LOGI(TAG, "Speaker FREQ: %lu -> %lu Hz (div=%u)", 
                     (unsigned long)old_freq, (unsigned long)spk.hw_freq, 
                     (unsigned)spk.pit_reload);
        }
    }
    
    // Set duty cycle and ensure output is on
    uint32_t duty = calc_duty(spk.volume);
    if (!spk.hw_active || duty != ledc_get_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH)) {
        ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH, duty);
        ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH);
    }
    
    if (!spk.hw_active) {
        if (spk.hw_freq == spk.target_freq) {
            // Frequency wasn't changed, just enabled
            ESP_LOGI(TAG, "Speaker START: %lu Hz (div=%u)", 
                     (unsigned long)spk.hw_freq, (unsigned)spk.pit_reload);
        }
        spk.hw_active = true;
    }
}

/**
 * Initialize the speaker subsystem.
 */
void speaker_init(void)
{
    ESP_LOGI(TAG, "Initializing PC speaker emulation on Pin %d", SPEAKER_PIN);
    
    // Configure LEDC timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode = SPEAKER_LEDC_MODE,
        .duty_resolution = SPEAKER_LEDC_RES,
        .timer_num = SPEAKER_LEDC_TIMER,
        .freq_hz = 1000,  // Initial frequency (will be changed)
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    
    // Configure LEDC channel
    ledc_channel_config_t channel_cfg = {
        .gpio_num = SPEAKER_PIN,
        .speed_mode = SPEAKER_LEDC_MODE,
        .channel = SPEAKER_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SPEAKER_LEDC_TIMER,
        .duty = 0,  // Start with speaker off
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    
    // Initialize state
    spk.initialized = true;
    spk.gate_enabled = false;
    spk.speaker_enabled = false;
    spk.pit_mode = PIT_MODE_3;
    spk.pit_reload = 0;
    spk.pit_counting = false;
    spk.target_freq = 0;
    spk.hw_active = false;
    spk.hw_freq = 1000;
    spk.volume = 128;
    
    ESP_LOGI(TAG, "Speaker initialized");
}

/**
 * Called when PIT control word is written (port 0x43) for channel 2.
 * This typically happens when software wants to start a new sound.
 * 
 * Control word format:
 * Bits 7-6: Channel select (10 = channel 2)
 * Bits 5-4: Access mode (11 = lobyte/hibyte)
 * Bits 3-1: Mode (011 = mode 3 square wave)
 * Bit 0: BCD mode (usually 0)
 */
void speaker_set_pit_control(uint8_t mode)
{
    if (!spk.initialized) return;
    
    // Mode 6 and 7 are aliases for mode 2 and 3
    if (mode >= 6) {
        mode -= 4;
    }
    
    bool mode_changed = (spk.pit_mode != mode);
    spk.pit_mode = mode;
    
    // Reset counting state - new control word resets the PIT channel
    spk.pit_counting = false;
    
    if (mode_changed) {
        static const char* mode_names[] = {
            "interrupt on terminal count",
            "hardware retriggerable one-shot",
            "rate generator", 
            "square wave generator",
            "software triggered strobe",
            "hardware triggered strobe"
        };
        const char* name = (mode <= 5) ? mode_names[mode] : "unknown";
        ESP_LOGI(TAG, "PIT mode set: %d (%s)", mode, name);
    }
    
    // For most modes, the output goes to a known state when control word is written
    // Mode 0: output low
    // Mode 1: output high (waiting for gate trigger)
    // Mode 2,3: output high
    // Mode 4: output high
    // Mode 5: output high (waiting for gate trigger)
    
    // Don't update hardware yet - wait for reload value to be set
}

/**
 * Called when a complete reload value has been written to PIT channel 2.
 * This sets the frequency divisor and may start the tone.
 */
void speaker_set_counter(uint16_t reload_value, uint8_t mode)
{
    if (!spk.initialized) return;
    
    spk.pit_reload = reload_value;
    spk.pit_mode = mode;
    spk.target_freq = calc_frequency(reload_value);
    
    ESP_LOGI(TAG, "Set counter: reload=%u freq=%lu Hz mode=%d", 
             (unsigned)reload_value, (unsigned long)spk.target_freq, mode);
    
    // In mode 3 (square wave), counting starts immediately when reload is set
    // and gate is enabled
    if (mode == PIT_MODE_3 || mode == PIT_MODE_2) {
        if (spk.gate_enabled) {
            spk.pit_counting = true;
        }
    }
    // In mode 1, counting waits for gate rising edge
    // In mode 0, counting starts when reload is set (if gate high)
    else if (mode == PIT_MODE_0) {
        if (spk.gate_enabled) {
            spk.pit_counting = true;
        }
    }
    
    speaker_update_hw();
}

/**
 * Legacy API - calls speaker_set_counter with current mode.
 */
void speaker_set_frequency(uint16_t reload_value)
{
    speaker_set_counter(reload_value, spk.pit_mode);
}

/**
 * Called when port 0x61 is written.
 * Handles speaker enable and PIT gate control.
 * 
 * Bit 0: Timer 2 gate to speaker (enables PIT counting)
 * Bit 1: Speaker data (connects PIT output to speaker)
 */
void speaker_update_port61(uint8_t port61_value)
{
    if (!spk.initialized) return;
    
    bool new_gate = (port61_value & 0x01) != 0;
    bool new_speaker = (port61_value & 0x02) != 0;
    
    bool gate_changed = (new_gate != spk.gate_enabled);
    bool speaker_changed = (new_speaker != spk.speaker_enabled);
    
    if (gate_changed || speaker_changed) {
        ESP_LOGI(TAG, "Port 0x61: gate=%d->%d enable=%d->%d", 
                 spk.gate_enabled, new_gate,
                 spk.speaker_enabled, new_speaker);
    }
    
    // Check for gate rising edge (0->1 transition)
    bool gate_rising = (!spk.gate_enabled && new_gate);
    
    spk.gate_enabled = new_gate;
    spk.speaker_enabled = new_speaker;
    
    // Handle gate rising edge for modes that need it
    if (gate_rising) {
        // Mode 1 and 5 trigger on gate rising edge
        if (spk.pit_mode == PIT_MODE_1 || spk.pit_mode == PIT_MODE_5) {
            spk.pit_counting = true;
        }
        // Mode 2 and 3 restart counting on gate rising edge
        else if (spk.pit_mode == PIT_MODE_2 || spk.pit_mode == PIT_MODE_3) {
            spk.pit_counting = true;
        }
    }
    
    // If gate goes low, stop counting for modes 2 and 3
    if (!new_gate && (spk.pit_mode == PIT_MODE_2 || spk.pit_mode == PIT_MODE_3)) {
        spk.pit_counting = false;
    }
    
    speaker_update_hw();
}

/**
 * Force speaker on (for testing/debugging).
 */
void speaker_on(void)
{
    if (!spk.initialized) return;
    
    spk.gate_enabled = true;
    spk.speaker_enabled = true;
    speaker_update_hw();
}

/**
 * Force speaker off.
 */
void speaker_off(void)
{
    if (!spk.initialized) return;
    
#if M5PAPER_SPEAKER_DEBUG
    ESP_LOGI(TAG, "Speaker OFF (forced)");
#endif
    
    ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH, 0);
    ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH);
    spk.hw_active = false;
}

/**
 * Check if speaker is currently producing sound.
 */
bool speaker_is_active(void)
{
    return spk.initialized && spk.hw_active;
}

/**
 * Set speaker volume (0-255).
 */
void speaker_set_volume(uint8_t volume)
{
    spk.volume = volume;
    if (spk.hw_active) {
        uint32_t duty = calc_duty(volume);
        ledc_set_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH, duty);
        ledc_update_duty(SPEAKER_LEDC_MODE, SPEAKER_LEDC_CH);
    }
}

/**
 * Get current speaker frequency.
 */
uint32_t speaker_get_frequency(void)
{
    return spk.hw_active ? spk.hw_freq : 0;
}

/**
 * Periodic tick (placeholder for future use).
 */
void speaker_tick(void)
{
    // Could be used for:
    // - Timeout-based silence detection
    // - Audio envelope effects
    // - Sample-based audio generation
}

/**
 * Shutdown speaker subsystem.
 */
void speaker_deinit(void)
{
    if (spk.initialized) {
        speaker_off();
        spk.initialized = false;
    }
}
