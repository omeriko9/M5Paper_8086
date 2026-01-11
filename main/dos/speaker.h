/**
 * @file speaker.h
 * @brief PC Speaker emulation for M5PaperS3
 * 
 * Emulates the old PC speaker/buzzer using the M5PaperS3's built-in speaker.
 * The PC speaker was controlled via port 0x61 (keyboard controller) and
 * port 0x42/0x43 (PIT channel 2) for tone generation.
 * 
 * The speaker output is controlled by:
 * - Port 0x61 bit 0: PIT channel 2 gate (enables timer counting)
 * - Port 0x61 bit 1: Speaker enable (connects PIT output to speaker)
 * - Port 0x43: PIT control word (sets mode for channel 2)
 * - Port 0x42: PIT channel 2 data (sets reload/frequency value)
 * 
 * For sound to play: both gate and enable must be set, and a valid
 * frequency must be programmed.
 */

#ifndef SPEAKER_H
#define SPEAKER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PIT operating modes (from port 0x43 control word bits 1-3)
 */
typedef enum {
    PIT_MODE_0 = 0,  // Interrupt on terminal count
    PIT_MODE_1 = 1,  // Hardware retriggerable one-shot  
    PIT_MODE_2 = 2,  // Rate generator
    PIT_MODE_3 = 3,  // Square wave generator (most common for speaker)
    PIT_MODE_4 = 4,  // Software triggered strobe
    PIT_MODE_5 = 5,  // Hardware triggered strobe
} pit_mode_t;

/**
 * Initialize speaker subsystem.
 * Configures LEDC PWM on the speaker pin for tone generation.
 */
void speaker_init(void);

/**
 * Notify speaker that PIT control word was written.
 * Called when a write to port 0x43 affects channel 2.
 * This resets the speaker state for a new sound sequence.
 * 
 * @param mode The PIT operating mode (0-5)
 */
void speaker_set_pit_control(uint8_t mode);

/**
 * Set the speaker frequency based on PIT channel 2 reload value.
 * Called when a complete reload value has been written to port 0x42.
 * The actual frequency is: 1193182 / reload_value Hz
 * 
 * @param reload_value PIT channel 2 reload value (0 treated as 65536)
 * @param mode Current PIT mode
 */
void speaker_set_counter(uint16_t reload_value, uint8_t mode);

/**
 * Update speaker state based on port 0x61 value.
 * Bit 0: PIT channel 2 gate (timer enable / trigger)
 * Bit 1: Speaker enable (output enable)
 * 
 * @param port61_value The value written to port 0x61
 */
void speaker_update_port61(uint8_t port61_value);

/**
 * Legacy API for setting frequency (calls speaker_set_counter internally).
 * @param reload_value PIT channel 2 reload value
 */
void speaker_set_frequency(uint16_t reload_value);

/**
 * Turn speaker on with current frequency (forces enable bits).
 */
void speaker_on(void);

/**
 * Turn speaker off (clears enable bits).
 */
void speaker_off(void);

/**
 * Check if speaker is currently producing sound.
 * @return true if speaker PWM is active
 */
bool speaker_is_active(void);

/**
 * Set speaker volume (0-255).
 * Maps to PWM duty cycle (255 = 50% duty for square wave).
 * @param volume Volume level
 */
void speaker_set_volume(uint8_t volume);

/**
 * Get current speaker frequency in Hz.
 * @return Current frequency in Hz, or 0 if not playing
 */
uint32_t speaker_get_frequency(void);

/**
 * Periodic tick function (placeholder for future use).
 */
void speaker_tick(void);

/**
 * Shutdown speaker subsystem and release resources.
 */
void speaker_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SPEAKER_H
