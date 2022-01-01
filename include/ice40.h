#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>

typedef esp_err_t (*ice40_get_done_t)(bool*);
typedef esp_err_t (*ice40_set_reset_t)(bool);

typedef struct ICE40 {
    //  Pins
    int spi_bus;
    int pin_cs;
    int pin_done;
    int pin_reset;
    int pin_int;
    
    // Configuration
    uint32_t spi_speed;
    uint32_t spi_max_transfer_size;
    
    // External pin handlers
    ice40_get_done_t get_done;
    ice40_set_reset_t set_reset;
    
    // Internal state
    spi_device_handle_t spi_device;
    bool cs_enabled;
} ICE40;

// Raw SPI transfer functions
esp_err_t ice40_send(ICE40* device, const uint8_t* data, int length);
esp_err_t ice40_receive(ICE40* device, uint8_t* data, int length);

// Devic state management
esp_err_t ice40_enable(ICE40* device);
esp_err_t ice40_disable(ICE40* device);
esp_err_t ice40_get_done(ICE40* device, bool* done);
esp_err_t ice40_load_bitstream(ICE40* device, const uint8_t* bitstream, uint32_t length);

// Driver initialization
esp_err_t ice40_init(ICE40* device);
