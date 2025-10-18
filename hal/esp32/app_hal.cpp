/**
 * @file app_hal.cpp
 * @author Modified for Simple UART Mirroring
 * @brief ESP32 Display Controller - Simple UART Text Mirror
 *
 * @details This code displays text received over UART on an LVGL display.
 * Simplified version with no handshaking - just mirrors UART input to screen.
 */

#include "Arduino.h"
#include <lvgl.h>
#include <cstring>
#include "app_hal.h"
#include "displays/pins.h"
#include "displays/generic.hpp"

// --- LVGL & Display Setup ---
static const uint32_t screenWidth = SCREEN_WIDTH;
static const uint32_t screenHeight = SCREEN_HEIGHT;
#define LV_BUFFER_SIZE (SCREEN_WIDTH * 10)
static uint8_t lvBuffer[LV_BUFFER_SIZE];
static uint8_t lvBuffer2[LV_BUFFER_SIZE];

// LVGL UI Objects
static lv_obj_t *main_text_label = nullptr;

// Buffer for incoming text
static String receivedText = "";

// --- Forward Declarations ---
static uint32_t my_tick(void);
void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data);
static void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data);
void screenBrightness(uint8_t value);
void processIncomingUart();

// --- Main HAL Functions ---

void hal_setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Starting ESP32 Simple UART Mirror...");

    // Using Serial1 for incoming text
    Serial1.setPins(20, 21);
    Serial1.setRxBufferSize(1024);
    Serial1.begin(115200);
    Serial.println("Serial1 initialized on GPIO20 (RX) and GPIO21 (TX)");

    // TFT and LVGL Initialization
    tft.init();
    tft.initDMA();
    tft.startWrite();
    tft.fillScreen(TFT_BLACK);
    screenBrightness(150);

    // Initialize LVGL
    lv_init();
    lv_tick_set_cb(my_tick);

    // Create LVGL display
    static lv_display_t *lvDisplay = lv_display_create(screenWidth, screenHeight);
    lv_display_set_color_format(lvDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(lvDisplay, my_disp_flush);
    lv_display_set_buffers(lvDisplay, lvBuffer, lvBuffer2, LV_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Create LVGL input device (touch)
    static lv_indev_t *lvInput = lv_indev_create();
    lv_indev_set_type(lvInput, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvInput, my_touchpad_read);

    // Initialize and apply the theme
    lv_display_set_theme(lvDisplay, lv_theme_default_init(lvDisplay, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), 1, &lv_font_montserrat_14));

    // --- UI Setup ---
    lv_obj_t *bg = lv_screen_active();

    // Main Text Label (centered, wraps text)
    main_text_label = lv_label_create(bg);
    lv_obj_set_width(main_text_label, screenWidth - 20);
    lv_label_set_long_mode(main_text_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(main_text_label, "Ready...\nWaiting for text");
    lv_obj_set_style_text_align(main_text_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(main_text_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(main_text_label, &lv_font_montserrat_24, 0);

    Serial.println("HAL Setup complete. Ready to mirror UART text.");
}

void hal_loop() {
    processIncomingUart();
    lv_timer_handler();
}

// --- Core Communication Logic ---

void processIncomingUart() {
    static unsigned long lastUpdate = 0;
    bool hasNewData = false;

    // Read all available characters
    while (Serial1.available()) {
        char c = Serial1.read();
        
        // Echo to USB Serial immediately (for debugging)
        Serial.write(c);
        
        // Handle special characters
        if (c == '\n') {
            // Newline - just add it to the string
            receivedText += '\n';
        } else if (c == '\r') {
            // Carriage return - ignore or handle as needed
            continue;
        } else if (c >= 32 && c <= 126) {
            // Printable ASCII character
            receivedText += c;
        }
        // Ignore other control characters
        
        hasNewData = true;
        lastUpdate = millis();
    }

    // Update display if we have new data and haven't updated in 50ms
    // This batches rapid updates to avoid flickering
    if (hasNewData && (millis() - lastUpdate) > 50) {
        // Limit text length to prevent memory issues
        if (receivedText.length() > 500) {
            receivedText = receivedText.substring(receivedText.length() - 500);
        }
        
        lv_label_set_text(main_text_label, receivedText.c_str());
        Serial.print("\n[Display updated with ");
        Serial.print(receivedText.length());
        Serial.println(" chars]");
        hasNewData = false;
    }
}

// --- LVGL Helper Functions ---

static uint32_t my_tick(void) {
    return millis();
}

void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    lv_draw_sw_rgb565_swap(data, w * h);
    if (tft.getStartCount() == 0) {
        tft.endWrite();
    }
    tft.pushImageDMA(area->x1, area->y1, w, h, (uint16_t *)data);
    lv_display_flush_ready(display);
}

static void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data) {
    data->state = LV_INDEV_STATE_RELEASED;
}

void screenBrightness(uint8_t value) {
    tft.setBrightness(value);
}