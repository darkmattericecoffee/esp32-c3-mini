#include "Arduino.h"
#include <lvgl.h>
#include "app_hal.h"
#include "displays/pins.h"
#include "displays/generic.hpp"

// --- Global variables and constants ---
static const uint32_t screenWidth = SCREEN_WIDTH;
static const uint32_t screenHeight = SCREEN_HEIGHT;

// Use a macro to ensure the size is a compile-time constant
#define LV_BUFFER_SIZE (SCREEN_WIDTH * 10) 
static uint8_t lvBuffer[LV_BUFFER_SIZE];
static uint8_t lvBuffer2[LV_BUFFER_SIZE];


// --- Helper function definitions (moved outside of hal_setup) ---

// Required by LVGL to get the current time
static uint32_t my_tick(void) {
    return millis();
}

// Required by LVGL to flush the display buffer to the screen
void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data)
{
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    lv_draw_sw_rgb565_swap(data, w * h);

    if (tft.getStartCount() == 0) {
        tft.endWrite();
    }

    tft.pushImageDMA(area->x1, area->y1, w, h, (uint16_t *)data);
    lv_display_flush_ready(display);
}

// Minimal touch driver required by LVGL
static void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data) {
    data->state = LV_INDEV_STATE_RELEASED; // Always report as not touched
}

// Minimal stub for screen brightness function
void screenBrightness(uint8_t value) {
    tft.setBrightness(value);
}


// --- Main Setup Function ---
// --- Main Setup Function ---
void hal_setup()
{
    Serial.begin(115200);
    Serial.println("Starting minimal setup...");

    // Initialize the display hardware
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

    // --- NEW LINE 1: INITIALIZE AND APPLY THE THEME ---
    // This will read LV_THEME_DEFAULT_DARK and set the background to black.
    lv_display_set_theme(lvDisplay, lv_theme_default_init(lvDisplay, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), 1, &lv_font_montserrat_14));


    // Create the "Hello World" label
    lv_obj_t *hello_label = lv_label_create(lv_screen_active());
    lv_label_set_text(hello_label, "Hello World!");
    lv_obj_set_style_text_color(hello_label, lv_color_hex(0xFFFFFF), 0); // White text
    
    // --- NEW LINE 2: SET A LARGER FONT ---
    lv_obj_set_style_text_font(hello_label, &lv_font_montserrat_24, 0);

    lv_obj_center(hello_label); // Center the label on the screen

    Serial.println("Minimal setup done. 'Hello World' should be visible.");
}


// --- Main Loop Function ---
void hal_loop()
{
    lv_timer_handler(); // Let LVGL handle its tasks
    delay(5);           // Yield to other tasks
}