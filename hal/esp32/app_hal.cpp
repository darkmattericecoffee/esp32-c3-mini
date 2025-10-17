/**
 * @file esp32_slave.ino
 * @author Gemini
 * @brief ESP32 Slave Display Controller for Teensy Master
 *
 * @details This code configures the ESP32 to act as a passive slave device.
 * It initializes an LVGL display and waits for commands from a Teensy master
 * over UART (Serial1). It does not initiate any communication itself, but simply
 * listens and updates the display based on the master's messages.
 *
 * Responsibilities:
 * - Initialize TFT display and LVGL.
 * - Display "Waiting for Teensy..." on startup.
 * - Listen for newline-terminated commands on Serial1.
 * - Process commands to:
 * - Display log/status messages ('L').
 * - Draw a G-code path ('T').
 * - Show the current toolhead position ('P').
 * - Reset the path ('R').
 * - Update connection status to "Connected" upon receiving the first message.
 * - If no message is received for a set timeout, update status to "Connection Lost!".
 * The connection is automatically re-established on the next message from the master.
 */

#include "Arduino.h"
#include <lvgl.h>
#include "app_hal.h"
#include "displays/pins.h"
#include "displays/generic.hpp"

// LVGL screen and buffer setup
static const uint32_t screenWidth = SCREEN_WIDTH;
static const uint32_t screenHeight = SCREEN_HEIGHT;
#define LV_BUFFER_SIZE (SCREEN_WIDTH * 20)
static uint8_t lvBuffer[LV_BUFFER_SIZE * 2];

// LVGL UI Objects
static lv_obj_t *status_label = nullptr;
static lv_obj_t *connection_label = nullptr;
static lv_obj_t *path_obj = nullptr; // To draw the G-code path
static lv_obj_t *toolhead_obj = nullptr; // To represent the toolhead

// Communication & State
static bool handshakeComplete = false;
static unsigned long lastPacketTime = 0;
const unsigned long CONNECTION_TIMEOUT = 5000; // Timeout for receiving any message from master

// Buffers
#define TEXT_BUFFER_SIZE 512 // Increased to handle path segments
static char textBuffer[TEXT_BUFFER_SIZE];
static int textBufferIndex = 0;

// Path storage
#define MAX_PATH_POINTS 10000 // Match Teensy's capacity
static lv_point_precise_t path_points[MAX_PATH_POINTS];
static int path_point_count = 0;

// Forward declarations for LVGL
static uint32_t my_tick(void);
void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data);
static void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data);
void screenBrightness(uint8_t value);

// Forward declarations for communication
void sendResponse(const char* message);
void updateConnectionStatus(const char* status, uint32_t color);
void processMessage(const String& message);
void handlePathMessage(const char* payload);
void handlePositionMessage(const char* payload);
void handleAngleMessage(const char* payload);
void handleLogMessage(const char* payload);
void handleResetMessage();

// HAL Setup - Initializes hardware and UI
void hal_setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting ESP32 Slave HAL setup...");

    // Using Serial1 for Teensy communication
    Serial1.setPins(20, 21);
    Serial1.begin(115200);
    Serial.println("Serial1 initialized on GPIO20 (RX) and GPIO21 (TX)");

    // TFT and LVGL Initialization
    tft.init();
    tft.initDMA();
    tft.startWrite();
    tft.fillScreen(TFT_BLACK);
    screenBrightness(150);

    lv_init();
    lv_tick_set_cb(my_tick);

    static lv_display_t *lvDisplay = lv_display_create(screenWidth, screenHeight);
    lv_display_set_color_format(lvDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(lvDisplay, my_disp_flush);
    lv_display_set_buffers(lvDisplay, lvBuffer, NULL, sizeof(lvBuffer), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Basic UI Setup
    lv_obj_t *bg = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bg, screenWidth, screenHeight);
    lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
    lv_obj_set_style_border_width(bg, 0, 0);

    connection_label = lv_label_create(lv_screen_active());
    lv_obj_align(connection_label, LV_ALIGN_TOP_MID, 0, 5);
    // As a slave, we just wait for the master to contact us.
    updateConnectionStatus("Waiting for Teensy...", 0xFFFF00); // Yellow

    status_label = lv_label_create(lv_screen_active());
    lv_label_set_text(status_label, "Awaiting data...");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_width(status_label, screenWidth - 10);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_label_set_long_mode(status_label, LV_LABEL_LONG_WRAP);

    // Path drawing object (a line)
    path_obj = lv_line_create(lv_screen_active());
    lv_obj_set_style_line_width(path_obj, 2, 0);
    lv_obj_set_style_line_color(path_obj, lv_color_hex(0x00FF00), 0); // Green path
    lv_line_set_points(path_obj, path_points, 0); // Initially empty

    // Toolhead object (a circle)
    toolhead_obj = lv_obj_create(lv_screen_active());
    lv_obj_set_size(toolhead_obj, 8, 8);
    lv_obj_set_style_radius(toolhead_obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(toolhead_obj, lv_color_hex(0xFF0000), 0); // Red toolhead
    lv_obj_set_style_border_width(toolhead_obj, 0, 0);
    lv_obj_center(toolhead_obj);
    lv_obj_add_flag(toolhead_obj, LV_OBJ_FLAG_HIDDEN); // Hide until position is known

    Serial.println("HAL Setup complete. Waiting for master...");
}

// HAL Loop - Main application loop
void hal_loop() {
    // 1. Process all available serial data from the master
    while (Serial1.available() > 0) {
        char inChar = Serial1.read();
        if (inChar == '\n') {
            if (textBufferIndex > 0) {
                textBuffer[textBufferIndex] = '\0';
                processMessage(String(textBuffer));
                textBufferIndex = 0;
            }
        } else if (inChar != '\r' && textBufferIndex < TEXT_BUFFER_SIZE - 1) {
            textBuffer[textBufferIndex++] = inChar;
        }
    }

    // 2. Check for connection timeout
    // If we haven't heard from the master, update status. Connection will be
    // re-established on the next message received.
    if (handshakeComplete && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
        updateConnectionStatus("Connection Lost!", 0xFF0000); // Red
        handshakeComplete = false;
        lv_obj_add_flag(toolhead_obj, LV_OBJ_FLAG_HIDDEN);
    }

    // 3. Handle LVGL tasks
    lv_timer_handler();
    delay(5);
}

// Process incoming text messages from the master
void processMessage(const String& message) {
    lastPacketTime = millis(); // We got a message, reset timeout timer

    if (message.length() < 1) return;
    char command = message.charAt(0);
    const char* payload = message.c_str() + 1;

    // The first valid message from Teensy completes the "handshake"
    if (!handshakeComplete) {
        handshakeComplete = true;
        updateConnectionStatus("Connected", 0x00FF00); // Green
    }

    switch (command) {
        case 'S': // System message (like PING) from master
            Serial.printf("System message: %s\n", payload);
            sendResponse("ACK"); // Acknowledge the master
            break;
        case 'L': // Log message
            handleLogMessage(payload);
            sendResponse("ACK");
            break;
        case 'D': // Debug message
            Serial.printf("Teensy Debug: %s\n", payload);
            sendResponse("ACK");
            break;
        case 'T': // Path data
            handlePathMessage(payload);
            sendResponse("ACK");
            break;
        case 'P': // Position update
            handlePositionMessage(payload);
            sendResponse("ACK");
            break;
        case 'A': // Angle update
            handleAngleMessage(payload);
            sendResponse("ACK");
            break;
        case 'R': // Reset path
            handleResetMessage();
            sendResponse("ACK");
            break;
        default:
            Serial.printf("Unknown command: %s\n", message.c_str());
            sendResponse("NACK"); // Negative acknowledgment for unknown commands
            break;
    }
}

void handleLogMessage(const char* payload) {
    Serial.printf("Teensy Log: %s\n", payload);
    if (status_label) {
        lv_label_set_text(status_label, payload);
    }
}

void handleResetMessage() {
    Serial.println("Resetting path data.");
    path_point_count = 0;
    if (path_obj) {
        lv_line_set_points(path_obj, path_points, 0);
        lv_obj_invalidate(path_obj); // Force redraw
    }
    if(toolhead_obj){
        lv_obj_add_flag(toolhead_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

void handlePathMessage(const char* payload) {
    Serial.printf("Received path segment: %s\n", payload);
    char* mutable_payload = strdup(payload);
    if (!mutable_payload) return;
    char* point_str = strtok(mutable_payload, ";");

    while (point_str != NULL && path_point_count < MAX_PATH_POINTS) {
        int x, y;
        if (sscanf(point_str, "%d,%d", &x, &y) == 2) {
            path_points[path_point_count].x = x;
            path_points[path_point_count].y = y;
            path_point_count++;
        }
        point_str = strtok(NULL, ";");
    }
    free(mutable_payload);

    if (path_obj) {
        lv_line_set_points(path_obj, path_points, path_point_count);
    }
    handleLogMessage("Path data updated.");
}

void handlePositionMessage(const char* payload) {
    int x, y;
    if (sscanf(payload, "%d,%d", &x, &y) == 2) {
        if (toolhead_obj) {
            lv_obj_clear_flag(toolhead_obj, LV_OBJ_FLAG_HIDDEN);
            // Center the 8x8 circle on the given coordinates, relative to screen center
            lv_obj_set_pos(toolhead_obj, x - 4 + (screenWidth / 2), y - 4 + (screenHeight / 2));
        }
    }
}

void handleAngleMessage(const char* payload) {
    int angle;
    if (sscanf(payload, "%d", &angle) == 1) {
        if (toolhead_obj) {
            // Placeholder for rotating an indicator if we add one
        }
    }
}

// Send simple newline-terminated responses back to the master
void sendResponse(const char* message) {
    if (!message) return;
    Serial1.println(message);
    Serial.printf("Sent ACK/NACK: %s\n", message);
}

void updateConnectionStatus(const char* status, uint32_t color) {
    if (connection_label != nullptr) {
        lv_label_set_text(connection_label, status);
        lv_obj_set_style_text_color(connection_label, lv_color_hex(color), 0);
    }
    Serial.println(status);
}

// --- LVGL helper functions ---

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
