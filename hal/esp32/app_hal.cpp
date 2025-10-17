/**
 * @file app_hal.cpp
 * @author Gemini
 * @brief ESP32 Slave Display Controller for Teensy Master using LVGL.
 *
 * @details This code configures the ESP32 to act as a passive slave device.
 * It initializes an LVGL display and waits for binary packet commands from a
 * Teensy master over UART. It parses these packets to mirror the master's
 * text display content.
 */

#include "Arduino.h"
#include <lvgl.h>
#include <cstring> // For strncpy, strnlen
#include "app_hal.h"
#include "displays/pins.h"
#include "displays/generic.hpp"

// --- LVGL & Display Setup ---
static const uint32_t screenWidth = SCREEN_WIDTH;
static const uint32_t screenHeight = SCREEN_HEIGHT;
// Use a macro to ensure the size is a compile-time constant, matching the user's example
#define LV_BUFFER_SIZE (SCREEN_WIDTH * 10)
static uint8_t lvBuffer[LV_BUFFER_SIZE];
static uint8_t lvBuffer2[LV_BUFFER_SIZE];


// LVGL UI Objects
static lv_obj_t *connection_label = nullptr;
static lv_obj_t *main_text_label = nullptr;

// --- Communication Protocol Definition (must match Teensy's coms.h) ---
constexpr uint8_t START_BYTE = 0xA5;

enum class PacketType : uint8_t {
    // Teensy -> ESP32 Commands
    SYNC = 0x01,
    CLEAR_SCREEN = 0x11,
    DRAW_TEXT = 0x12,
    // ESP32 -> Teensy Responses
    SYNC_ACK = 0x81,
    ACK = 0x85
};

enum class Alignment : uint8_t {
    CENTERED = 0,
    TOP_LEFT = 1
};

#pragma pack(push, 1)
struct DrawTextPayload {
    uint8_t textSize;
    Alignment alignment;
    int16_t x;
    int16_t y;
    // Null-terminated string follows
};
#pragma pack(pop)

// --- State Management ---
static bool isConnected = false;

// --- UART Packet Parsing State Machine ---
enum class ParseState {
    WAIT_FOR_START, WAIT_FOR_TYPE, WAIT_FOR_LEN, READ_PAYLOAD, WAIT_FOR_CHECKSUM
};
static ParseState currentState = ParseState::WAIT_FOR_START;
static uint8_t calculatedChecksum = 0;
static PacketType receivedType;
static uint8_t payloadLength = 0;
static uint8_t payloadBuffer[256];
static uint8_t payloadIndex = 0;

// --- Forward Declarations ---
// LVGL
static uint32_t my_tick(void);
void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data);
static void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data);
void screenBrightness(uint8_t value);
// Communication
void processIncomingUart();
void handlePacket(PacketType type, const uint8_t* payload, uint8_t len);
void sendResponse(PacketType type);
// Drawing
void drawTextFromPacket(const uint8_t* payload, uint8_t len);
void updateConnectionStatus(const char* status, lv_color_t color);

// --- Main HAL Functions ---

void hal_setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting ESP32 Slave HAL setup...");

    // Using Serial1 for Teensy communication
    Serial1.setPins(20, 21);
    Serial1.setRxBufferSize(1024); // Add this line BEFORE begin()
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

    // Initialize and apply the theme, this sets the background color
    lv_display_set_theme(lvDisplay, lv_theme_default_init(lvDisplay, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), 1, &lv_font_montserrat_14));

    // --- UI Setup for Mirrored Display ---
    lv_obj_t *bg = lv_screen_active();

    // 1. Connection Status Label (Top)
    connection_label = lv_label_create(bg);
    lv_obj_align(connection_label, LV_ALIGN_TOP_MID, 0, 5);
    updateConnectionStatus("Waiting...", lv_color_hex(0xFFFF00)); // Yellow

    // 2. Main Text Label (for mirrored content)
    main_text_label = lv_label_create(bg);
    lv_obj_set_width(main_text_label, screenWidth - 20); // Add some padding
    lv_label_set_long_mode(main_text_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(main_text_label, "Waiting for\nConnection...");
    lv_obj_set_style_text_align(main_text_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(main_text_label, LV_ALIGN_CENTER, 0, 0);

    Serial.println("HAL Setup complete. Waiting for master...");
}

void hal_loop() {
    static unsigned long lastPrint = 0;
    static ParseState lastPrintedState = ParseState::WAIT_FOR_START;
    
    processIncomingUart();
    lv_timer_handler();
    
    // Print state every second if it changes
    if (millis() - lastPrint > 1000) {
        if (currentState != lastPrintedState) {
            Serial.printf("Current parser state: %d\n", (int)currentState);
            lastPrintedState = currentState;
        }
        lastPrint = millis();
    }

}

// --- Core Communication Logic ---

void processIncomingUart() {
    static unsigned long lastByteTime = 0;

    while (Serial1.available()) {
        uint8_t byte = Serial1.read();
        lastByteTime = millis();

        switch (currentState) {
            case ParseState::WAIT_FOR_START:
                if (byte == START_BYTE) {
                    calculatedChecksum = byte;
                    currentState = ParseState::WAIT_FOR_TYPE;
                    Serial.println("\n--- New Packet ---");
                    Serial.printf("START: 0x%02X\n", byte);
                }
                break;
                
            case ParseState::WAIT_FOR_TYPE:
                receivedType = (PacketType)byte;
                calculatedChecksum ^= byte;
                currentState = ParseState::WAIT_FOR_LEN;
                Serial.printf("TYPE:  0x%02X\n", byte);
                break;
                
            case ParseState::WAIT_FOR_LEN:
                payloadLength = byte;
                calculatedChecksum ^= byte;
                payloadIndex = 0;
                currentState = (payloadLength == 0) ? ParseState::WAIT_FOR_CHECKSUM : ParseState::READ_PAYLOAD;
                Serial.printf("LEN:   %d\n", payloadLength);
                if (payloadLength > 0) {
                    Serial.print("PAYLOAD: ");
                }
                break;
                
            case ParseState::READ_PAYLOAD:
                if (payloadIndex < sizeof(payloadBuffer)) {
                    payloadBuffer[payloadIndex++] = byte;
                    calculatedChecksum ^= byte;
                    Serial.printf("0x%02X ", byte);
                    
                    if (payloadIndex == payloadLength) {
                        Serial.println(); // End payload line
                        currentState = ParseState::WAIT_FOR_CHECKSUM;
                    }
                } else {
                    Serial.println("\nERROR: Payload buffer overflow!");
                    currentState = ParseState::WAIT_FOR_START;
                    calculatedChecksum = 0;
                }
                break;
                
            case ParseState::WAIT_FOR_CHECKSUM:
                Serial.printf("CHKSUM: 0x%02X (expected 0x%02X)\n", byte, calculatedChecksum);
                if (byte == calculatedChecksum) {
                    Serial.printf("✓ Packet Valid: type=0x%02X len=%d\n", (uint8_t)receivedType, payloadLength);
                    
                    // Decode and print text if it's a DRAW_TEXT packet
                    if (receivedType == PacketType::DRAW_TEXT && payloadLength >= sizeof(DrawTextPayload)) {
                        const DrawTextPayload* header = (const DrawTextPayload*)payloadBuffer;
                        const char* text = (const char*)(payloadBuffer + sizeof(DrawTextPayload));
                        Serial.printf("  └─> Text: \"%s\" (size=%d, align=%d)\n", 
                                     text, header->textSize, (int)header->alignment);
                    }
                    
                    handlePacket(receivedType, payloadBuffer, payloadLength);
                } else {
                    Serial.printf("✗ Checksum Mismatch!\n");
                }
                currentState = ParseState::WAIT_FOR_START;
                calculatedChecksum = 0;
                break;
        }
    }

    // Timeout recovery: if we're stuck mid-packet for >50ms, reset
    if (currentState != ParseState::WAIT_FOR_START &&
        millis() - lastByteTime > 200) {
        Serial.println("⚠ Parser timeout - resetting to WAIT_FOR_START");
        currentState = ParseState::WAIT_FOR_START;
        calculatedChecksum = 0;
    }
}

void handlePacket(PacketType type, const uint8_t* payload, uint8_t len) {
    // Only require connection for non-SYNC packets
    if (!isConnected && type != PacketType::SYNC) return;

    switch (type) {
        case PacketType::SYNC:
            if (!isConnected) {
                isConnected = true;
                Serial.println("Connection established with Teensy.");
                updateConnectionStatus("Connected", lv_color_hex(0x00FF00)); // Green
                lv_label_set_text(main_text_label, ""); // Clear waiting message
            }
            sendResponse(PacketType::SYNC_ACK);
            break;
        case PacketType::CLEAR_SCREEN:
            Serial.println("CMD: Clear Screen");
            lv_label_set_text(main_text_label, "");
            sendResponse(PacketType::ACK);
            break;
        case PacketType::DRAW_TEXT:
            Serial.println("CMD: Draw Text");
            drawTextFromPacket(payload, len);
            sendResponse(PacketType::ACK);
            break;
        default:
            Serial.printf("Warning: Unknown packet type received: 0x%02X\n", (uint8_t)type);
            break;
    }
}

void sendResponse(PacketType type) {
    uint8_t len = 0;
    uint8_t checksum = START_BYTE ^ (uint8_t)type ^ len;
    Serial1.write(START_BYTE);
    Serial1.write((uint8_t)type);
    Serial1.write(len);
    Serial1.write(checksum);
    Serial1.flush();
    Serial.printf("TX response: type 0x%02X (full packet)\n", (uint8_t)type);  // Debug
}

// --- Drawing Logic ---

void drawTextFromPacket(const uint8_t* payload, uint8_t len) {
    Serial.printf("DEBUG: drawTextFromPacket - len: %d\n", len);
    if (len < sizeof(DrawTextPayload)) {
        Serial.println("Error: DRAW_TEXT payload is too short.");
        return;
    }

    const DrawTextPayload* header = (const DrawTextPayload*)payload;
    const char* text = (const char*)(payload + sizeof(DrawTextPayload));
    size_t maxTextLen = len - sizeof(DrawTextPayload);

    // Optional: Hex dump the received string data
    Serial.print("Text payload (hex): ");
    for(size_t i = 0; i < maxTextLen; ++i) {
        Serial.printf("%02X ", text[i]);
    }
    Serial.println();

    if (strnlen(text, maxTextLen) >= maxTextLen) {
        Serial.println("Error: Received text payload is not null-terminated.");
        return;
    }

    lv_label_set_text(main_text_label, text);

    // Map the textSize from Teensy (GFX style) to an appropriate LVGL font.
    const lv_font_t *font;
    switch (header->textSize) {
        case 1:
            font = &lv_font_montserrat_14;
            break;
        case 2:
            font = &lv_font_montserrat_24;
            break;
        case 3:
            font = &lv_font_montserrat_32;
            break;
        default:
            font = &lv_font_montserrat_14;
            break;
    }
    lv_obj_set_style_text_font(main_text_label, font, 0);

    if (header->alignment == Alignment::CENTERED) {
        lv_obj_set_style_text_align(main_text_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(main_text_label, LV_ALIGN_CENTER, 0, 0);
    } else {
        lv_obj_set_style_text_align(main_text_label, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(main_text_label, LV_ALIGN_TOP_LEFT, header->x, header->y);
    }
}

void updateConnectionStatus(const char* status, lv_color_t color) {
    if (connection_label) {
        lv_label_set_text(connection_label, status);
        lv_obj_set_style_text_color(connection_label, color, 0);
    }
    Serial.println(status);
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
