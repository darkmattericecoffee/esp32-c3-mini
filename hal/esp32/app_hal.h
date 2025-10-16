// hal/esp32/app_hal.h

#ifndef DRIVER_H
#define DRIVER_H

// --- All ENABLE_FACE lines should already be commented out ---

// #if defined(ESPS3_1_69) || defined(ESPS3_1_28) || defined(VIEWE_SMARTRING)
// #define ENABLE_APP_QMI8658C
// #define ENABLE_APP_ATTITUDE
// #endif

// #if defined(ELECROW_C3)
// #define ENABLE_RTC
// #endif

// #if defined(M5_STACK_DIAL) || defined(VIEWE_KNOB_15)
// #define ENABLE_APP_RANGE
// #endif

// #define ENABLE_APP_CALENDAR
// #define ENABLE_APP_SAMPLE
// #define ENABLE_GAME_TASK
// #define ENABLE_GAME_RACING
// #define ENABLE_GAME_SIMON
// #define ENABLE_APP_NAVIGATION
// #define ENABLE_APP_CONTACTS
// #define ENABLE_APP_TIMER

#ifdef __cplusplus
extern "C" {
#endif

void hal_setup(void);
void hal_loop(void);
void vibratePin(bool state);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DRIVER_H*/