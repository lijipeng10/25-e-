#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H


#define D36A_MICROSTEP              32U
#define MOTOR_STEPS_PER_REV         (200U * D36A_MICROSTEP)


#define TEST_MOTOR_RPM              10U
#define TEST_FORWARD_DIR_LEVEL      1U


#define ENCODER_COUNTS_PER_REV      4000U

#define ENCODER_AXIS_X_SIGN         1

#define APP_TICK_MS                 5U
#define KEY_DEBOUNCE_MS             20U
#define OLED_REFRESH_MS             200U
#define SERIAL_PRINT_MS             1000U

#if ((KEY_DEBOUNCE_MS % APP_TICK_MS) != 0U)
#error "KEY_DEBOUNCE_MS must be a multiple of APP_TICK_MS"
#endif

#endif
