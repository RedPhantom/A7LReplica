// Author: Itay Asayag

// INCLUDES
#include <Arduino.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// MACROS

// Push-to-talk
// TODO: Validate the following line, it may be incorrect.
// Pin D18, INPUT, supports interrupts.
#define IN_BUTTON_PTT (3)

// Audio
// External speaker, OUTPUT, connected in-line. Marked "AUD OVERRIDE".
#define OUT_SPEAKER (7)

// Temperature
// Temperature sensor count.
#define TEMP_SENSOR_COUNT (4)
// Pin A0.
#define AIN_TEMP_1 (0)
// Pin A1.
#define AIN_TEMP_2 (1)
// Pin A2.
#define AIN_TEMP_3 (2)
// Pin A3.
#define AIN_TEMP_4 (3)
// Pin A10, pump flow rate input via potentiometer.
#define AIN_FLOW_RATE (10)
// Pin A11, battery voltage measurement.
#define AIN_BATTERY_VCC (11)
// Temperature measurement reference resistor value, Ohm.
#define TEMP_REFERENCE_OHM (10000)
// Temperature measurement constants.
#define TEMP_CONST_1 (1.009249522e-03)
#define TEMP_CONST_2 (2.378405444e-04)
#define TEMP_CONST_3 (2.019202697e-07)
// Number of diagnostic temperature measurements to perform.
#define TEMP_DIAGNOSTIC_CYCLE_COUNT (15)
// Maxmimum temperature difference to measure 
// between diagnostic maximum and minimum, Celsius.
#define TEMP_DIAGNOSTIC_MAX_DIFFERENCE (15)
// Delay between diagnostic measurements, milliseconds.
#define TEMP_DIAGNOSTIC_MEASUREMENT_INTERVAL (10)
// Minimum sensible temperature that can be measured 
// when performing temperature diagnostics, Celsius.
#define TEMP_DIAGNOSTIC_TEMP_MIN (0)
// Maximum sensible temperature that can be measured 
// when performing temperature diagnostics, Celsius.
#define TEMP_DIAGNOSTIC_TEMP_MAX (50)

// Water Pump
// Pin D3, H-Bridge ENABLE-A pin.
#define OUT_WATER_ENABLE (8)
// Pin D4, H-Bridge IN1 pin.
#define OUT_WATER_IN_1 (9)
// Pin D5, H-Bridge IN2 pin.
#define OUT_WATER_IN_2 (10)
// Duration of each cooling cycle, seconds.
#define CLIMATE_CYCLE_DURATION_SECS (10)

// Power Management
// Battery maximum voltage, Volts.
#define BATTERY_VOLTAGE_MAX (6.7)
// Battery minimum voltage, Volts.
#define BATTERY_VOLTAGE_MIN (5.25)

// I2C Interface
// I2C LCD address.
#define LCD_ADDR (0x27)

// Quindar Tones
// Transmit Quindar tone, Hz.
#define QUINDAR_FREQ_TX (2525)
// Receive Quindar tone, Hz.
#define QUINDAR_FREQ_RX (2475)
// Duration of Quindar tone, ms.
#define QUINDAR_DURATION_MS (250)

// Serial Communication
#define SERIAL_BAUD_RATE (9600)

// LCD
// Width of the LCD, characters.
#define LCD_WIDTH (16)
// Height of the LCD, characters.
#define LCD_HEIGHT (2)
// Digital pin controlling LCD backlight.
#define LCD_PIN_BACKLIGHT (3)

#define LCD_SIGN_DEGREE {0x18, 0x18, 0x07, 0x08, 0x08, 0x08, 0x08, 0x07}
#define LCD_SIGN_DEGREE_INDEX (0)
#define LCD_SIGN_PTT_ON {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04}
#define LCD_SIGN_PTT_ON_INDEX (1)
#define LCD_SIGN_PTT_OFF {0x04, 0x0A, 0x11, 0x04, 0x0A, 0x11, 0x04, 0x04}
#define LCD_SIGN_PTT_OFF_INDEX (2)

// Consts
#define UINT10_MIN (0)
#define UINT10_MAX ((1 << 10) - 1)
#define UINT10_MAX_FLOAT (UINT10_MAX * 1.0)

#define PERCENT_MIN (0)
#define PERCENT_MAX (100)
#define PERCENT_MAX_FLOAT (PERCENT_MAX * 1.0)

#define KELVIN_CELSIUS_OFFSET (273.15)

// Minimum internal reference voltage, Volt, hundredths (e.g. 100 = 1.0V).
#define INTERNAL_VCC_MIN (425)
// Maximum internal reference voltage, Volt, hundredths.
#define INTERNAL_VCC_MAX (550)

// Core
// Execution frequency of the loop() method.
#define PROGRAM_FREQUENCY (10)

// TYPES

typedef uint16_t size_t;

// ENUMS

typedef enum 
{
    RETURN_CODE__UNINITIALIZED = -1,
    RETURN_CODE__SUCCESS = 0,
    
    // A null pointer has been detected.
    RETURN_CODE__NULL_POINTER,

    // Expected value bounds of a sensor or device are exceeded
    // by measured values.
    RETURN_CODE__TEMP_TOO_HIGH,
    RETURN_CODE__TEMP_TOO_LOW,

    // Exceeded internal voltage.
    RETURN_CODE__VCC_TOO_HIGH,
    RETURN_CODE__VCC_TOO_LOW,

    // Exceeded battery voltage.
    RETURN_CODE__BATTERY_TOO_HIGH,
    RETURN_CODE__BATTERY_TOO_LOW,

    // Expected maximum difference between measured values
    // of a sensor are exceeded.
    RETURN_CODE__UNSTABLE_VALUE_TEMP,

    // Memory allocation operation has failed.
    RETURN_CODE__ALLOCATION_FAILED,

} return_code_t;

// STRUCTS

typedef struct
{
    // Array of values measured from sensors, Celsius.
    uint8_t values[TEMP_SENSOR_COUNT];

} temperatures_t;

typedef struct
{
    // Water flow rate.
    uint16_t flow_rate;

    // Counter for loop() cycles the pump should be running.
    uint8_t run_cycles;

    // Counter for loop() cycles the pump should be idle.
    uint8_t stop_cycles;
} climate_control_t;

typedef struct
{
    // Whether push-to-talk is pressed.
    bool is_on;
    
    // TODO: What does this do?
    bool should_listen_for_dn;
} ptt_t;

typedef struct
{
    // Counter that tracks the program cycles until the next LCD refresh event,
    // during which this counter is reset to 0.
    uint8_t refresh_cycle;
} lcd_t;

// Holds the suite's entire state.
typedef struct 
{   
    // Hold temperature sensor data.
    temperatures_t temperatures;

    // Hold climate control status.
    climate_control_t climate_control;

    // Hold Push-to-talk status.
    ptt_t ptt;

    // Hold LCD status.
    lcd_t lcd;

    // Level of the battery, percentage.
    uint8_t battery_level;
} suite_state_t;

// GLOBALS

LiquidCrystal_I2C g_lcd(LCD_ADDR, 2, 1, 0, 4, 5, 6, 7);
suite_state_t * g_suite_ptr = NULL;

// FUNCTIONS

/**
 * Check the PTT pin, playing a Quindar tone if needed.
 * 
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */
return_code_t ptt__check(suite_state_t * const suite_state)
{   
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    int is_on = digitalRead(IN_BUTTON_PTT);
    int should_listen_for_dn = suite_state->ptt.should_listen_for_dn;

    if (is_on && (!should_listen_for_dn))
    {
        should_listen_for_dn = true;
        is_on = false;
        tone(OUT_SPEAKER, QUINDAR_FREQ_TX, QUINDAR_DURATION_MS);
    }
    else if ((!is_on) && should_listen_for_dn)
    {
        should_listen_for_dn = false;
        tone(OUT_SPEAKER, QUINDAR_FREQ_RX, QUINDAR_DURATION_MS);
    }

    suite_state->ptt.is_on = is_on;
    suite_state->ptt.should_listen_for_dn = should_listen_for_dn;
    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Initialize the LCD display.
 * 
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */
return_code_t lcd__init(void)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    uint8_t degree[] = LCD_SIGN_DEGREE;
    uint8_t cpttOff[] = LCD_SIGN_PTT_ON;
    uint8_t cpttOn[] = LCD_SIGN_PTT_OFF;

    // Begin I2C.
    g_lcd.begin(LCD_WIDTH, LCD_HEIGHT);
    g_lcd.setBacklightPin(LCD_PIN_BACKLIGHT, POSITIVE);
    g_lcd.setBacklight(HIGH);

    // Add custom characters.
    g_lcd.createChar(LCD_SIGN_DEGREE_INDEX, degree);
    g_lcd.createChar(LCD_SIGN_PTT_OFF_INDEX, cpttOff);
    g_lcd.createChar(LCD_SIGN_PTT_ON_INDEX, cpttOn);

    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Initialize the suit.
 * 
 * @param[in,out] suite Pointer to an suite state struct.
 * @return return_code_t 
 */
return_code_t core__init(suite_state_t ** suite)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    suite_state_t * prepared_suite_state = NULL;

    // Validate received pointer.
    if (NULL == suite)
    {
        return_code = RETURN_CODE__NULL_POINTER;
        goto l_cleanup;
    }

    // Allocate memory for the suite status.
    prepared_suite_state = (suite_state_t *)malloc(sizeof(suite_state_t));
    if (NULL == prepared_suite_state)
    {
        return_code = RETURN_CODE__ALLOCATION_FAILED;
        goto l_cleanup;
    }

    *suite = prepared_suite_state
    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

void setup()
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    suite_state_t * suite = NULL;

    // Initialize serial connection.
    // TODO

    // Initialize suite state.
    return_code = core__init(&suite);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    // attach interrupt for ptt button change.
    // attachInterrupt(digitalPinToInterrupt(IN_BUTTON_PTT), pttChange, CHANGE );
    digitalWrite(IN_BUTTON_PTT, HIGH);

    // verbose debugging on serial.
    Serial.begin(SERIAL_BAUD_RATE);

    // setup the LCD.
    lcd__init();    // TODO: check return code

    // Configure input/output pins.
    // Inputs - Analog
    pinMode(AIN_TEMP_1,       INPUT);
    pinMode(AIN_TEMP_2,       INPUT);
    pinMode(AIN_TEMP_3,       INPUT);
    pinMode(AIN_TEMP_4,       INPUT);
    pinMode(AIN_FLOW_RATE,    INPUT);
    pinMode(AIN_BATTERY_VCC,  INPUT);

    // Inputs - Digital
    pinMode(IN_BUTTON_PTT,    INPUT);

    // Outputs - Analog
    pinMode(OUT_SPEAKER,      OUTPUT);
    pinMode(OUT_WATER_IN_1,   OUTPUT);
    pinMode(OUT_WATER_IN_2,   OUTPUT);

    // Outputs - Digital
    pinMode(OUT_WATER_ENABLE, OUTPUT);

    // to conclude:
    // performDiagnostics();
l_cleanup:
    
    // TODO
}

void loop()
{
    // TODO: save loop() iteration start time.
    // monitor:

    // ptt
    ptt__check();

    // measure temperatures
    temp1 = (int)measureTemp(AIN_TEMP_1);
    temp2 = (int)measureTemp(AIN_TEMP_2);
    temp3 = (int)measureTemp(AIN_TEMP_3);
    temp4 = (int)measureTemp(AIN_TEMP_4);

    Serial.print(temp1);
    Serial.print(",");
    Serial.print(temp2);
    Serial.print(",");
    Serial.println(temp3);

    //<< (String)temp2 << ", " << (String)temp3);

    // measure power
    power__get_battery_level();

    // measure requested flowrate
    pumpFlowRate = analogRead(AIN_FLOW_RATE);

    // write to LCD:

    if (dispRefresher == 5)
    {                                                                         // frequency of screen updates is dispRefresher-compared-to value divided by 10, in seconds.
        lcd__update(temp1, temp2, temp3, pttOn, pumpFlowRate, battPercent); // DEBUG

        dispRefresher = 0;
    }
    else
    {
        dispRefresher++;
    }

    // modify accordingly:
    // control the pump

    if (pumpCtrOn >= 0 && pumpCtrOff >= 0)
    {

        if (pumpCtrOn > 0)
        {
            digitalWrite(OUT_WATER_ENABLE, HIGH);
            pumpCtrOn--;
        }
        else
        {
            digitalWrite(OUT_WATER_ENABLE, LOW);
            pumpCtrOff--;
        }
    }
    else
    {
        climate__calculate_flow_rate(map(pumpFlowRate, UINT10_MIN, UINT10_MAX, PERCENT_MIN, PERCENT_MAX));
    }

    delay(100); // 10 Hz refresh rate.
    // TODO: wait the remaining time to achieve a `PROGRAM_RATE` frequency.
    // Log an error if it cannot be achieved.
}

/**
 * Measure the temperature from a thermistor connected to
 * an analog input pin.
 * 
 * @param[in]  pin   Analog input pin to which the measured thermistor is connected.
 * @param[out] temp  Temperature measured over the thermistor in Celsius.
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */ 
return_code_t temp__measure(uint8_t pin, float * temp)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    uint16_t sensor_raw_value = 0;
    float complement_resistance = 0;
    float complement_resistance_log = 0;
    float result = 0;

    if (NULL == temp)
    {
        return_code = RETURN_CODE__NULL_POINTER;
        goto l_cleanup;
    }

    sensor_raw_value = analogRead(pin);
    complement_resistance = TEMP_REFERENCE_OHM * (UINT10_MAX_FLOAT / (float)sensor_raw_value - 1.0);
    complement_resistance_log = log(complement_resistance);
    result = (1.0 / (TEMP_CONST_1 + 
                        (TEMP_CONST_2 * complement_resistance_log) + 
                            (
                                TEMP_CONST_3 * pow(complement_resistance_log, 3)
                            )
                        )
            );
    
    *temp = result;
    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Measure and calculate battery power level percentage.
 * 
 * @param[in]  pin    Analog input pin over which the battery voltage is measured.
 * @param[out] level  Measured battery level as a percentage.
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */ 
return_code_t power__get_battery_level(uint8_t pin, uint8_t * level)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    float batteryVoltage = 0;
    int vccVoltage = 0;
    
    if (NULL == level)
    {
        return_code = RETURN_CODE__NULL_POINTER;
        goto l_cleanup;
    }

    // In hunders of volts, so actual value is vccVoltage/100.0.
    vccVoltage = power__get_bandgap();
    batteryVoltage = map(analogRead(AIN_BATTERY_VCC), UINT10_MIN, UINT10_MAX, 0, vccVoltage) / PERCENT_MAX;
    
    // Percentage is calculated by divison in the (maximum battery voltage - minimum battery voltage),
    // i.e., Vmax-Vmin, e.g., 6.6v - 5.25v -> 4.5v.
    *level = (1 - (BATTERY_VOLTAGE_MAX - batteryVoltage) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * PERCENT_MAX;
    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Set the climate control water pump flow rate.
 * 
 * @param[in]  duty_cycle   Duty cycle (percentage) in which the pump should be working.
 * @param[out] run_cycles   Program cycles the pump should be running for.
 * @param[out] stop_cycles  Program cycles the pump should be idle for.
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */ 
return_code_t climate__calculate_flow_rate(uint8_t duty_cycle, uint8_t * run_cycles, uint8_t * stop_cycles)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    uint16_t tick_value = CLIMATE_CYCLE_DURATION_SECS * PROGRAM_FREQUENCY;
    float dc = duty_cycle / PERCENT_MAX_FLOAT;

    if ((NULL == run_cycles) || (NULL == stop_cycles))
    {
        return_code = RETURN_CODE__NULL_POINTER;
        goto l_cleanup;
    }

    *run_cycles = dc * tick_value;
    *stop_cycles = (1 - dc) * tick_value;
    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Update the LCD to represent the latest data.
 * 
 * @param[in] state Suite state.
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 */ 
return_code_t lcd__update(suite_state_t * state)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    size_t i = 0;
    
    if (NULL == state)
    {
        return_code = RETURN_CODE__NULL_POINTER;
        goto l_cleanup;
    }

    // Clear and home.
    g_lcd.clear();
    g_lcd.home();

    // Display temperatures.
    // Values are printed accross
    for (i = 0; i < TEMP_SENSOR_COUNT; ++i)
    {
        g_lcd.print(state->temperatures.values[i]);
        g_lcd.write(LCD_SIGN_DEGREE_INDEX);
    }

    // Display push-to-talk status.
    // Send the cursor to the end of the first line.
    g_lcd.setCursor(LCD_WIDTH - 1, 0);
    if (state.ptt.is_on)
    {
        g_lcd.write(LCD_SIGN_PTT_ON_INDEX);
    }
    else
    {
        g_lcd.write(LCD_SIGN_PTT_OFF_INDEX);
    }

    // Display the battery level and cooling flow rate.
    // Send the cursor to the last row.
    g_lcd.setCursor(0, LCD_HEIGHT - 1);
    
    g_lcd.print("PWR ");
    g_lcd.print(state->battery_level);
    g_lcd.print("% ");

    g_lcd.print("FLOW ");
    g_lcd.print(map(state->climate_control.flow_rate, 0, UINT10_MAX, PERCENT_MIN, PERCENT_MAX));
    g_lcd.print("%");

    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

// Get the internal voltage.
/**
 * Measure the internal processor voltage, used for accurate external voltage measurement.
 * 
 * @return int Measured voltage in hundredths of volts (e.g., a return value of 100 is 1.00V)
 */
int power__get_bandgap()
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // For mega boards
    const long InternalReferenceVoltage = 1115L; // Adjust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
    // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#else
    // For 168/328 boards
    const long InternalReferenceVoltage = 1056L; // Adjust this value to your boards specific internal BG voltage x1000
    // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
    // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#endif
    delay(50); // Let mux settle a little to get a more stable A/D conversion
    // Start a conversion
    ADCSRA |= _BV(ADSC);
    // Wait for it to complete
    while (((ADCSRA & (1 << ADSC)) != 0))
        ;
    // Scale the value
    int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
    return results;
}

/**
 * Validate temperature sensor values. Realistically, they should be in a range of 0 to 50 degrees Celsius.
 * 
 * @param[in] value Measured temperature in Celsius.
 * @return RETURN_CODE__SUCCESS        Success.
 *         RETURN_CODE__UNINITIALIZED  An unexpected error occurred.
 *         RETURN_CODE__TEMP_TOO_LOW   Temperatured measured is lower than `TEMP_DIAGNOSTIC_TEMP_MIN`.
 *         RETURN_CODE__TEMP_TOO_HIGH  Temperatured measured is lower than `TEMP_DIAGNOSTIC_TEMP_MAX`.
 */
return_code_t diagnostics__assert_temp_valid(uint8_t value)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;

    if (value < TEMP_DIAGNOSTIC_TEMP_MIN)
    {
        return_code = RETURN_CODE__TEMP_TOO_LOW;
        goto l_cleanup;
    }

    if (value > TEMP_DIAGNOSTIC_TEMP_MAX)
    {
        return_code = RETURN_CODE__TEMP_TOO_HIGH;
        goto l_cleanup;
    }

    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Validate temperature sensor value range and stability.
 * 
 * @note Maximum function duration: TEMP_DIAGNOSTIC_MEASUREMENT_INTERVAL * TEMP_DIAGNOSTIC_CYCLE_COUNT.
 * @param[in] pin Pin to which the thermistor to be tested is connected.
 * @return RETURN_CODE__SUCCESS              Success.
 *         RETURN_CODE__UNINITIALIZED        An unexpected error occurred.
 *         RETURN_CODE__UNSTABLE_VALUE_TEMP  The measured temperature exceeds TEMP_DIAGNOSTIC_MAX_DIFFERENCE`.
 *         RETURN_CODE__TEMP_TOO_LOW         The measured temperature is lower than the acceptable value.
 *         RETURN_CODE__TEMP_TOO_HIGH        The measured temperature is higher than the acceptable value.
 */
return_code_t diagnostics__assert_temp_sensor_valid(uint8_t pin)
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    uint8_t cycle = 0;
    uint16_t temp_max = 0;  // TODO: Replace with float's minimum value.
    uint16_t temp_min = UINT10_MAX;
    float temp = 0;
    
    return_code = temp__measure(pin, &temp);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }
    
    // Check the stable value is in range.
    return_code = diagnostics__assert_temp_valid(temp);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }
    
    // Check the sensor value is stable.
    for (cycle = 0; cycle < TEMP_DIAGNOSTIC_CYCLE_COUNT; ++cycle)
    {
        return_code = temp__measure(pin, &temp);
        if (RETURN_CODE__SUCCESS != return_code)
        {
            goto l_cleanup;
        }

        if (temp > temp_min)
        {
            temp_min = temp;
        }

        if (temp < temp_max)
        {
            temp_max = temp;
        }

        // Wait for values to stabilize.
        delay(TEMP_DIAGNOSTIC_MEASUREMENT_INTERVAL);
    }

    // Measure the maximum difference of all measurements.
    if ((temp_max - temp_min) > TEMP_DIAGNOSTIC_MAX_DIFFERENCE)
    {
        return_code = RETURN_CODE__UNSTABLE_VALUE_TEMP;
        goto l_cleanup;
    }

    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Perform system-wide diagnostics.
 * 
 * @note Minimum function duration: TBD.
 * @return return_code_t 
 */
return_code_t diagnostics__run()
{
    return_code_t return_code = RETURN_CODE__UNINITIALIZED;
    uint8_t battery_level = 0;

    lcd__write_status("SELF TESTING...");
    delay(1500);

    // Verify valid internal voltage.
    // This must be done before validating battery level since
    // battery level measurement depends on the internal voltage.
    if (power__get_bandgap() > INTERNAL_VCC_MAX)
    {
        return_code = RETURN_CODE__VCC_TOO_HIGH;
        goto l_cleanup;
    }

    if (power__get_bandgap() < INTERNAL_VCC_MIN)
    {
        return_code = RETURN_CODE__VCC_TOO_LOW;
        goto l_cleanup;
    }

    // Verify successful battery level measurement.
    return_code = power__get_battery_level(AIN_BATTERY_VCC, &battery_level);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    // Verify valid battery level.
    if (battery_level < BATTERY_VOLTAGE_MIN)
    {
        return_code = RETURN_CODE__BATTERY_TOO_LOW;
        goto l_cleanup;
    }

    if (battery_level > BATTERY_VOLTAGE_MAX)
    {
        return_code = RETURN_CODE__BATTERY_TOO_HIGH;
        goto l_cleanup;
    }

    // Verify valid and stable temperature range.
    return_code = diagnostics__assert_temp_sensor_valid(AIN_TEMP_1);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    return_code = diagnostics__assert_temp_sensor_valid(AIN_TEMP_2);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    return_code = diagnostics__assert_temp_sensor_valid(AIN_TEMP_3);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    return_code = diagnostics__assert_temp_sensor_valid(AIN_TEMP_4);
    if (RETURN_CODE__SUCCESS != return_code)
    {
        goto l_cleanup;
    }

    return_code = RETURN_CODE__SUCCESS;

l_cleanup:
    return return_code;
}

/**
 * Output a status message to the LCD.
 * 
 * @param[in] s String to write to screen.
 */
void lcd__write_status(String s)
{
    g_lcd.clear();
    g_lcd.home();
    g_lcd.print(s);
}

/**
 * Output an error message to the LCD.
 * @note Minimum function duration: 1000ms.
 * 
 * @param[in] s String to write to screen.
 */
void lcd__write_error(String s)
{
    lcd__write_status("ERR" + s);
    lcd.setBacklight(HIGH);
    delay(200);
    lcd.setBacklight(LOW);
    delay(200);
    lcd.setBacklight(HIGH);
    delay(200);
    lcd.setBacklight(LOW);
    delay(200);
    lcd.setBacklight(HIGH);
    delay(200);
}
