#include <unity.h>
#include <stdint.h>
#include <Arduino.h>

// --- Gateway-Logik-Funktionen zur Testvalidierung ---

String formatFloatComma(float val) {
    String s = String(val, 1);
    s.replace('.', ',');
    return s;
}

int calculateLeds(uint16_t ldr) {
    int leds = 0;
    if (ldr < 800) leds = 1;
    if (ldr < 650) leds = 2;
    if (ldr < 450) leds = 3;
    if (ldr < 300) leds = 4;
    return leds;
}

// --- Unit Tests ---

void test_float_comma_formatting(void) {
    TEST_ASSERT_EQUAL_STRING("20,1", formatFloatComma(20.1f).c_str());
    TEST_ASSERT_EQUAL_STRING("5,5", formatFloatComma(5.5f).c_str());
}

void test_node1_payload_decoding(void) {
    // CAN-Frame für 0x101 (Node 1): B0=Switch(1), B1=LDR_Low(232), B2=LDR_High(3) -> LDR = 1000
    uint8_t data[8] = {1, 232, 3, 0, 0, 0, 0, 0};
    uint8_t sw = data[0];
    uint16_t ldr = (data[2] << 8) | data[1];

    TEST_ASSERT_EQUAL_UINT8(1, sw);
    TEST_ASSERT_EQUAL_UINT16(1000, ldr);
}

void test_node2_distance_decoding(void) {
    // CAN-Frame für 0x102 (Node 2 - Ultraschall): B0=High-Byte, B1=Low-Byte (z.B. 162 cm)
    uint8_t data[8] = {0, 162, 0, 0, 0, 0, 0, 0};
    uint16_t distance = (data[0] << 8) | data[1];

    TEST_ASSERT_EQUAL_UINT16(162, distance);
}

void test_node3_climate_decoding(void) {
    // CAN-Frame für 0x103 - 0x105 (Node 3 - Temp & Feuchte x10)
    // Temp: raw 201 -> 20,1 °C | Feuchte: raw 550 -> 55,0 % (High=2, Low=38 -> 2*256 + 38 = 550)
    uint8_t data[8] = {0, 201, 2, 38, 0, 0, 0, 0};
    
    int16_t rawTemp = (data[0] << 8) | data[1];
    int16_t rawHum = (data[2] << 8) | data[3];
    
    float temp = rawTemp / 10.0;
    float hum = rawHum / 10.0;

    TEST_ASSERT_FLOAT_WITHIN(0.01, 20.1, temp);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 55.0, hum);
}

void test_lighting_logic(void) {
    // Test der LDR-abhängigen Schwellwerte für die 4 LED-Stufen
    TEST_ASSERT_EQUAL(0, calculateLeds(900)); // Hell -> 0 LEDs
    TEST_ASSERT_EQUAL(1, calculateLeds(750)); // < 800 -> Stufe 1
    TEST_ASSERT_EQUAL(2, calculateLeds(600)); // < 650 -> Stufe 2
    TEST_ASSERT_EQUAL(3, calculateLeds(400)); // < 450 -> Stufe 3
    TEST_ASSERT_EQUAL(4, calculateLeds(200)); // < 300 -> Stufe 4
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    
    RUN_TEST(test_float_comma_formatting);
    RUN_TEST(test_node1_payload_decoding);
    RUN_TEST(test_node2_distance_decoding);
    RUN_TEST(test_node3_climate_decoding);
    RUN_TEST(test_lighting_logic);
    
    UNITY_END();
}

void loop() {
    // Leer bei Embedded Tests
}