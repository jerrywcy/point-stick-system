#include "RFID.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define CS1 5
#define CS2 6
#define RST1 7
#define RST2 8
#define IRQ_PIN 16

CLRC663 reader1(CS1, RST1, IRQ_PIN), reader2(CS2, RST2, IRQ_PIN);

void print_block(uint8_t *block, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (block[i] < 16) {
            Serial.print("0");
            Serial.print(block[i], HEX);
        } else {
            Serial.print(block[i], HEX);
        }
        Serial.print(" ");
    }
    Serial.println("");
}

void setup() {
    Serial.begin(115200);
    reader1.begin();
    reader2.begin();
}

void loop() {
    reader1.soft_reset();
    reader1.load_protocol();

    uint8_t uid[10] = {0}; // variable for 10byte UID
    bool cardFound = false;
    uint8_t uid_len = reader1.read_tag(uid);
    if (uid_len != 0) {
        Serial.print("ISO-15693 tag found! UID of ");
        Serial.print(uid_len);
        Serial.print(" bytes: ");
        print_block(uid, uid_len);
        Serial.print("\n");
        delay(100);
        return;
    }
    // no card found!
    //
}
