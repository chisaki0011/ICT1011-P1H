#include "BMA250.h"
#include <SPI.h>
#include <STBLE.h>
#include <TinyScreen.h>
#include <Wire.h>

TinyScreen display = TinyScreen(TinyScreenPlus);
UBMA250 accel;

uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;

#define PIPE_UART_OVER_BTLE_UART_TX_TX 0

unsigned long lastUpdate = 0;
const unsigned long interval = 33; // Read sensor every 500 ms

const int THRESHOLD = 5; // Minimum change (counts) to send

int lastX = 0, lastY = 0;
bool haveLast = false;     // False until first reading
bool wasConnected = false; // Connection state in last loop
bool sendPending = false;  // True if a send must be done or retried

void setup() {
  delay(500);
  Wire.begin();

  display.begin();
  display.setBrightness(10);
  display.setFont(thinPixel7_10ptFontInfo);
  display.clearScreen();

  display.setCursor(0, 0);
  display.fontColor(TS_8b_Green, TS_8b_Black);
  display.print("BLE Live Data");

  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);
  BLEsetup();
}

void loop() {
  aci_loop();

  // New connection: send the current value one time
  if (ble_connection_state && !wasConnected) {
    sendPending = true;
  }
  wasConnected = ble_connection_state;

  if (millis() - lastUpdate < interval)
    return;
  lastUpdate = millis();

  accel.read();
  int x = accel.X;
  int y = accel.Y;

  bool changed =
      !haveLast || abs(x - lastX) > THRESHOLD || abs(y - lastY) > THRESHOLD;

  if (changed) {
    lastX = x;
    lastY = y;
    haveLast = true;
    sendPending = true;

    // Update screen only on change. Block BLE interrupt during SPI use.
    char line[16];
    NVIC_DisableIRQ(EIC_IRQn);
    display.fontColor(TS_8b_White, TS_8b_Black);
    snprintf(line, sizeof(line), "X: %d     ", x);
    display.setCursor(0, 16);
    display.print(line);
    snprintf(line, sizeof(line), "Y: %d     ", y);
    display.setCursor(0, 28);
    display.print(line);
    NVIC_EnableIRQ(EIC_IRQn);
  }

  // Send one packet only if needed
  if (sendPending && ble_connection_state) {
    char sendBuffer[20];
    int len =
        snprintf(sendBuffer, sizeof(sendBuffer), "X%d,Y%d\n", lastX, lastY);

    // lib_aci_send_data returns 1 on success
    if (lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)sendBuffer,
                          len)) {
      sendPending = false; // Sent. Wait for next change.
    } // Failed: try again in next 500 ms cycle
  }
}
