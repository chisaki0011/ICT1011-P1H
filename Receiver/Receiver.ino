#include <SPI.h>
#include <STBLE.h>
#include <TinyScreen.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

// Receiver only: no accelerometer library, initialization, or sensor reads.
TinyScreen display(TinyScreenPlus);
const uint8_t serviceUUID[16] = {
  0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
};
const uint8_t notifyUUID[16] = {
  0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E
};

enum State { RESTART, START_SCAN, SCANNING, STOP_SCAN, WAIT_SCAN_STOP, PICK_DEVICE,
             CONNECT, CONNECTING, FIND_SERVICE, SERVICE_PENDING,
             FIND_CHARS, CHARS_PENDING, FIND_CCCD, CCCD_PENDING,
             SUBSCRIBE, SUBSCRIBE_PENDING, RECEIVING };
State state = RESTART;
uint32_t stateSince = 0;
uint16_t connection = 0, serviceStart = 0, serviceEnd = 0;
uint16_t notifyHandle = 0, descriptorEnd = 0, cccdHandle = 0;
uint8_t peerAddress[6], peerAddressType;
bool haveSample = false, screenDirty = true;
int remoteX = 0, remoteY = 0;
char lineBuffer[64];
size_t lineLength = 0;
bool droppingLine = false;

uint16_t read16(const uint8_t *p) { return p[0] | (uint16_t(p[1]) << 8); }
void changeState(State next) {
  state = next;
  stateSince = millis();
  screenDirty = true;
}
void commandResult(uint8_t status, State next) {
  if (status == BLE_STATUS_SUCCESS) changeState(next);
  else {
    SerialUSB.print("BLE command failed: 0x");
    SerialUSB.println(status, HEX);
    changeState(RESTART);
  }
}

// Stable insertion order keeps the selection still while advertisements arrive.
struct Device {
  uint8_t address[6], addressType;
  char name[20];
  int8_t rssi;
  bool connectable;
};
const uint8_t MAX_DEVICES = 24;
Device devices[MAX_DEVICES];
uint8_t deviceCount = 0, selectedDevice = 0;
bool selectionPending = false;

void recordDevice(uint8_t type, const uint8_t *address, uint8_t eventType,
                  const uint8_t *data, uint8_t length, int8_t rssi) {
  if (state != SCANNING) return;
  uint8_t index = 0;
  while (index < deviceCount && (devices[index].addressType != type ||
         memcmp(devices[index].address, address, 6))) ++index;
  if (index == deviceCount) {
    if (deviceCount == MAX_DEVICES) return;
    memset(&devices[index], 0, sizeof(Device));
    memcpy(devices[index].address, address, 6);
    devices[index].addressType = type;
    ++deviceCount;
  }
  Device &device = devices[index];
  device.rssi = rssi;
  if (eventType == ADV_IND || eventType == ADV_DIRECT_IND) device.connectable = true;
  for (unsigned offset = 0; offset < length;) {
    uint8_t size = data[offset];
    if (!size || offset + size + 1 > length) break;
    uint8_t adType = data[offset + 1];
    if ((adType == 0x08 || adType == 0x09) && size > 1) {
      unsigned count = size - 1;
      if (count >= sizeof(device.name)) count = sizeof(device.name) - 1;
      for (unsigned i = 0; i < count; ++i) {
        char c = data[offset + 2 + i];
        device.name[i] = (c >= 32 && c <= 126) ? c : '?';
      }
      device.name[count] = '\0';
    }
    offset += size + 1;
  }
  screenDirty = true;
}

void pollButtons() {
  static uint8_t rawPrevious = 0, stable = 0;
  static uint32_t changedAt = 0;
  uint8_t raw = display.getButtons();
  if (raw != rawPrevious) { rawPrevious = raw; changedAt = millis(); }
  if (millis() - changedAt < 35 || raw == stable) return;
  uint8_t pressed = raw & ~stable;
  stable = raw;
  if (pressed & TSButtonLowerRight) {
    // Resetting the radio also safely cancels scanning or a connection.
    changeState(RESTART);
    return;
  }
  if (state != SCANNING && state != PICK_DEVICE) return;
  if (!deviceCount) return;
  if (pressed & TSButtonUpperLeft) selectedDevice = (selectedDevice + deviceCount - 1) % deviceCount;
  if (pressed & TSButtonLowerLeft) selectedDevice = (selectedDevice + 1) % deviceCount;
  if ((pressed & TSButtonUpperRight) && devices[selectedDevice].connectable) {
    memcpy(peerAddress, devices[selectedDevice].address, 6);
    peerAddressType = devices[selectedDevice].addressType;
    selectionPending = true;
    changeState(state == SCANNING ? STOP_SCAN : CONNECT);
  }
  screenDirty = true;
}

void receiveBytes(const uint8_t *data, unsigned length) {
  for (unsigned i = 0; i < length; ++i) {
    char c = data[i];
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      int x, y, consumed = 0;
      // Sender protocol: X<signed count>,Y<signed count>,Z<signed count>\n.
      if (!droppingLine && sscanf(lineBuffer, "X%d,Y%d%n", &x, &y, &consumed) == 2 &&
          consumed == int(lineLength)) {
        remoteX = x; remoteY = y;
        haveSample = true;
        screenDirty = true;
      }
      lineLength = 0;
      droppingLine = false;
    } else if (!droppingLine) {
      if (lineLength < sizeof(lineBuffer) - 1) lineBuffer[lineLength++] = c;
      else { lineLength = 0; droppingLine = true; }
    }
  }
}

uint8_t initializeRadio() {
  deviceCount = selectedDevice = 0;
  selectionPending = false;
  haveSample = false;
  lineLength = 0;
  droppingLine = false;
  serviceStart = serviceEnd = notifyHandle = descriptorEnd = cccdHandle = 0;
  HCI_Init();
  BNRG_SPI_Init();
  BlueNRG_RST();
  // Different from the reference emitter's public address.
  uint8_t address[] = {0x13,0x34,0x00,0xE1,0x80,0x02};
  uint8_t ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, address);
  if (ret) return ret;
  ret = aci_gatt_init();
  if (ret) return ret;
  uint16_t service, name, appearance;
  ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 7, &service, &name, &appearance);
  if (ret) return ret;
  return aci_hal_set_tx_power_level(1, 3);
}

void drawScreen() {
  display.clearScreen();
  display.setCursor(0, 0);
  display.fontColor(TS_8b_Green, TS_8b_Black);
  if (state == SCANNING || state == PICK_DEVICE || state == START_SCAN) {
    display.print(state == PICK_DEVICE ? "Devices " : "Scanning ");
    display.print(deviceCount);
    display.fontColor(TS_8b_White, TS_8b_Black);
    if (!deviceCount) {
      display.setCursor(0, 16); display.print("No devices yet");
    } else {
      Device &device = devices[selectedDevice];
      display.setCursor(0, 11);
      display.print(selectedDevice + 1); display.print("/"); display.print(deviceCount);
      display.print(" "); display.print(device.rssi); display.print("dBm");
      char label[17];
      snprintf(label, sizeof(label), "%.16s", device.name[0] ? device.name : "(unnamed)");
      display.setCursor(0, 22); display.print(label);
      char address[18];
      snprintf(address, sizeof(address), "%02X:%02X:%02X:%02X:%02X:%02X",
               device.address[5], device.address[4], device.address[3],
               device.address[2], device.address[1], device.address[0]);
      display.setCursor(0, 33); display.print(address);
      display.setCursor(0, 44);
      display.print(device.connectable ? "UL up UR link" : "Not connectable");
    }
    display.setCursor(0, 54); display.print("LL down LR scan");
    return;
  }
  display.print("BLE Receiver");
  display.fontColor(TS_8b_White, TS_8b_Black);
  display.setCursor(0, 14);
  if (state == RECEIVING) {
    if (!haveSample) {
      display.print("Waiting data");
      // The sender's connection-time sample can precede our subscription.
      display.setCursor(0, 26); display.print("Move sender");
      return;
    }
    // Sender transmits only on change (>5 counts), with no heartbeat.
    display.print("Last sample");
    display.setCursor(0, 26); display.print("X: "); display.print(remoteX);
    display.setCursor(0, 38); display.print("Y: "); display.print(remoteY);
    
  } else if (state == SCANNING || state == START_SCAN) display.print("Searching...");
  else if (state == CONNECTING || state == CONNECT) display.print("Connecting...");
  else if (state == RESTART) display.print("Retrying...");
  else display.print("Subscribing...");
}

void setup() {
  SerialUSB.begin(115200); // No wait for USB: boot works without a computer.
  delay(500);
  Wire.begin();
  display.begin();
  display.setBrightness(10);
  display.setFont(thinPixel7_10ptFontInfo);
}

void loop() {
  // HCI queues and SPI are not initialized until the first radio restart.
  if (state != RESTART) HCI_Process();
  pollButtons();
  // Recover from lost procedure events, failed connections, or a stalled stack.
  if (state != RESTART && state != RECEIVING && state != PICK_DEVICE && state != SCANNING && millis() - stateSince > 20000)
    changeState(RESTART);
  switch (state) {
    case RESTART:
      if (millis() - stateSince >= 1000) commandResult(initializeRadio(), START_SCAN);
      break;
    case START_SCAN:
      commandResult(aci_gap_start_general_discovery_proc(0x60, 0x30, PUBLIC_ADDR, 0), SCANNING);
      break;
    case SCANNING:
      if (millis() - stateSince >= 10000) changeState(STOP_SCAN);
      break;
    case STOP_SCAN:
      commandResult(aci_gap_terminate_gap_procedure(GAP_GENERAL_DISCOVERY_PROC), WAIT_SCAN_STOP);
      break;
    case CONNECT:
      commandResult(aci_gap_create_connection(0x60, 0x30, peerAddressType, peerAddress,
                    PUBLIC_ADDR, 24, 40, 0, 400, 0, 0), CONNECTING);
      break;
    case FIND_SERVICE:
      commandResult(aci_gatt_disc_prim_service_by_uuid(connection, UUID_TYPE_128,
                    const_cast<uint8_t *>(serviceUUID)), SERVICE_PENDING);
      break;
    case FIND_CHARS:
      commandResult(aci_gatt_disc_all_charac_of_serv(connection, serviceStart, serviceEnd), CHARS_PENDING);
      break;
    case FIND_CCCD:
      commandResult(aci_gatt_disc_all_charac_descriptors(connection, notifyHandle, descriptorEnd), CCCD_PENDING);
      break;
    case SUBSCRIBE: {
      uint8_t enabled[] = {1, 0};
      commandResult(aci_gatt_write_charac_descriptor(connection, cccdHandle, 2, enabled), SUBSCRIBE_PENDING);
      break;
    }
    default: break;
  }
  static uint32_t lastDraw = 0;
  if (screenDirty && millis() - lastDraw >= 100) {
    // Display and BLE share SPI; prevent the BLE ISR interrupting a redraw.
    NVIC_DisableIRQ(EIC_IRQn);
    drawScreen();
    NVIC_EnableIRQ(EIC_IRQn);
    lastDraw = millis();
    screenDirty = false;
  }
}

// Callbacks only record events; the main loop issues the next BLE command.
void HCI_Event_CB(void *packet) {
  hci_uart_pckt *uart = (hci_uart_pckt *)packet;
  if (uart->type != HCI_EVENT_PKT) return;
  hci_event_pckt *event = (hci_event_pckt *)uart->data;
  if (event->evt == EVT_DISCONN_COMPLETE) {
    haveSample = false;
    changeState(RESTART);
    return;
  }
  if (event->evt == EVT_LE_META_EVENT) {
    evt_le_meta_event *meta = (evt_le_meta_event *)event->data;
    if (meta->subevent == EVT_LE_ADVERTISING_REPORT && state == SCANNING) {
      // LE report: count, then [event type, address type, address(6), length,
      // advertising data, RSSI] for each device. Bound reads by packet size.
      const uint8_t *report = meta->data;
      unsigned remaining = event->plen >= 2 ? event->plen - 2 : 0;
      uint8_t reports = meta->data[0];
      ++report;
      for (unsigned i = 0; i < reports && remaining >= 10; ++i) {
        unsigned length = report[8];
        if (length > 31 || remaining < length + 10) break;
        recordDevice(report[1], report + 2, report[0], report + 9,
                     length, (int8_t)report[9 + length]);
        report += length + 10;
        remaining -= length + 10;
      }
    }
    if (meta->subevent == EVT_LE_CONN_COMPLETE && state == CONNECTING) {
      evt_le_connection_complete *cc = (evt_le_connection_complete *)meta->data;
      if (cc->status) changeState(RESTART);
      else { connection = cc->handle; changeState(FIND_SERVICE); }
    }
    return;
  }
  if (event->evt != EVT_VENDOR) return;
  evt_blue_aci *blue = (evt_blue_aci *)event->data;
  switch (blue->ecode) {
    case EVT_BLUE_GAP_DEVICE_FOUND: {
      evt_gap_device_found *found = (evt_gap_device_found *)blue->data;
      recordDevice(found->bdaddr_type, found->bdaddr, found->evt_type,
                   found->data_RSSI, found->data_length,
                   (int8_t)found->data_RSSI[found->data_length]);
      break;
    }
    case EVT_BLUE_GAP_PROCEDURE_COMPLETE: {
      evt_gap_procedure_complete *done = (evt_gap_procedure_complete *)blue->data;
      if (done->procedure_code == GAP_GENERAL_DISCOVERY_PROC) {
        if (state == WAIT_SCAN_STOP || state == STOP_SCAN || state == SCANNING)
          changeState(selectionPending ? CONNECT : PICK_DEVICE);
      } else if (state == CONNECTING && done->status) changeState(RESTART);
      break;
    }
    case EVT_BLUE_ATT_FIND_BY_TYPE_VAL_RESP: {
      evt_att_find_by_type_val_resp *response = (evt_att_find_by_type_val_resp *)blue->data;
      if (state == SERVICE_PENDING && response->conn_handle == connection && response->event_data_length >= 4) {
        serviceStart = read16(response->handles_info_list);
        serviceEnd = read16(response->handles_info_list + 2);
      }
      break;
    }
    case EVT_BLUE_ATT_READ_BY_TYPE_RESP: {
      evt_att_read_by_type_resp *response = (evt_att_read_by_type_resp *)blue->data;
      if (state != CHARS_PENDING || response->conn_handle != connection) break;
      unsigned size = response->handle_value_pair_length;
      if (size < 7 || response->event_data_length < 1) break;
      for (unsigned i = 0; i + size <= unsigned(response->event_data_length - 1); i += size) {
        const uint8_t *p = response->handle_value_pair + i;
        uint16_t declaration = read16(p);
        if (notifyHandle && declaration > notifyHandle && declaration - 1 < descriptorEnd)
          descriptorEnd = declaration - 1;
        if (size == 21 && (p[2] & CHAR_PROP_NOTIFY) && memcmp(p + 5, notifyUUID, 16) == 0) {
          notifyHandle = read16(p + 3);
          descriptorEnd = serviceEnd;
        }
      }
      break;
    }
    case EVT_BLUE_ATT_FIND_INFORMATION_RESP: {
      evt_att_find_information_resp *response = (evt_att_find_information_resp *)blue->data;
      if (state != CCCD_PENDING || response->conn_handle != connection ||
          response->format != 1 || response->event_data_length < 1) break;
      for (unsigned i = 0; i + 4 <= unsigned(response->event_data_length - 1); i += 4) {
        const uint8_t *p = response->handle_uuid_pair + i;
        if (read16(p + 2) == 0x2902 && read16(p) > notifyHandle) cccdHandle = read16(p);
      }
      break;
    }
    case EVT_BLUE_GATT_PROCEDURE_COMPLETE: {
      evt_gatt_procedure_complete *done = (evt_gatt_procedure_complete *)blue->data;
      if (done->conn_handle != connection) break;
      // Discovery may end with ATT Attribute Not Found (0x0A) after returning matches.
      bool discoveryOK = done->error_code == 0 || done->error_code == 0x0A;
      if (state == SERVICE_PENDING) changeState(discoveryOK && serviceStart ? FIND_CHARS : RESTART);
      else if (state == CHARS_PENDING) changeState(discoveryOK && notifyHandle && descriptorEnd > notifyHandle ? FIND_CCCD : RESTART);
      else if (state == CCCD_PENDING) changeState(discoveryOK && cccdHandle ? SUBSCRIBE : RESTART);
      else if (state == SUBSCRIBE_PENDING) changeState(done->error_code == 0 ? RECEIVING : RESTART);
      break;
    }
    case EVT_BLUE_GATT_NOTIFICATION: {
      evt_gatt_attr_notification *notification = (evt_gatt_attr_notification *)blue->data;
      if ((state == RECEIVING || state == SUBSCRIBE_PENDING) && notification->conn_handle == connection &&
          notification->attr_handle == notifyHandle && notification->event_data_length >= 2)
        receiveBytes(notification->attr_value, notification->event_data_length - 2);
      break;
    }
    case EVT_BLUE_GATT_PROCEDURE_TIMEOUT:
      changeState(RESTART);
      break;
  }
}
