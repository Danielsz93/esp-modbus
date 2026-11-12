#include <NimBLEDevice.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>

// ESP32-C3 SuperMini - RS485 UART pins. Change to match your wiring.
static const int RS485_RX_PIN = 21; // ESP RX <- RS485 module TX
static const int RS485_TX_PIN = 20; // ESP TX -> RS485 module RX
static const uint32_t MODBUS_BAUD = 9600;
// A Slave ID-t minden lekérdezésnél a Windows alkalmazás küldi.
// Set to a GPIO if your RS485 board needs DE/RE direction control.
// Leave -1 for an automatic-direction RS485 module.
static const int RS485_DE_RE_PIN = -1;

static const char* BLE_NAME = "ESP32-MODBUS";
static const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // write from PC
static const char* TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notify to PC

HardwareSerial ModbusSerial(1);
ModbusMaster node;
NimBLECharacteristic* notifyChar = nullptr;
String inputLine;

void preTransmission() { if (RS485_DE_RE_PIN >= 0) digitalWrite(RS485_DE_RE_PIN, HIGH); }
void postTransmission() { if (RS485_DE_RE_PIN >= 0) digitalWrite(RS485_DE_RE_PIN, LOW); }

void sendJson(JsonDocument& doc) {
  String out; serializeJson(doc, out);
  if (notifyChar) {
    // BLE notifications are limited in size; send this compact JSON in one packet.
    notifyChar->setValue(out.c_str());
    notifyChar->notify();
  }
}

void handleCommand(const String& line) {
  JsonDocument req;
  DeserializationError err = deserializeJson(req, line);
  if (err) return;
  const char* cmd = req["cmd"] | "";
  if (strcmp(cmd, "read") != 0) return;
  JsonArray items = req["items"].as<JsonArray>();
  // One BLE notification per register keeps packets below the BLE MTU limit.
  for (JsonObject item : items) {
    JsonDocument response;
    response["type"] = "values";
    JsonArray values = response["values"].to<JsonArray>();
    int id = item["id"] | -1;
    uint16_t address = item["address"] | 0;
    int function = item["function"] | 3;
    uint8_t slave = item["slave"] | 16;
    uint8_t result = 0;
    if (slave < 1 || slave > 247) {
      JsonObject v = values.add<JsonObject>();
      v["id"] = id; v["address"] = address; v["ok"] = false; v["error"] = "invalid_slave";
      sendJson(response);
      delay(35);
      continue;
    }
    // Regiszterenként váltható Modbus-cím (Slave ID).
    node.begin(slave, ModbusSerial);
    if (function == 4) result = node.readInputRegisters(address, 1);
    else result = node.readHoldingRegisters(address, 1);
    JsonObject v = values.add<JsonObject>();
    v["id"] = id;
    v["address"] = address;
    if (result == node.ku8MBSuccess) {
      v["raw"] = node.getResponseBuffer(0);
      v["ok"] = true;
    } else {
      v["ok"] = false;
      v["error"] = result;
    }
    sendJson(response);
    delay(35);
  }
}

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    std::string value = characteristic->getValue();
    for (char c : value) {
      if (c == '\n') { handleCommand(inputLine); inputLine = ""; }
      else if (c != '\r' && inputLine.length() < 4000) inputLine += c;
    }
  }
};

void setup() {
  Serial.begin(115200);
  if (RS485_DE_RE_PIN >= 0) { pinMode(RS485_DE_RE_PIN, OUTPUT); digitalWrite(RS485_DE_RE_PIN, LOW); }
  ModbusSerial.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setMTU(185);
  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* service = server->createService(SERVICE_UUID);
  NimBLECharacteristic* rx = service->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  notifyChar = service->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  service->start();
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  // Tegyük a nevet a scan response csomagba is, hogy a Windows BLE keresője
  // akkor is felismerje, ha az első advertising csomagban nincs elég hely.
  advertising->setName(BLE_NAME);
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->enableScanResponse(true);
  advertising->start();
  Serial.println("BLE Modbus bridge ready: ESP32-MODBUS (Slave ID per request)");
}

void loop() { delay(20); }
