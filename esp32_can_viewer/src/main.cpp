#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <driver/twai.h>

// WiFi Configuration
const char* ssid = "Tiida_CAN_Viewer";
const char* password = ""; // Open network

// CAN Configuration (TWAI)
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// Web Server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Signal Configuration Structure
struct CanSignal {
    String name;
    uint32_t id;
    uint8_t startBit;
    uint8_t length;
    bool isLittleEndian;
    float factor;
    float offset;
    String unit;
};

std::vector<CanSignal> signals;
const char* configFile = "/config.json";

// Function Declarations
void loadConfig();
void saveConfig();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len, AsyncWebSocketClient *client);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
float parseCanSignal(const uint8_t* data, const CanSignal& sig);

void setup() {
    Serial.begin(115200);

    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }

    // Load Configurations
    loadConfig();

    // Initialize WiFi AP
    WiFi.softAP(ssid, password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Initialize Web Server
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Initialize WebSocket
    ws.onEvent(onEvent);
    server.addHandler(&ws);

    server.begin();

    // Initialize CAN (TWAI) Driver
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN Driver installed");
    } else {
        Serial.println("Failed to install CAN driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("CAN Driver started");
    } else {
        Serial.println("Failed to start CAN driver");
        return;
    }
}

void loop() {
    ws.cleanupClients();

    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK) {
        // Find if this ID is in our config
        for (const auto& sig : signals) {
            if (sig.id == message.identifier) {
                if (message.data_length_code > 0) {
                    float value = parseCanSignal(message.data, sig);

                    // Send to websockets
                    JsonDocument doc;
                    doc["type"] = "data";
                    doc["name"] = sig.name;
                    doc["value"] = value;

                    String output;
                    serializeJson(doc, output);
                    ws.textAll(output);
                }
            }
        }
    }
}

// -------------------------------------------------------------
// Signal Parsing Logic
// -------------------------------------------------------------
float parseCanSignal(const uint8_t* data, const CanSignal& sig) {
    uint64_t rawValue = 0;

    if (sig.isLittleEndian) {
        // Little Endian parsing (Intel)
        int startByte = sig.startBit / 8;
        int bitInByte = sig.startBit % 8;

        int currentBit = 0;
        while (currentBit < sig.length) {
            int bitsToRead = min((int)sig.length - currentBit, 8 - bitInByte);
            uint8_t mask = (1 << bitsToRead) - 1;
            uint8_t extracted = (data[startByte] >> bitInByte) & mask;
            rawValue |= ((uint64_t)extracted << currentBit);

            currentBit += bitsToRead;
            bitInByte = 0;
            startByte++;
        }
    } else {
        // Big Endian parsing (Motorola)
        int currentByte = sig.startBit / 8;
        int currentBitInByte = sig.startBit % 8;
        int bitsRead = 0;

        while (bitsRead < sig.length) {
            int bitsToRead = min((int)sig.length - bitsRead, currentBitInByte + 1);
            uint8_t mask = (1 << bitsToRead) - 1;
            int shift = currentBitInByte - bitsToRead + 1;
            uint8_t extracted = (data[currentByte] >> shift) & mask;

            rawValue = (rawValue << bitsToRead) | extracted;

            bitsRead += bitsToRead;
            currentBitInByte = 7;
            currentByte++;
        }
    }

    // Apply factor and offset
    return (rawValue * sig.factor) + sig.offset;
}

// -------------------------------------------------------------
// Configuration & WebSocket Logic
// -------------------------------------------------------------
void loadConfig() {
    if (!LittleFS.exists(configFile)) return;

    File file = LittleFS.open(configFile, "r");
    if (!file) return;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) return;

    signals.clear();
    JsonArray arr = doc["signals"].as<JsonArray>();
    for (JsonObject obj : arr) {
        CanSignal sig;
        sig.name = obj["name"].as<String>();
        sig.id = obj["id"];
        sig.startBit = obj["startBit"];
        sig.length = obj["length"];
        sig.isLittleEndian = obj["isLittleEndian"];
        sig.factor = obj["factor"];
        sig.offset = obj["offset"];
        sig.unit = obj["unit"].as<String>();
        signals.push_back(sig);
    }
}

void saveConfig() {
    File file = LittleFS.open(configFile, "w");
    if (!file) return;

    JsonDocument doc;
    JsonArray arr = doc["signals"].to<JsonArray>();

    for (const auto& sig : signals) {
        JsonObject obj = arr.add<JsonObject>();
        obj["name"] = sig.name;
        obj["id"] = sig.id;
        obj["startBit"] = sig.startBit;
        obj["length"] = sig.length;
        obj["isLittleEndian"] = sig.isLittleEndian;
        obj["factor"] = sig.factor;
        obj["offset"] = sig.offset;
        obj["unit"] = sig.unit;
    }

    serializeJson(doc, file);
    file.close();
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len, AsyncWebSocketClient *client) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        String msg = (char*)data;

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, msg);
        if (err) return;

        String cmd = doc["cmd"].as<String>();

        if (cmd == "getConfig") {
            JsonDocument res;
            res["type"] = "config";
            JsonArray arr = res["signals"].to<JsonArray>();
            for (const auto& sig : signals) {
                JsonObject obj = arr.add<JsonObject>();
                obj["name"] = sig.name;
                obj["id"] = sig.id;
                obj["startBit"] = sig.startBit;
                obj["length"] = sig.length;
                obj["isLittleEndian"] = sig.isLittleEndian;
                obj["factor"] = sig.factor;
                obj["offset"] = sig.offset;
                obj["unit"] = sig.unit;
            }
            String output;
            serializeJson(res, output);
            client->text(output);
        }
        else if (cmd == "setConfig") {
            signals.clear();
            JsonArray arr = doc["signals"].as<JsonArray>();
            for (JsonObject obj : arr) {
                CanSignal sig;
                sig.name = obj["name"].as<String>();
                sig.id = obj["id"];
                sig.startBit = obj["startBit"];
                sig.length = obj["length"];
                sig.isLittleEndian = obj["isLittleEndian"];
                sig.factor = obj["factor"];
                sig.offset = obj["offset"];
                sig.unit = obj["unit"].as<String>();
                signals.push_back(sig);
            }
            saveConfig();
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len, client);
    }
}
