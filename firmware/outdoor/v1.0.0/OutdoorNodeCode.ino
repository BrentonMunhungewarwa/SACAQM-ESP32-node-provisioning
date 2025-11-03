#include <Arduino.h>
#include <SensirionI2cSen66.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>

// Server configuration
const char* serverHost = "sacaqm.web.cern.ch";
const char* serverPath = "/dbwrite.php";

// Fixed values for LTE fields
const char* fixedArea = "wifi";
const char* fixedOperator = "wifi";
const char* fixedCellId = "wifi";

// Sensor and error handling
SensirionI2cSen66 sensor;
static char errorMessage[64];
static int16_t error;
#define NO_ERROR 0

char deviceSerialNumber[65] = "";

// Provisioned values from NVS
Preferences prefs;
String wifiSSID;
String wifiPASS;
String nodeDesc;
String nodeType;
String nodeGPS;
String nodeFWVer;

// Connection status
enum ConnectionStatus {
    DISCONNECTED = 0,
    CONNECTING = 1,
    CONNECTED = 2,
    ERROR = 3
};
ConnectionStatus wifiStatus = DISCONNECTED;

// --------------------------------------------------
// Helper: Load provisioning data from NVS
// --------------------------------------------------
void loadProvisioningData() {
    prefs.begin("sacaqm", true); // read-only
    wifiSSID = prefs.getString("ssid", "");
    wifiPASS = prefs.getString("pass", "");
    nodeDesc = prefs.getString("desc", "");
    nodeType = prefs.getString("type", "");
    nodeGPS  = prefs.getString("gps", "");
    nodeFWVer= prefs.getString("fwver", "");
    prefs.end();

    Serial.println("Provisioned values loaded:");
    Serial.println("SSID: " + wifiSSID);
    Serial.println("PASS: " + wifiPASS);
    Serial.println("Desc: " + nodeDesc);
    Serial.println("Type: " + nodeType);
    Serial.println("GPS: " + nodeGPS);
    Serial.println("FWVer: " + nodeFWVer);
}

// --------------------------------------------------
// Get MAC address without colons
// --------------------------------------------------
void getMacAddressWithoutColons(char* macStr) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// --------------------------------------------------
// Connect to WiFi using provisioned credentials
// --------------------------------------------------
void connectToWiFi() {
    if (wifiSSID.isEmpty() || wifiPASS.isEmpty()) {
        Serial.println("No WiFi credentials found in NVS.");
        wifiStatus = ERROR;
        return;
    }

    Serial.print("Connecting to WiFi: ");
    Serial.println(wifiSSID);

    WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n WiFi connected.");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        wifiStatus = CONNECTED;
    } else {
        Serial.println("\n Failed to connect to WiFi.");
        wifiStatus = ERROR;
    }
}

// --------------------------------------------------
// Helper: Convert byte array to hex string
// --------------------------------------------------
void byteArrayToHexString(uint8_t* array, uint16_t len, char* output) {
    for (uint16_t i = 0; i < len; i++) {
        sprintf(&output[i * 2], "%02x", array[i]);
    }
    output[len * 2] = '\0';
}

// --------------------------------------------------
// Setup sensor
// --------------------------------------------------
void setupSensor() {
    Wire.begin();
    sensor.begin(Wire, SEN66_I2C_ADDR_6B);

    error = sensor.deviceReset();
    if (error != NO_ERROR) {
        Serial.print(F("Error executing deviceReset(): "));
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        return;
    }
    delay(1200);

    int8_t serialNumber[32] = {0};
    error = sensor.getSerialNumber(serialNumber, 32);
    if (error != NO_ERROR) {
        Serial.print(F("Error executing getSerialNumber(): "));
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
        return;
    }

    Serial.print(F("Sensor Serial Number: 0x"));
    byteArrayToHexString((uint8_t*)serialNumber, 32, deviceSerialNumber);
    Serial.println(deviceSerialNumber);

    error = sensor.startContinuousMeasurement();
    if (error != NO_ERROR) {
        Serial.print(F("Error executing startContinuousMeasurement(): "));
        errorToString(error, errorMessage, sizeof(errorMessage));
        Serial.println(errorMessage);
    }
}

// --------------------------------------------------
// Send data to server
// --------------------------------------------------
void sendSensorData(float pm1p0, float pm2p5, float pm4p0, float pm10p0,
                    float humidity, float temperature, float vocIndex,
                    float noxIndex, uint16_t co2) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected. Skipping send.");
        return;
    }

    Serial.print(F("WiFi RSSI: "));
    Serial.println(WiFi.RSSI());

    if (pm1p0 >= 6553.0 || co2 >= 65530) {
        Serial.println(F("Invalid sensor readings detected. Skipping this cycle."));
        return;
    }

    // Build URL with query parameters
    char url[600];
    char macWithoutColons[13];
    getMacAddressWithoutColons(macWithoutColons);

    snprintf(url, sizeof(url),
        "https://%s%s?cmd=add_sen55"
        "&sensor_id=%s"
        "&area=%s"
        "&operator=%s"
        "&cellid=%s"
        "&temp=%.2f"
        "&humi=%.2f"
        "&pm1p0=%.2f"
        "&pm2p5=%.2f"
        "&pm4p0=%.2f"
        "&pm10p0=%.2f"
        "&voc=%.2f"
        "&nox=%.2f"
        "&co2=%d",
        serverHost, serverPath,
        macWithoutColons,
        fixedArea,
        fixedOperator,
        fixedCellId,
        temperature,
        humidity,
        pm1p0,
        pm2p5,
        pm4p0,
        pm10p0,
        vocIndex,
        noxIndex,
        co2
    );

    Serial.println("Request URL: ");
    Serial.println(url);

    WiFiClientSecure client;
    HTTPClient http;

    client.setInsecure(); // Skip SSL verification (for testing)

    if (http.begin(client, url)) {
        int httpCode = http.GET();

        if (httpCode > 0) {
            Serial.printf("HTTP status code: %d\n", httpCode);
            if (httpCode == HTTP_CODE_OK) {
                String response = http.getString();
                Serial.println("Server response: " + response);
            }
        } else {
            Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    } else {
        Serial.println("Unable to connect to server");
    }
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Serial.println("\nStarting ESP32 Outdoor Node with SEN55 Sensor");
    Serial.println("================================");

    loadProvisioningData();   // 🔑 Load WiFi + metadata from NVS
    setupSensor();
    connectToWiFi();
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        float pm1p0 = 0.0, pm2p5 = 0.0, pm4p0 = 0.0, pm10p0 = 0.0;
        float humidity = 0.0, temperature = 0.0;
        float vocIndex = 0.0, noxIndex = 0.0;
        uint16_t co2 = 0;

        error = sensor.readMeasuredValues(
            pm1p0, pm2p5, pm4p0, pm10p0,
            humidity, temperature, vocIndex, noxIndex,
            co2
        );

        if (error != NO_ERROR) {
            Serial.print(F("Error reading values: "));
            errorToString(error, errorMessage, sizeof(errorMessage));
            Serial.println(errorMessage);
        }else{
        Serial.println("\n[SENSOR READINGS]");
        Serial.printf("PM1.0: %.2f  PM2.5: %.2f  PM4.0: %.2f  PM10.0: %.2f\n", pm1p0, pm2p5, pm4p0, pm10p0);
        Serial.printf("Temp: %.2f  Humi: %.2f  VOC: %.2f  NOx: %.2f  CO2: %d\n", temperature, humidity, vocIndex, noxIndex, co2);
        sendSensorData(pm1p0, pm2p5, pm4p0, pm10p0,
                       humidity, temperature, vocIndex, noxIndex, co2);
        }

        Serial.println("\nSleeping for 5 minutes...");
        unsigned long startTime = millis();
        const unsigned long interval = 300000; // 5 min

        while (millis() - startTime < interval) {
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("WiFi disconnected. Reconnecting...");
                connectToWiFi();
            }
            delay(60000);
            Serial.print(".");
        }
        Serial.println("\nWake up");
    } else {
        Serial.println(F("WiFi disconnected. Trying to reconnect..."));
        connectToWiFi();
        delay(60000);
    }
}
