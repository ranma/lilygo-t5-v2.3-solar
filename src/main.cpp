// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <GxEPD2_BW.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <Fonts/FreeSans24pt7b.h>
#include <espMqttClientAsync.h>

#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_213_B73
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

SPIClass hspi(HSPI);
espMqttClientAsync mqttClient;

static RTC_DATA_ATTR int survives_sleep = 0;

extern const uint8_t hostname_start[] asm("_binary_cfg_hostname_start");
extern const uint8_t hostname_end[] asm("_binary_cfg_hostname_end");
extern const uint8_t wifi_ssid_start[] asm("_binary_cfg_wifi_ssid_start");
extern const uint8_t wifi_ssid_end[] asm("_binary_cfg_wifi_ssid_end");
extern const uint8_t wifi_password_start[] asm("_binary_cfg_wifi_password_start");
extern const uint8_t wifi_password_end[] asm("_binary_cfg_wifi_password_end");
extern const uint8_t mqtt_host_start[] asm("_binary_cfg_mqtt_host_start");
extern const uint8_t mqtt_host_end[] asm("_binary_cfg_mqtt_host_end");
extern const uint8_t mqtt_port_start[] asm("_binary_cfg_mqtt_port_start");
extern const uint8_t mqtt_port_end[] asm("_binary_cfg_mqtt_port_end");
static String hostname(hostname_start, hostname_end - hostname_start);
static String wifi_ssid(wifi_ssid_start, wifi_ssid_end - wifi_ssid_start);
static String wifi_password(wifi_password_start, wifi_password_end - wifi_password_start);
static String mqtt_host(mqtt_host_start, mqtt_host_end - mqtt_host_start);

static void fs_init()
{
	printf("Initialize FS... ");
	if (LittleFS.begin(false)) {
		printf("done\n");
		return;
	}
	printf("failed... trying to format...");
	if (!LittleFS.begin(true)) {
		printf("success\n");
	} else {
		printf("failed\n");
	}
}

static void wifi_event_cb(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        printf("WiFi connected\n");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        // Reason codes can be found here: https://github.com/espressif/esp-idf/blob/5454d37d496a8c58542eb450467471404c606501/components/esp_wifi/include/esp_wifi_types_generic.h#L79-L141
        printf("WiFi disconnected: %d\n", info.wifi_sta_disconnected.reason);
        WiFi.disconnect(true, false);
        WiFi.begin();
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        printf("WiFi got ip: %s\n", WiFi.localIP().toString().c_str());
        mqttClient.connect();
        break;
    default:
        printf("Unhandled event: %d\n", event);
        break;
    }
}

static void wifi_init()
{
	hostname.trim();
	wifi_ssid.trim();
	wifi_password.trim();

	WiFi.onEvent(std::bind(wifi_event_cb, std::placeholders::_1, std::placeholders::_2));
	WiFi.persistent(false);
	WiFi.mode(WIFI_MODE_NULL);
	WiFi.setHostname(hostname.c_str());
	WiFi.mode(WIFI_MODE_STA);
	WiFi.begin(wifi_ssid, wifi_password, WIFI_ALL_CHANNEL_SCAN);
}

void onMqttConnect(bool sessionPresent)
{
	printf("Connected to MQTT (%d)\n", sessionPresent);
	mqttClient.subscribe("solar/battery/stateOfCharge", 0);
	mqttClient.publish("epaper/test/value", 0, true, "42");
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
	String data(payload, len);
	printf("received topic %s: %s\n", topic, data.c_str());
}

static void mqtt_init()
{
	mqtt_host.trim();

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onMessage(onMqttMessage);
	mqttClient.setServer(mqtt_host.c_str(), 1883);
}

void setup()
{
	printf("Hello world (wake=%d) s=%d!\n", survives_sleep++, sizeof(display));
	fs_init();
	mqtt_init();
	wifi_init();
#if 0
	hspi.begin(/*SCK*/18, /*MISO*/-1, /*MOSI*/23, /*SS*/-1);
	display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
#else
	SPI.begin(/*SCK*/18, /*MISO*/-1, /*MOSI*/23, /*SS*/-1);
#endif

	bool full_init = survives_sleep < 2;
	if (!full_init) {
		printf("skipping full display init!\n");
	}
	display.init(115200, full_init);
	display.setRotation(1);
	display.setFont(&FreeSans24pt7b);
	display.setFullWindow();
	display.firstPage();
	display.fillScreen(GxEPD_WHITE);
	display.setTextColor(GxEPD_BLACK);
	display.setCursor(0, 32);
	display.printf("Hello %d!", survives_sleep);
	display.display(!full_init);
	display.hibernate();
}

void loop()
{
	static int i = 0;
	printf("Loop%d host=%s ssid=%s\n", i++, hostname.c_str(), wifi_ssid.c_str());
	delay(1000);

	if (i >= 10) {
		esp_sleep_enable_timer_wakeup(10 * 1000000);
		printf("esp_deep_sleep_start()\n");
		esp_deep_sleep_start();
	}
}
