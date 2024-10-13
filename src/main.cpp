// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cmath>
#include <GxEPD2_BW.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ESP32AnalogRead.h>
#include <Fonts/FreeSans18pt7b.h>
#include <espMqttClientAsync.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>

// Partial copy of components/esp_netif/lwip/esp_netif_lwip_internal.h
struct esp_netif_obj {
	uint8_t mac[NETIF_MAX_HWADDR_LEN];
	void* ip_info;
	void* ip_info_old;
	struct netif *lwip_netif;
	// don't care about remaining entries here.
};

// extract the DHCP lease time.
static uint32_t netif_get_dhcp_lease(esp_netif_t *netif) {
	if (netif != nullptr && netif->lwip_netif != nullptr) {
		struct netif *lwip_netif = netif->lwip_netif;
		struct dhcp *dhcp = netif_dhcp_data(lwip_netif);
		if (dhcp != nullptr) {
			return dhcp->t0_timeout;
		}
	}
	return 0;
}

#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_213_B73
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

SPIClass hspi(HSPI);
espMqttClientAsync mqttClient;

struct rtc_config {
	uint8_t bssid[6];
	uint8_t channel;
	uint32_t ip;
	uint32_t gateway;
	uint32_t netmask;
	uint32_t primaryDNS;
	uint32_t secondaryDNS;
	int dhcp_lease;
	int dhcp_renew_remain;
};

static RTC_DATA_ATTR int survives_sleep = 0;
static RTC_DATA_ATTR struct rtc_config rtc_config;

static float batteryCurrent = NAN;
static float batterySoC = NAN;
static float inverterPower = NAN;
static float intBatV = NAN;
static bool dataReady = false;
static bool goToSleep = false;

constexpr int batPin = 35;


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

static void fs_init(bool full_init)
{
	printf("Initialize FS... ");
	if (LittleFS.begin(false)) {
		printf("done\n");
		return;
	}
	if (full_init) {
		printf("failed... trying to format...");
		if (!LittleFS.begin(true)) {
			printf("success\n");
			return;
		}
	}
	printf("failed\n");
}

static void wifi_event_cb(arduino_event_id_t event, arduino_event_info_t info)
{
    uint8_t *bssid;
    switch (event) {
    case ARDUINO_EVENT_WIFI_READY: // 0
        printf("[%ld] WiFi ready\n", millis());
        break;
    case ARDUINO_EVENT_WIFI_STA_START: // 2
        printf("[%ld] WiFi STA start\n", millis());
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP: // 3
        printf("[%ld] WiFi STA stop\n", millis());
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: // 4
        bssid = info.wifi_sta_connected.bssid;
        printf("[%ld] WiFi STA connected (ch=%d bss=%02x%02x%02x%02x%02x%02x)\n",
		millis(), info.wifi_sta_connected.channel, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
	rtc_config.channel = info.wifi_sta_connected.channel;
	memcpy(rtc_config.bssid, info.wifi_sta_connected.bssid, sizeof(rtc_config.bssid));
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: // 5
        // Reason codes can be found here: https://github.com/espressif/esp-idf/blob/5454d37d496a8c58542eb450467471404c606501/components/esp_wifi/include/esp_wifi_types_generic.h#L79-L141
        printf("[%ld] WiFi STA disconnected: %d\n", millis(), info.wifi_sta_disconnected.reason);
        goToSleep = true;
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: // 7
        printf("[%ld] WiFi STA got ip: %s/%s gw %s rssi=%d\n", millis(),
		 WiFi.localIP().toString().c_str(),
		 WiFi.subnetMask().toString().c_str(),
		 WiFi.gatewayIP().toString().c_str(),
		 WiFi.RSSI());
	rtc_config.ip = WiFi.localIP();
	rtc_config.gateway = WiFi.gatewayIP();
	rtc_config.netmask = WiFi.subnetMask();
	rtc_config.primaryDNS = WiFi.dnsIP(0);
	rtc_config.secondaryDNS = WiFi.dnsIP(1);
	if (!rtc_config.dhcp_lease) {
		rtc_config.dhcp_lease = netif_get_dhcp_lease(info.got_ip.esp_netif);
		rtc_config.dhcp_renew_remain = rtc_config.dhcp_lease / 2;
	}

        mqttClient.connect();
        break;
    default:
        printf("[%ld] Unhandled event: %d (%s)\n", millis(), event, WiFi.eventName(event));
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
	if (rtc_config.channel) {
		uint8_t *bssid = rtc_config.bssid;
		if (rtc_config.dhcp_lease) {
			printf("Re-using IP %08x (lease=%d renew=%d)\n", rtc_config.ip, rtc_config.dhcp_lease, rtc_config.dhcp_renew_remain);
			WiFi.config(rtc_config.ip, rtc_config.gateway, rtc_config.netmask,
				rtc_config.primaryDNS, rtc_config.secondaryDNS);
			if (rtc_config.dhcp_renew_remain < 0) {
				rtc_config.dhcp_lease = 0;
			}
		}
		printf("Reconnecting to ch=%d bss=%02x%02x%02x%02x%02x%02x\n",
			rtc_config.channel, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		WiFi.begin(wifi_ssid, wifi_password, rtc_config.channel, rtc_config.bssid);
	} else {
		WiFi.begin(wifi_ssid, wifi_password, WIFI_ALL_CHANNEL_SCAN);
	}
	printf("Default txpower=%d\n", WiFi.getTxPower());
	WiFi.setTxPower(WIFI_POWER_11dBm);
}

constexpr char kMqttBatteryCurrent[] = "solar/battery/current";
constexpr char kMqttBatterySoC[] = "solar/battery/stateOfCharge";
constexpr char kMqttInverterPower[] = "solar/ac/power";

void onMqttConnect(bool sessionPresent)
{
	printf("[%ld] Connected to MQTT (%d)\n", millis(), sessionPresent);
	mqttClient.subscribe(kMqttBatteryCurrent, 0);
	mqttClient.subscribe(kMqttBatterySoC, 0);
	mqttClient.subscribe(kMqttInverterPower, 0);
	String rssi(WiFi.RSSI());
	String batt(intBatV);
	mqttClient.publish("epaper/sensor/wifi/rssi", 0, true, rssi.c_str());
	mqttClient.publish("epaper/sensor/battery_voltage/state", 0, true, batt.c_str());
}

static void dprintf(int x, int y, const GFXfont *font, const char *fmt, ...)
{
	char buf[40];
	va_list ap;
	display.setCursor(x, y);
	display.setFont(font);
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	display.print(buf);
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
	String data(payload, len);
	printf("[%ld] received topic %s: %s\n", millis(), topic, data.c_str());
	if (strcmp(topic, kMqttBatteryCurrent) == 0) {
		batteryCurrent = data.toFloat();
	} else if (strcmp(topic, kMqttBatterySoC) == 0) {
		batterySoC = data.toFloat();
	} else if (strcmp(topic, kMqttInverterPower) == 0) {
		inverterPower = data.toFloat();
	}
	if (isnan(batteryCurrent) || isnan(batterySoC) || isnan(inverterPower)) {
		return;
	}

	dataReady = true;
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
}

static void mqtt_init()
{
	mqtt_host.trim();

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onMessage(onMqttMessage);
	mqttClient.setServer(mqtt_host.c_str(), 1883);
}

static float batVolt()
{
	static ESP32AnalogRead adc;
	static bool initialized = false;
	if (!initialized) {
		adc.attach(batPin);
		initialized = true;
	}
	return 2.0 * adc.readVoltage();
}

void setup()
{
	bool full_init = survives_sleep == 0;
	float temp = temperatureRead();
	intBatV = batVolt();

	printf("Hello world (wake=%d) bat=%.2fV t=%.1f!\n", survives_sleep++, intBatV, temp);
	fs_init(full_init);
	mqtt_init();
	wifi_init();
	hspi.begin(/*SCK*/18, /*MISO*/-1, /*MOSI*/23, /*SS*/-1);
	display.epd2.selectSPI(hspi, SPISettings(10000000, MSBFIRST, SPI_MODE0));

	if (!full_init) {
		printf("[%ld] skipping full display init!\n", millis());
	}
	display.init(0/*diag disabled*/, full_init);
	display.setRotation(1);
	display.setFullWindow();
	if (full_init) {
		display.firstPage();
		display.fillScreen(GxEPD_WHITE);
		display.setTextColor(GxEPD_BLACK);
		display.setFont(&FreeSans18pt7b);
		display.setCursor(0, 32);
		display.printf("Connecting...");
		display.display();
	} else {
		// trigger power-on.
		display.refresh(true);
	}
	printf("[%ld] setup done\n", millis());
}

void loop()
{
	if (dataReady) {
		float p = (1-(4.1-intBatV)/(4.1-3.3))*100;
		p = p < 100 ? p : 100;

		printf("[%ld] update display\n", millis());
		display.firstPage();
		display.fillScreen(GxEPD_WHITE);
		display.setTextColor(GxEPD_BLACK);

		dprintf(0, 4+28, &FreeSans18pt7b, "I: %.0fW", inverterPower);
		dprintf(0, 4+56, &FreeSans18pt7b, "B: %.0f%%", batterySoC);
		dprintf(0, 4+84, &FreeSans18pt7b, "%.2fV %.1f%%", intBatV, p);

		display.display(/*partial*/ true);
		goToSleep = true;
	}
	if (goToSleep || millis() >= 20000) {
		long sleepTime = 30;
		rtc_config.dhcp_renew_remain -= sleepTime;

		long sleepMillis = sleepTime * 1000 - millis();
		printf("[%ld] going to sleep for %ldms\n", millis(), sleepMillis);
		display.hibernate();
		esp_sleep_enable_timer_wakeup(sleepMillis * 1000);
		// Make sure epaper CS# is kept high during sleep.
		pinMode(16, INPUT_PULLUP);
		printf("[%ld] esp_deep_sleep_start()\n", millis());
		gpio_deep_sleep_hold_en();
#if 0
		// Make sure SDIO power domain (SPI flash) is disabled during sleep.
		// Doesn't do anything here, as it's also directly connected to +3V3...
		esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF)
#endif
		esp_deep_sleep_start();

		// Deep sleep power draw measured ~394uA after adding a pull-up
		// to the SPI flash CS# (3.6mA before).
	}
	delay(1);
}
