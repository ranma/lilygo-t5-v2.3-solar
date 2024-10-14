// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cmath>
#include <GxEPD2_BW.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <ESP32AnalogRead.h>
#include <Fonts/FreeSans18pt7b.h>
#include <esp_adc_cal.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <hal/sar_ctrl_ll.h>

extern "C"{
#include <lwip/pbuf.h>
#include <lwip/opt.h>
#include <lwip/tcp.h>
#include <lwip/tcpip.h>
#include <lwip/inet.h>
#include <lwip/netif.h>
#include <lwip/dhcp.h>
}

#if CONFIG_ESP32_ULP_COPROC_RESERVE_MEM != 512
#error Unexpected reserved mem value for ULP
#endif

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

static float intBatV = NAN;
static bool pixbufReady = false;
static bool dataReady = false;
static bool goToSleep = false;

constexpr int batPin = 35; // ADC1 input 7


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

static char mydata[512];
static uint8_t imgbuf[8192];
static int imgbufptr = 0;
static int imgbuflen = sizeof(imgbuf);
static int pixbufofs = 0;

static void tcp_error(void *arg, int8_t err) {
	ets_printf("[%ld] +E: %p %d\n", millis(), arg, err);
}

static int8_t tcp_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *pb, int8_t err) {
	ets_printf("[%ld] +R: %p %p %d\n", millis(), arg, pcb, err);
	if (pb != NULL) {
		ets_printf("rx %d bytes\n", pb->tot_len);
		memcpy(&imgbuf[imgbufptr], pb->payload, pb->tot_len);
		if (imgbufptr == 0 && imgbuf[0] == 'B' && imgbuf[1] == 'M') {
			imgbuflen = imgbuf[2]
				| (imgbuf[3] << 8)
				| (imgbuf[4] << 16)
				| (imgbuf[5] << 24);
			pixbufofs = imgbuf[10]
				| (imgbuf[11] << 8)
				| (imgbuf[12] << 16)
				| (imgbuf[13] << 24);
			ets_printf("bmp size: %d (pixbuf @%d)\n", imgbuflen, pixbufofs);
		}
		imgbufptr += pb->tot_len;
		tcp_recved(pcb, pb->tot_len);
		pbuf_free(pb);
		if (imgbufptr >= imgbuflen) {
			pixbufReady = true;
			tcp_close(pcb);
		}
	} else {
		ets_printf("(connection closed)\n");
		tcp_close(pcb);
	}
	return ERR_OK;
}

static int8_t tcp_sent(void *arg, struct tcp_pcb *pcb, uint16_t len) {
	ets_printf("[%ld] +S: %p %p %d\n", millis(), arg, pcb, len);
	return ERR_OK;
}

static int8_t tcp_poll(void *arg, struct tcp_pcb *pcb) {
	ets_printf("[%ld] +P: %p %p\n", millis(), arg, pcb);
	return ERR_OK;
}

static int8_t tcp_connected(void *arg, struct tcp_pcb *pcb, int8_t err) {
	ets_printf("[%ld] +C: %p %p %d\n", millis(), arg, pcb, err);
	if (err != ERR_OK) {
		ets_printf("connection error: %d\n", err);
		return ERR_OK;
	}

	err = tcp_write(pcb, mydata, strlen(mydata), TCP_WRITE_FLAG_COPY);
	if (err != ERR_OK) {
		ets_printf("tcp_write: %d\n", err);
	}
	err = tcp_output(pcb);
	if (err != ERR_OK) {
		ets_printf("tcp_output: %d\n", err);
	}
	return ERR_OK;
}

static void tcp_do_connect(void *ctx)
{
	ip_addr_t addr;
	addr.type = IPADDR_TYPE_V4;
	addr.u_addr.ip4.addr = IPAddress(192, 168, 8, 240);
	constexpr int port = 4341;

	tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
	if (!pcb) {
		ets_printf("pcb == NULL\n");
		return;
	}

	tcp_arg(pcb, ctx);
	tcp_nagle_disable(pcb);
	tcp_err(pcb, &tcp_error);
	tcp_recv(pcb, &tcp_recv);
	tcp_sent(pcb, &tcp_sent);
	tcp_poll(pcb, &tcp_poll, 16); /* Every 8s or so */
	err_t err = tcp_connect(pcb, &addr, port, tcp_connected);
	if (err != ERR_OK) {
		ets_printf("tcp_connect: %d\n");
	}
}

static void tcp_connect()
{
	String rssi(WiFi.RSSI());
	String batt(intBatV);
	snprintf(mydata, sizeof(mydata), "rssi=%s\nvbat=%s\n", rssi.c_str(), batt.c_str());

	tcpip_try_callback(tcp_do_connect, nullptr);
}

static void wifi_event_cb(arduino_event_id_t event, arduino_event_info_t info)
{
	uint8_t *bssid;
	switch (event) {
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

		tcp_connect();
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

esp_adc_cal_characteristics_t characteristics;

// The first 512 bytes of RTC_SLOW_MEM are reserved for ULP use, see platformio sdkconfig.h
// Use the first half to store ADC values.
constexpr size_t ulp_data_addr = 4;
constexpr size_t ulp_data_size = CONFIG_ESP32_ULP_COPROC_RESERVE_MEM / 4 / 2;
constexpr size_t ulp_load_addr = ulp_data_addr + ulp_data_size;

static void dump_adc_data()
{
	for (int i = 0; i < 8; i++) {
		int raw = RTC_SLOW_MEM[ulp_data_addr + i] & 0xffff;
		int mV = esp_adc_cal_raw_to_voltage(raw, &characteristics);
		printf("ulp_adc_data[%d] = %d (%d/%x)\n", i, mV, raw, raw);
	}
	for (int i = ulp_data_size - 8; i < ulp_data_size; i++) {
		int raw = RTC_SLOW_MEM[ulp_data_addr + i] & 0xffff;
		int mV = esp_adc_cal_raw_to_voltage(raw, &characteristics);
		printf("ulp_adc_data[%d] = %d (%d/%x)\n", i, mV, raw, raw);
	}
}

static void ulp_init()
{
	const ulp_insn_t program[] = {
		I_MOVI(R0, 0),          // R0 <- 0
		M_LABEL(1),             // label_1
		I_ADC(R3, 0, 7),        // Read ADC from ADC1 input 7
		I_MOVI(R2, ulp_data_addr),  // R1 <- data_addr
		I_ADDR(R2, R2, R0),     // R2 <- R2 + R0
		I_ST(R3, R2, 0),        // R3 -> RTC_SLOW_MEM[R2 + 0]
		I_ADDI(R0, R0, 1),      // R0 <- R0 + 1
		M_BL(1, ulp_data_size),     // if (R0 < data_size) goto label_1
		I_HALT(),
	};
	size_t size = sizeof(program)/sizeof(ulp_insn_t);

	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT, 1100/*VREF*/, &characteristics);

	esp_err_t err = ulp_process_macros_and_load(ulp_load_addr, program, &size);
	printf("ulp program size=%d err=%04x\n", (int)size, err);
	printf("load_addr=%04x\n", (int)ulp_load_addr);
	printf("data_addr=%04x\n", (int)ulp_data_addr);
	for (int i = 0; i < size; i++) {
		printf("program[%d]=%08x\n", i, RTC_SLOW_MEM[ulp_load_addr + i]);
	}
	dump_adc_data();

	rtc_gpio_init(GPIO_NUM_35);
	rtc_gpio_set_direction(GPIO_NUM_35, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_set_direction_in_sleep(GPIO_NUM_35, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_hold_en(GPIO_NUM_35);
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_ulp_enable();
	// wake-up ULP from halt once per second
	ulp_set_wakeup_period(0, 1000 * 1000);
	ulp_run(ulp_load_addr);
}

void setup()
{
	bool full_init = survives_sleep == 0;
	float temp = temperatureRead();
	intBatV = batVolt();

	printf("Hello world (wake=%d) bat=%.2fV t=%.1f!\n", survives_sleep++, intBatV, temp);
	ulp_init();
	fs_init(full_init);
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
	dump_adc_data();
}

void loop()
{
	if (pixbufReady) {
		printf("[%ld] update display\n", millis());
		delay(1);
		WiFi.disconnect(true);
		WiFi.mode(WIFI_OFF);
		display.firstPage();
		display.drawBitmap(0, 0, &imgbuf[pixbufofs], 250, 122, GxEPD_WHITE, GxEPD_BLACK);
		display.display(/*partial*/ true);
		goToSleep = true;
	}
	if (goToSleep || millis() >= 20000) {
		long sleepTime = 30;
		rtc_config.dhcp_renew_remain -= sleepTime;

		display.hibernate();

		// Make sure epaper CS# is kept high during sleep.
		pinMode(16, INPUT_PULLUP);
		printf("[%ld] esp_deep_sleep_start()\n", millis());
#if 0
		gpio_deep_sleep_hold_en();
		gpio_hold_dis((gpio_num_t)batPin);
#endif
#if 0
		// Make sure SDIO power domain (SPI flash) is disabled during sleep.
		// Doesn't do anything here, as it's also directly connected to +3V3...
		esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF)
#endif

		long sleepMillis = sleepTime * 1000 - millis();
		printf("[%ld] going to sleep for %ldms\n", millis(), sleepMillis);
		// esp_deep_sleep_disable_rom_logging();
		// Need to re-init ADC after wifi stop? https://github.com/espressif/esp-idf/issues/10471
		adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
		adc1_config_width(ADC_WIDTH_BIT_12);
		adc1_ulp_enable();
		ulp_run(ulp_load_addr);
		esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
		esp_sleep_enable_ulp_wakeup();
		esp_sleep_enable_timer_wakeup(sleepMillis * 1000);
		rtc_gpio_init(GPIO_NUM_35);
		rtc_gpio_set_direction(GPIO_NUM_35, RTC_GPIO_MODE_INPUT_ONLY);
		rtc_gpio_set_direction_in_sleep(GPIO_NUM_35, RTC_GPIO_MODE_INPUT_ONLY);
		rtc_gpio_hold_en(GPIO_NUM_35);
		esp_deep_sleep_start();

		// Deep sleep power draw measured ~394uA after adding a pull-up
		// to the SPI flash CS# (3.6mA before).
	}
	delay(1);
}

extern "C" {

static RTC_RODATA_ATTR const char kWakeStub[] = "Wake stub!\n";

// Note: RTC IRAM/FAST memory is only accessible to the "PRO" core
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
	ets_printf(kWakeStub);
	esp_default_wake_deep_sleep();
	// Add additional functionality here
}

}

