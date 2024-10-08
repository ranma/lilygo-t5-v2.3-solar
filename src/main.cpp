// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <GxEPD2_BW.h>
#include <Fonts/FreeSans24pt7b.h>

#define MAX_DISPLAY_BUFFER_SIZE 65536ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_213_B73
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(/*CS=5*/ 5, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

SPIClass hspi(HSPI);

void setup()
{
	printf("Hello world!\n");
#if 0
	hspi.begin(/*SCK*/18, /*MISO*/-1, /*MOSI*/23, /*SS*/-1);
	display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
#else
	SPI.begin(/*SCK*/18, /*MISO*/-1, /*MOSI*/23, /*SS*/-1);
#endif
	display.init(115200);
	display.setRotation(1);
	display.setFont(&FreeSans24pt7b);
	display.setFullWindow();
	display.firstPage();
	display.fillScreen(GxEPD_WHITE);
	display.setTextColor(GxEPD_BLACK);
	display.setCursor(0, 32);
	display.print("Hello world!");
	display.display();
	display.hibernate();
}

void loop()
{
	static int i = 0;
	printf("Loop%d\n", i++);
	delay(1000);
}
