/*
SSD1306 - Screen module
Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "SSD1306Wire.h"
#include "OLEDDisplay.h"
#include "images.h"
#include "fonts.h"
#include "config.h"

SSD1306Wire * display;
uint8_t _screen_line = SCREEN_HEADER_HEIGHT - 1;

void _screen_header() {
}

void screen_show_logo() {
    if(!display) return;

    uint8_t x = (display->getWidth() - HELIUM_IMAGE_WIDTH) / 2;
    uint8_t y = SCREEN_HEADER_HEIGHT + (display->getHeight() - SCREEN_HEADER_HEIGHT - HELIUM_IMAGE_HEIGHT) / 2 + 1;
    display->drawXbm(x, y, HELIUM_IMAGE_WIDTH, HELIUM_IMAGE_HEIGHT, HELIUM_IMAGE);
}

void screen_off() {
    if(!display) return;

    display->displayOff();
}

void screen_on() {
    if (!display) return;

    display->displayOn();
}

void screen_clear() {
    if(!display) return;

    display->clear();
}

void screen_print(const char * text, uint8_t x, uint8_t y, uint8_t alignment) {
   // DEBUG_MSG(text);

    if(!display) return;

    display->setTextAlignment((OLEDDISPLAY_TEXT_ALIGNMENT) alignment);
    display->drawString(x, y, text);
}

void screen_print(const char * text, uint8_t x, uint8_t y) {
    screen_print(text, x, y, TEXT_ALIGN_LEFT);
}

void screen_loop() {
    if (!display) return;
    
    display->clear();
    _screen_header();
    display->drawLogBuffer(0, SCREEN_HEADER_HEIGHT);
    display->display();
}

void screen_print(const char * text) {
    //Serial.printf("Screen: %s\n", text);
    if(!display) return;

    display->print(text);
    if (_screen_line + 8 > display->getHeight()) {
        // scroll
    }
    _screen_line += 8;
    screen_loop();
}

void screen_update() {
    if (display) display->display();
}

void screen_setup() {
    // Display instance
    display = new SSD1306Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
    display->init();
    display->flipScreenVertically();
    display->setFont(Custom_ArialMT_Plain_10);

    // Scroll buffer
    display->setLogBuffer(5, 30);
}

