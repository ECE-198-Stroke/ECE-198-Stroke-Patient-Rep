/*
 * wifi_config.h - network settings for TableSlideAccel. Edit this file, then upload.
 *
 * Put your Wi-Fi name and password between the quotes. Use a normal home, lab or phone-hotspot
 * network (2.4 GHz). eduroam and other networks with a username login will not work.
 *
 * Leave WIFI_SSID empty, or if joining fails, the ESP32 creates its own open network named
 * AP_NAME instead. Join that, then open http://192.168.4.1
 *
 * This file holds your Wi-Fi password: do not share it or post it anywhere.
 */
#pragma once

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define AP_NAME "TableSlide"
#define MDNS_NAME "tableslide"  // Also reachable as http://tableslide.local on most computers.
