BOARD_TAG   = nano
BOARD_SUB   = atmega328
ARDUINO_DIR = /opt/arduino
MONITOR_PORT = /dev/ttyUSB0
ARDMK_DIR = /opt/arduino-mk

AVRDUDE      = $(ARDUINO_DIR)/hardware/tools/avr/bin/avrdude
AVRDUDE_CONF = $(ARDUINO_DIR)/hardware/tools/avr/etc/avrdude.conf

AVR_TOOLS_DIR = $(ARDUINO_DIR)/hardware/tools/avr
AVR_TOOLS_PATH = $(ARDUINO_DIR)/hardware/tools/avr/bin
ARDUINO_LIBS = SerialDXL Encoder PID ros_lib Control

CFLAGS_STD = -std=gnu11
CXXFLAGS_STD = -std=gnu++11
CXXFLAGS = -pedantic -Wall -Wextra

OPTIMIZATION_LEVEL = 3

include $(ARDMK_DIR)/Arduino.mk