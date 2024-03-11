cmake_minimum_required(VERSION 3.22)
project(devices)

include(esp32_project)

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

find_srcs()
build_lib(
  SRCS ${SOURCES}
  ESP_TARGET ${ESP_TARGET}
  LIBS idf::freertos idf::spi_flash idf::esp_timer
  COMPONENTS freertos esptool_py
  )
