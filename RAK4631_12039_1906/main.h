/**
 * @file main.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Includes, definitions and global declarations for DeepSleep example
 * @version 0.1
 * @date 2020-08-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Arduino.h>
#include <SPI.h>

#include <LoRaWan-RAK4630.h>

// Comment the next line if you want DEBUG output. But the power savings are not as good then!!!!!!!
#define MAX_SAVE

/* Time the device is sleeping in milliseconds = 2 minutes * 60 seconds * 1000 milliseconds */
#define SLEEP_TIME 2 * 60 * 1000

// LoRaWan stuff
int8_t initLoRaWan(void);
bool sendLoRaFrame(
  float temperature,
  float pressure,
  float humidity,
  float gas_resistance,
  uint16_t pm10_std,
  uint16_t pm25_std,
  uint16_t pm100_std,
  uint16_t pm10_env,
  uint16_t pm25_env,
  uint16_t pm100_env,
  uint16_t part03,
  uint16_t part05,
  uint16_t part10,
  uint16_t part25,
  uint16_t part50,
  uint16_t part100
);
extern SemaphoreHandle_t loraEvent;

// Main loop stuff
void periodicWakeup(TimerHandle_t unused);
extern SemaphoreHandle_t taskEvent;
extern uint8_t rcvdLoRaData[];
extern uint8_t rcvdDataLen;
extern uint8_t eventType;
extern SoftwareTimer taskWakeupTimer;
