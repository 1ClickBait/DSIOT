/**
 * @file RAK4631-DeepSleep-LoRaWan.ino
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief LoRaWan deep sleep example
 * Device goes into sleep after successful OTAA/ABP network join.
 * Wake up every SLEEP_TIME seconds. Set time in main.h 
 * @version 0.1
 * @date 2020-09-05
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#include "main.h"
#include "RAK12039_PMSA003I.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

Adafruit_BME680 bme;
// Might need adjustments
#define SEALEVELPRESSURE_HPA (1010.0)

void bme680_init()
{
  Wire.begin();

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void bme680_get()
{
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.println();
}

RAK_PMSA003I PMSA003I;

/*
 * @brief WB_IO6 is connected to the SET pin.
 *        Set pin/TTL level @3.3V, high level or suspending is normal working status.
 *        while low level is sleeping mode.
 */
#define SET_PIN   WB_IO6

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/** Buffer for received LoRaWan data */
uint8_t rcvdLoRaData[256];
/** Length of received data */
uint8_t rcvdDataLen = 0;

/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused)
{
	// Switch on blue LED to show we are awake
	digitalWrite(LED_BUILTIN, HIGH);
	eventType = 1;
	// Give the semaphore, so the loop task will wake up
	xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}

/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void)
{
	// Create the LoRaWan event semaphore
	taskEvent = xSemaphoreCreateBinary();
	// Initialize semaphore
	xSemaphoreGive(taskEvent);

	// Initialize the built in LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize the connection status LED
	pinMode(LED_CONN, OUTPUT);
	digitalWrite(LED_CONN, HIGH);

   // Sensor power on.
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH); 

  pinMode(SET_PIN, OUTPUT);
  digitalWrite(SET_PIN, HIGH); 

#ifndef MAX_SAVE
	// Initialize Serial for debug output
	Serial.begin(115200);

	time_t timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		}
		else
		{
			break;
		}
	}
#endif
  bme680_init();
   Wire.begin();
  if(!PMSA003I.begin()) 
  {
    Serial.println("PMSA003I begin fail,please check connection!");
    delay(100);
    while(1);
  }
	digitalWrite(LED_BUILTIN, LOW);

#ifndef MAX_SAVE
	Serial.println("=====================================");
	Serial.println("RAK4631 LoRaWan Deep Sleep Test");
	Serial.println("=====================================");
#endif

	// Initialize LoRaWan and start join request
	int8_t loraInitResult = initLoRaWan();

#ifndef MAX_SAVE
	if (loraInitResult != 0)
	{
		switch (loraInitResult)
		{
		case -1:
			Serial.println("SX126x init failed");
			break;
		case -2:
			Serial.println("LoRaWan init failed");
			break;
		case -3:
			Serial.println("Subband init error");
			break;
		case -4:
			Serial.println("LoRa Task init error");
			break;
		default:
			Serial.println("LoRa init unknown error");
			break;
		}

		// Without working LoRa we just stop here
		while (1)
		{
			Serial.println("Nothing I can do, just loving you");
			delay(5000);
		}
	}
	Serial.println("LoRaWan init success");
#endif

	// Take the semaphore so the loop will go to sleep until an event happens
	xSemaphoreTake(taskEvent, 10);
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void)
{
   PMSA_Data_t data;
	// Switch off blue LED to show we go to sleep
	digitalWrite(LED_BUILTIN, LOW);

	// Sleep until we are woken up by an event
	if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE)
	{
		// Switch on blue LED to show we are awake
		digitalWrite(LED_BUILTIN, HIGH);
		delay(500); // Only so we can see the blue LED

		// Check the wake up reason
		switch (eventType)
		{
		case 0: // Wakeup reason is package downlink arrived
#ifndef MAX_SAVE
			Serial.println("Received package over LoRaWan");
#endif
			if (rcvdLoRaData[0] > 0x1F)
			{
#ifndef MAX_SAVE
				Serial.printf("%s\n", (char *)rcvdLoRaData);
#endif
			}
			else
			{
#ifndef MAX_SAVE
				for (int idx = 0; idx < rcvdDataLen; idx++)
				{
					Serial.printf("%X ", rcvdLoRaData[idx]);
				}
				Serial.println("");
#endif
			}

			break;
		case 1: // Wakeup reason is timer
#ifndef MAX_SAVE
			Serial.println("Timer wakeup");
#endif
			/// \todo read sensor or whatever you need to do frequently
      if (PMSA003I.readDate(&data)) 
  {
    Serial.println("PMSA003I read date success.");

    Serial.println("Standard particulate matter:");
    Serial.print("PM1.0: "); 
    Serial.print(data.pm10_standard);
    Serial.println(" [µg/𝑚3]"); 
    
    Serial.print("PM2.5: "); 
    Serial.print(data.pm25_standard);
    Serial.println(" [µg/𝑚3]"); 
    
    Serial.print("PM10 : "); 
    Serial.print(data.pm100_standard);
    Serial.println(" [µg/𝑚3]"); 

    Serial.println("Atmospheric environment:");
    Serial.print("PM1.0: "); 
    Serial.print(data.pm10_env);
    Serial.println(" [µg/𝑚3]");
     
    Serial.print("PM2.5: "); 
    Serial.print(data.pm25_env);
    Serial.println(" [µg/𝑚3]"); 
    
    Serial.print("PM10 : "); 
    Serial.print(data.pm100_env);
    Serial.println(" [µg/𝑚3]");

    Serial.println("The number of particles in 0.1L air (above diameter):");
    Serial.print("0.3um:"); 
    Serial.println(data.particles_03um);
    Serial.print("0.5um:"); 
    Serial.println(data.particles_05um);
    Serial.print("1.0um:"); 
    Serial.println(data.particles_10um);
    Serial.print("2.5um:"); 
    Serial.println(data.particles_25um);
    Serial.print("5.0um:"); 
    Serial.println(data.particles_50um);
    Serial.print("10 um:"); 
    Serial.println(data.particles_100um);
  }
  else
  {
    Serial.println("PMSA003I read failed!");
  }
  Serial.println();
  delay(1000);
   if (! bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
  }
  bme680_get();
  delay(5000);
			// Send the data package
			if (sendLoRaFrame(bme.temperature,bme.pressure,bme.humidity,bme.gas_resistance,data.pm10_standard,data.pm25_standard,data.pm100_standard,data.pm10_env,data.pm25_env,data.pm100_env,data.particles_03um,data.particles_05um,data.particles_10um,data.particles_25um,data.particles_50um,data.particles_100um))
			{
#ifndef MAX_SAVE
				Serial.println("LoRaWan package sent successfully");
#endif
			}
			else
			{
#ifndef MAX_SAVE
				Serial.println("LoRaWan package send failed");
				/// \todo maybe you need to retry here?
#endif
			}

			break;
		default:
#ifndef MAX_SAVE
			Serial.println("This should never happen ;-)");
#endif
			break;
		}
   digitalWrite(LED_BUILTIN, LOW);
		// Go back to sleep
		xSemaphoreTake(taskEvent, 10);
	}
}
