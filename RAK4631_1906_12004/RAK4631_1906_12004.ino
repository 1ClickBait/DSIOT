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
#include <Wire.h>
#include "ADC121C021.h"     // Click to install library: http://librarymanager/All#MQx
#include <U8g2lib.h>       // Click to install library: http://librarymanager/All#u8g2
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

Adafruit_BME680 bme;
// Might need adjustments
#define SEALEVELPRESSURE_HPA (1010.0)
float sensorPPM;
float PPMpercentage;
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

#define EN_PIN        WB_IO6  //Logic high enables the device. Logic low disables the device
#define ALERT_PIN     WB_IO5  //a high indicates that the respective limit has been violated.
#define MQ2_ADDRESS   0x51    //the device i2c address

#define      RatioMQ2CleanAir     (1.0)   //RS / R0 = 1.0 ppm 
#define      MQ2_RL               (10.0)  //the board RL = 10KΩ  can adjust

ADC121C021 MQ2;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

uint16_t result;
char displayData[32];    //OLED dispaly datas

//Function declaration
void firstDisplay();
void firstDisplay()
{  
  u8g2.clearDisplay();
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font  
  memset(displayData, 0, sizeof(displayData));
  sprintf(displayData, "RAK12004 Test");
  u8g2.drawStr(3, 15, displayData);
  u8g2.sendBuffer();

  sprintf(displayData, "R0:%3.3f", MQ2.getR0());
  u8g2.drawStr(3, 30, displayData);      
  u8g2.sendBuffer(); 

  float voltage = MQ2.getSensorVoltage(); 
  sprintf(displayData, "voltage:%3.3f",voltage);
  u8g2.drawStr(3, 45, displayData);      
  u8g2.sendBuffer(); 
}
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
u8g2.begin();
  u8g2.clearDisplay();
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font  
  memset(displayData, 0, sizeof(displayData));
  sprintf(displayData, "RAK12004 Test");
  u8g2.drawStr(3, 15, displayData);
  sprintf(displayData, "MQ2 checking...");
  u8g2.drawStr(3, 45, displayData);
  u8g2.sendBuffer();


 //********ADC121C021 ADC convert init ********************************
   while(!(MQ2.begin(MQ2_ADDRESS,Wire)))
   {
    Serial.println("please check device!!!");
    delay(200);
   }    
  Serial.println("RAK12004 test Example");
 
  //**************init MQ2 *****************************************************
  MQ2.setRL(MQ2_RL);
  /*
   *detect Propane gas if to detect other gas  need to reset A and B value，it depend on MQ sensor datasheet
   */
  MQ2.setA(-0.890);   //A -> Slope, -0.685  
  MQ2.setB(1.125);    //B -> Intersect with X - Axis  1.019
 //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(0); //PPM =  pow(10, (log10(ratio)-B)/A)
 
  float calcR0 = 0;
  for(int i = 1; i<=100; i ++)
  {  
    calcR0 += MQ2.calibrateR0(RatioMQ2CleanAir);    
  }
  MQ2.setR0(calcR0/100);
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}


  float r0 = MQ2.getR0();
  Serial.printf("R0 Value is:%3.2f\r\n",r0);  
  firstDisplay();    
  delay(3000);


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
   Serial.println("Getting Conversion Readings from ADC121C021");
  Serial.println(" ");  
  sensorPPM = MQ2.readSensor(); 
  Serial.printf("sensor PPM Value is: %3.2f\r\n",sensorPPM);   
  PPMpercentage = sensorPPM/10000;
  Serial.printf("PPM percentage Value is:%3.2f%%\r\n",PPMpercentage);   
  Serial.println(" ");
  Serial.println("        ***************************        ");
  Serial.println(" ");  
     
  u8g2.clearDisplay();
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font  
  memset(displayData, 0, sizeof(displayData));
  sprintf(displayData, "RAK12004 Test");
  u8g2.drawStr(3, 15, displayData);
 
  sprintf(displayData, "Propane:");
  u8g2.drawStr(3, 30, displayData);      

  sprintf(displayData, "%3.2f PPM",sensorPPM);
  u8g2.drawStr(3, 45, displayData);  

  sprintf(displayData, "%3.2f %%",PPMpercentage);
  u8g2.drawStr(3, 60, displayData);      
  u8g2.sendBuffer();  
           
  delay(1000); 
  if (! bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
  }
  bme680_get();
  delay(5000);
			// Send the data package
			if (sendLoRaFrame(bme.temperature,bme.humidity,bme.pressure,bme.gas_resistance, sensorPPM, PPMpercentage ))
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
