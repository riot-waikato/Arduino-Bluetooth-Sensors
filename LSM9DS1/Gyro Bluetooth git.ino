#define BLE_DEBUG 1

#include "SPI.h"
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6A // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

String BlueText;
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
char data[21] = "asdfghjklasdfghjkl;'";
char data2[7] = "sdfghj";
//when using this project in the Arduino IDE, delete the following include and rename UART.h to UART.ino
#include "UART.h"

//############################################## - Changes to BLE library
#define CONNECTED ACI_EVT_CONNECTED
#define DISCONNECTED ACI_EVT_DISCONNECTED
uint8_t connection_status = 0;
bool connected = false;
//##############################################

//##############################################
#define SYNC 1
#define IDLE 0
uint8_t state = IDLE;
//##############################################
void setup(void)
{
	Serial.begin(9600);
	BLEsetup();

	Serial.begin(115200);

	// Before initializing the IMU, there are a few settings
	// we may need to adjust. Use the settings struct to set
	// the device's communication mode and addresses:
	imu.settings.device.commInterface = IMU_MODE_I2C;
	imu.settings.device.mAddress = LSM9DS1_M;
	imu.settings.device.agAddress = LSM9DS1_AG;
	// The above lines will only take effect AFTER calling
	// imu.begin(), which verifies communication with the IMU
	// and turns it on.
	if (!imu.begin())
	{
		// Serial.println("Failed to communicate with LSM9DS1.");
		// Serial.println("Double-check wiring.");
		// Serial.println("Default settings in this sketch will " \
		"work for an out of the box LSM9DS1 " \
		"Breakout, but may need to be modified " \
		"if the board jumpers are.");
		while (1)
		;
	}

}

unsigned long time = 0;
unsigned long last_time = 0;

void loop()
{
	aci_loop();//Process any ACI commands or events
	connection_status = getStatus();
	connected = connection_status == CONNECTED;

	if(ble_rx_buffer_len)
	{
		Serial.println(ble_rx_buffer_len);
		Serial.println((char*)ble_rx_buffer);
		char* textMessage[] = {(char*)ble_rx_buffer};
		BlueText = textMessage[0];

		uint8_t sendBuffer[20] = "Test";
		uint8_t length = 5;
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		ble_rx_buffer_len = 0;
	}

	//BlueText is a global string variable that you can use to check for input with
	//and pass to other functions
	if (BlueText == "anything")
	{
		//do something
	}

	time = millis();

	if((time - last_time) > 500 && connected && state == IDLE)
	{
		last_time = time;

		uint8_t* sendBuffer = (uint8_t *)printGyro();
		uint8_t length = 20;

		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		sendBuffer = (uint8_t *)printAccel();
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		sendBuffer = (uint8_t *)printMag();
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);

		ble_rx_buffer_len = 0;
		state = SYNC;
	}
	else if ((time - last_time) > 250 && connected && state == SYNC)
	{
		uint8_t* sendBuffer = (uint8_t *)"s|y|n|c";
		uint8_t length = 7;
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		
		ble_rx_buffer_len = 0;
		state = IDLE;
	}
}

char* printSet(float a, float b, float c)
{
	static int width = 6;
	static int offset = 1;

	dtostrf(a, width, offset, data2);
	copy(0, data, data2, 6);

	dtostrf(b, width, offset, data2);
	copy(7, data, data2, 6);

	dtostrf(c, width, offset, data2);
	copy(14, data, data2, 6);

	data[6]  = '|';
	data[13] = '|';

	return data;
}

char* printMag()
{
	imu.readMag();
	return printSet(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
}

char* printAccel()
{
	imu.readAccel();
	return printSet(imu.calcAccel(imu.ax)*9.8f, imu.calcAccel(imu.ay)*9.8f, imu.calcAccel(imu.az)*9.8f);
}

char* printGyro()
{
	imu.readGyro();
	return printSet(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
}

//todo: read Accel, readMag, that other thing
void copy(int offset, char* output, char* input, int length)
{
	for(int i = 0; i < length; i++)
	{
		output[i + offset] = input[i];
	}
	return;
}