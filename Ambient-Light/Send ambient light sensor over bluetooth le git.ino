#define BLE_DEBUG 1

#include <Wire.h>
#include "SPI.h"
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"

//############################################## - Ambient Light
#define TSL2572_I2CADDR     0x39

#define GAIN_1X 0
#define GAIN_8X 1
#define GAIN_16X 2
#define GAIN_120X 3

//only use this with 1x and 8x gain settings
#define GAIN_DIVIDE_6 true 

int gain_val = 0;
//##############################################

//############################################## - Bluetooth LE
String BlueText;
uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
unsigned long time = 0;
unsigned long last_time = 0;
boolean act = false;
//when using this project in the Arduino IDE, delete the following include and rename UART.h to UART.ino
#include "UART.h"

//##############################################

//############################################## - Changes to BLE library
#define CONNECTED ACI_EVT_CONNECTED
#define DISCONNECTED ACI_EVT_DISCONNECTED
uint8_t connection_status = 0;
bool connected = false;
//##############################################


void setup(void)
{
    Wire.begin();
	Serial.begin(115200);
	Serial.println("test");
	BLEsetup();
	TSL2572nit(GAIN_1X);
}

void loop() {
	aci_loop();//Process any ACI commands or events
	connection_status = getStatus();
	connected = connection_status == CONNECTED;
  		
	time = millis();
	if(time - last_time > 1000) {
		
		last_time = time;
		act = true;
	}

	if(ble_rx_buffer_len){
		Serial.println(ble_rx_buffer_len);
		Serial.println((char*)ble_rx_buffer);
		char* textMessage[] = {(char*)ble_rx_buffer};
		BlueText = textMessage[0];
		
		uint8_t sendBuffer[20]="Test";
		uint8_t length=5;
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		ble_rx_buffer_len=0;
	}

	if(act && connected) { //I eventually figured out how to actually check it I'm connected. What a life.
		float AmbientLightLux = Tsl2572ReadAmbientLight();
		
		uint8_t sendBuffer[20]="Test";
		//char csc[20] = (char *) sendBuffer;
		dtostrf(AmbientLightLux, 7, 2, (char *) sendBuffer);
		uint8_t length=7;
		lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, sendBuffer, length);
		ble_rx_buffer_len=0;
		act = false;
	}
	//BlueText is a global string variable that you can use to check for input with
	//and pass to other functions
	if (BlueText == "anything"){
	//do something
	}
}

void TSL2572nit(uint8_t gain)
{
  Tsl2572RegisterWrite( 0x0F, gain );//set gain
  Tsl2572RegisterWrite( 0x01, 0xED );//51.87 ms
  Tsl2572RegisterWrite( 0x00, 0x03 );//turn on
  if(GAIN_DIVIDE_6)
    Tsl2572RegisterWrite( 0x0D, 0x04 );//scale gain by 0.16
  if(gain==GAIN_1X)gain_val=1;
  else if(gain==GAIN_8X)gain_val=8;
  else if(gain==GAIN_16X)gain_val=16;
  else if(gain==GAIN_120X)gain_val=120;
}


void Tsl2572RegisterWrite( byte regAddr, byte regData )
{
  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0x80 | regAddr); 
  Wire.write(regData);
  Wire.endTransmission(); 
}


float Tsl2572ReadAmbientLight()
{     
  uint8_t data[4]; 
  int c0,c1;
  float lux1,lux2,cpl;

  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0xA0 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom(TSL2572_I2CADDR,4);
  for(uint8_t i=0;i<4;i++)
    data[i] = Wire.read();
     
  c0 = data[1]<<8 | data[0];
  c1 = data[3]<<8 | data[2];
  
  //see TSL2572 datasheet
  cpl = 51.87 * (float)gain_val / 60.0;
  if(GAIN_DIVIDE_6) cpl/=6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}
