/* 
 Project: SIM808 GSM GPRS GPS Bluetooth evolution board (EVB-V3.2)
 Function: Serial communication with evolution board
 
 RX is digital pin 2 of Arduino board (connect to TXD of EVB-V3.2)
 TX is digital pin 3 of Arduino board (connect to RXD of EVB-V3.2)
 */
//********************************************
#include <SoftwareSerial.h>//include library code
//********************************************
SoftwareSerial GPSmodule(2, 3);//RX, TX
//**********************************************
void setup()
{
GPSmodule.begin(9600);   
Serial.begin(9600);  
Serial.print("Please enter AT command:");  
delay(100);
}
void loop()
{
 if (Serial.available()>0)
 GPSmodule.write(Serial.read());
 if (GPSmodule.available()>0)
 Serial.write(GPSmodule.read());
}
