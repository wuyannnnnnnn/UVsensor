
#ifdef ENABLE_MT15

#ifndef ENABLE_DEBUG_MT15
#undef ENABLE_DEBUG
#endif


#include <Arduino.h>
#include <SoftwareSerial.h>

#include "config.h"
#include "debug.h"
#include "mt15.h"

MT15Class MT15;

#define DEFAULT_SERIAL_SPEED 9600

SoftwareSerial ScannerSerial(PIN_SCANNER_RX, PIN_SCANNER_TX); // RX, TX

String MT15Class::sendCommand(String command)
{
  // Send something to wake the module
  ScannerSerial.write('\r');
  ScannerSerial.print(command);
  ScannerSerial.readStringUntil('{');
  return ScannerSerial.readStringUntil('}');
}

void MT15Class::begin(int baud)
{
  DBUG("Configure MT15 ");
  DBUGVAR(baud);

#ifdef TRIGGER_MODULE
  pinMode(TRIGGER_MODULE, OUTPUT);
  digitalWrite(TRIGGER_MODULE, HIGH);
#endif

#ifdef PIN_SCANNER_POWER
  pinMode(PIN_SCANNER_POWER, OUTPUT);
  digitalWrite(PIN_SCANNER_POWER, LOW);
  delay(5);
#endif

  ScannerSerial.begin(baud);

/*
  // Read the version 
  String version = sendCommand("{M VERR}");
  DBUGVAR(version);
  if(version == "" && baud != DEFAULT_SERIAL_SPEED) 
  {
    DBUGLN("Configuring serial speed");
    delay(50);
    ScannerSerial.end();

    ScannerSerial.begin(DEFAULT_SERIAL_SPEED);
    switch(baud)
    {
      case 9600:
        sendCommand("{MG002W6,5,2,1}");
        break;
      
      case 115200:
        sendCommand("{MG002W11,5,2,1}");
        break;
      
    }
    ScannerSerial.end();

    ScannerSerial.begin(baud);
    version = sendCommand("{M VERR}");
    DBUGVAR(version);
  }
*/

#ifdef PIN_SCANNER_POWER
  digitalWrite(PIN_SCANNER_POWER, HIGH);
#endif

}

#endif // ENABLE_MT15
