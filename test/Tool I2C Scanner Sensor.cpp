// #include <Wire.h>

// #define XSDN1 A1
// #define XSDN2 A2

// void scanBus(const char* name)
// {
//   Serial.print("Scanning for ");
//   Serial.println(name);

//   for (byte address = 1; address < 127; address++)
//   {
//     Wire.beginTransmission(address);

//     if (Wire.endTransmission() == 0)
//     {
//       Serial.print("Found device at 0x");
//       Serial.println(address, HEX);
//     }
//   }

//   Serial.println();
// }

// void setup()
// {
//   Serial.begin(115200);
//   delay(3000);

//   Serial.println("Start Scanner");

//   pinMode(XSDN1, OUTPUT);
//   pinMode(XSDN2, OUTPUT);

//   // beide aus
//   digitalWrite(XSDN1, LOW);
//   digitalWrite(XSDN2, LOW);
//   delay(500);

//   Wire.begin();

//   // Sensor 1
//   Serial.println("Turning ON Sensor 1");
//   digitalWrite(XSDN1, HIGH);
//   delay(500);

//   scanBus("Sensor 1");

//   // Sensor 2
//   Serial.println("Turning ON Sensor 2");
//   digitalWrite(XSDN2, HIGH);
//   delay(500);

//   scanBus("Sensor 2");

//   Serial.println("Scan finished");
// }

// void loop(){}