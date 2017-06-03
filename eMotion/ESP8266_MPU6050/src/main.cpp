/*
  Original code was took from here http://fritzing.org/projects/huzzah-esp8266-mpu-6050
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>   /// https://github.com/CNMAT/OSC
#include <OSCBundle.h>  /// https://github.com/CNMAT/OSC
#include <Wire.h>
// requires I2Cdev library: https://github.com/jrowberg/i2cdevlib
#include "I2Cdev.h"
// requires MPU-6050 part of the I2Cdev lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "MPU6050.h"

///////////////////////
// MPU6050 Setup //
///////////////////////
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
int16_t ax, ay, az;
int16_t gx, gy, gz;

long sendCount = 0;
long frameCount = 0;

/***WIFI NAME AND PASSWORD****/
const char* ssid     = "C3P";
const char* password = "trespatios";
//const char* ssid     = "Your SSID name";
//const char* password = "YourPassword";

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//const IPAddress outIp(192, 168, 1, 95);
const IPAddress outIp(192, 168, 0, 141);
const unsigned int outPort = 10101;



void sendBundleViaOSC();

void getGyro();
void getAccel();

void sendViaOSC();

void setup() {



  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(4, 5);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(0, LOW);
    delay(10);
    digitalWrite(0, HIGH);
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void loop() {

  sendCount ++;
  frameCount++;
  if (frameCount < 2) {
    digitalWrite(2, LOW); //blue LED on
  } else {
    digitalWrite(2, HIGH);
  }
  if (frameCount > 500) {
    frameCount = 0;
  }
  if (sendCount > 1000)
  {
    getGyro();  // Print ?G: gx, gy, gz?
    getAccel(); // Print ?A: ax, ay, az?

    //sendViaOSC();
    sendBundleViaOSC();
  }
}

void sendViaOSC() {
  OSCMessage msg("/esp/accelX");
  msg.add(ax);
  msg.add("/esp/accelY");
  msg.add(ay);
  msg.add("/esp/accelZ");
  msg.add(az);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  sendCount = 0;
}

void sendBundleViaOSC() {
  OSCBundle bndl;

  bndl.add("/esp/accelX").add(ax);
  bndl.add("/esp/accelY").add(ay);
  bndl.add("/esp/accelZ").add(az);
  bndl.add("/esp/gyroX").add(gx);
  bndl.add("/esp/gyroY").add(gy);
  bndl.add("/esp/gyroZ").add(gz);

  Udp.beginPacket(outIp, outPort);
  bndl.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one

  // Serial.println(aX);
}



void getGyro()
{
  accelgyro.getRotation(&gx, &gy, &gz);
  Serial.print("gx:");
  Serial.println(gx);
  Serial.print("gy:");
  Serial.println(gy);
  Serial.print("gz:");
  Serial.println(gz);

}

void getAccel()
{
  accelgyro.getAcceleration(&ax, &ay, &az);
  Serial.print("ax:");
  Serial.println(ax);
  Serial.print("ay:");
  Serial.println(ay);
  Serial.print("az:");
  Serial.println(az);


}
