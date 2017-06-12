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
//#include "MPU6050.h"

#include "MPU6050_6Axis_MotionApps20.h"

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
//#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t ax, ay, az;
int16_t gx, gy, gz;

long sendCount = 0;
long frameCount = 0;

/***WIFI NAME AND PASSWORD****/
const char* ssid     = "Hostal de la 57";
const char* password = "hostal57";
//const char* ssid     = "Your SSID name";
//const char* password = "YourPassword";

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//const IPAddress outIp(192, 168, 1, 95);
const IPAddress outIp(192, 168, 1, 92);
const unsigned int outPort = 10101;



void sendBundleViaOSC();

void getGyro();
void getAccel();
void getMpuData();

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


  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = accelgyro.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  accelgyro.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        accelgyro.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = accelgyro.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


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
    //getGyro();  // Print ?G: gx, gy, gz?
    //getAccel(); // Print ?A: ax, ay, az?

    getMpuData();

    //sendViaOSC();
    //sendBundleViaOSC();
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

void getMpuData() {
  mpuIntStatus = accelgyro.getIntStatus();

  // get current FIFO count
  fifoCount = accelgyro.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
    #endif
  }
}

void getGyro() {
  accelgyro.getRotation(&gx, &gy, &gz);
}

void printGyro() {
  Serial.print("gx:");
  Serial.println(gx);
  Serial.print("gy:");
  Serial.println(gy);
  Serial.print("gz:");
  Serial.println(gz);
}

void getAccel() {
  //  accelgyro.getAcceleration(&ax, &ay, &az);
  #ifdef OUTPUT_READABLE_REALACCEL
  // display real acceleration, adjusted to remove gravity
  //accelgyro.dmpGetQuaternion(&q, fifoBuffer);
  accelgyro.dmpGetAccel(&aa, fifoBuffer);
  //accelgyro.dmpGetGravity(&gravity, &q);
  accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
  #endif

  /* Serial.print("ax:"); */
  /* Serial.println(ax); */
  /* Serial.print("ay:"); */
  /* Serial.println(ay); */
  /* Serial.print("az:"); */
  /* Serial.println(az); */
}
