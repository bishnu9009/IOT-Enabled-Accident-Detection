#define BLYNK_TEMPLATE_ID "TMPL3h-P1yoin"
#define BLYNK_DEVICE_NAME "Accident Location"
#define BLYNK_TEMPLATE_NAME "Accident Location"
#define BLYNK_AUTH_TOKEN "UPAWsInUPFhzsX-gHlfSpIgn0IERNeRY"

//GPS Module Settings
// GPS Module RX pin to ESP32 17
// GPS Module TX pin to ESP32 16

//MPU 6050 Module Settings
// SCL Connected to GPIO 22
// SDA Connected to GPIO 21

#define RXD2 16
#define TXD2 17

#include <Wire.h>             // For I2C Comms.
#include <WiFi.h>             // For ESP WiFi connection.
#include <WiFiClient.h>
#include <TinyGPS++.h> 
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

char auth[] = BLYNK_AUTH_TOKEN;
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "JioFiber-2.4Ghz";
char pass[] = "Jmn489#zmpSTR";

String message;

HardwareSerial neogps(2);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

BlynkTimer timer;

#define INTERVAL 1000L
/************************************************************************************
 *  This function sends Arduino's up time every second to Virtual Pin.
 *  In the app, Widget's reading frequency should be set to PUSH. This means
 *  that you define how often to send data to Blynk App.
 **********************************************************************************/
// float avg_acceleration;
// float avg_gyro;

void sendGps(){
  //-----------------------------------------------------------
  while(neogps.available()){
    if (gps.encode(neogps.read())){
      break;
    }
  }
  //-----------------------------------------------------------
  if (!gps.location.isValid()){
    Serial.println("Failed to read from GPS Module!");
    return;
  }
  //-----------------------------------------------------------
  //get latitude and longitude
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  float speed = gps.speed.kmph();
  //-----------------------------------------------------------
  //comment out this block of code to save space
  //used for debugging in serial monitor
  Serial.print("Latitude:  ");
  Serial.println(latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Speed: ");
  Serial.println(speed, 6);
  //-----------------------------------------------------------
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V1, String(latitude, 6));
  Blynk.virtualWrite(V2, String(longitude, 6));
  Blynk.virtualWrite(V0, String(speed));
  //-----------------------------------------------------------
  message = "https://maps.google.com/maps?&z=15&mrt=yp&t=k&q=" + String(latitude, 6) + "," + String(longitude, 6);

  Blynk.virtualWrite(V4, String(message));

}




/////////////////////////////////////////////////////////////////////////// MPU CODE

void sendSensor(){
  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    // Serial.print("AccelX:");
    // Serial.print(a.acceleration.x);
    // Serial.print(",");
    // Serial.print("AccelY:");
    // Serial.print(a.acceleration.y);
    // Serial.print(",");
    // Serial.print("AccelZ:");
    // Serial.print(a.acceleration.z);
    // Serial.print(", ");
    // Serial.print("GyroX:");
    // Serial.print(g.gyro.x);
    // Serial.print(",");
    // Serial.print("GyroY:");
    // Serial.print(g.gyro.y);
    // Serial.print(",");
    // Serial.print("GyroZ:");
    // Serial.print(g.gyro.z);
    // Serial.println("");
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    float avg_acceleration = (a.acceleration.x+a.acceleration.y+a.acceleration.z) / 3;
    float avg_gyro = (g.gyro.x+g.gyro.y+g.gyro.z) / 3;

    Serial.print("Avg A : ");
    Serial.print((a.acceleration.x+a.acceleration.y+a.acceleration.z) / 3);
    
    //Just Checking...
    // Serial.print("Avg G : ");
    // Serial.print((g.gyro.x+g.gyro.y+g.gyro.z) / 3);


    // Blynk.virtualWrite(V0, a.acceleration.x);
    // Blynk.virtualWrite(V1, a.acceleration.y);
    Blynk.virtualWrite(V3, avg_acceleration);
    // Blynk.virtualWrite(V3,g.gyro.x);
    // Blynk.virtualWrite(V4,g.gyro.y);
    // Blynk.virtualWrite(V4,avg_gyro); comment this in in case yoy eawnt gyro
    delay(200);
  }
}





/************************************************************************************
 *  setup() function
 **********************************************************************************/
void setup()
{
  //-----------------------------------------------------------
  //Debug console (Serial Monitor)
  Serial.begin(115200);

  // Try to initialize MPU6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  //setup the motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
 
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(100L, sendSensor);

  //-----------------------------------------------------------
  Blynk.begin(auth, ssid, pass);
  //You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  //-----------------------------------------------------------
  //Set GPS module baud rate
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("neogps serial initialize");
  delay(10);
  //-----------------------------------------------------------
  // Setup a function to be called every second
  timer.setInterval(INTERVAL, sendGps);
  //-----------------------------------------------------------
}


/************************************************************************************
 *  loop() function
 **********************************************************************************/
void loop(){
  Blynk.run();
  timer.run();
}