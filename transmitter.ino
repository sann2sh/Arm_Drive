#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "12345";



const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float angleX, angleY;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;
float elapsedTime, currentTime, previousTime;
int c = 0;


int tg1=6; //toggle switch 1 conected to pin 6
int tg2=3; //toggle switch 2 connected to pin 3
int p1=A6; //potentiometer 1 to pin A6
int p2=A7; //potentiometer 2 to pin A7 and so on...
int j1x=A3; //left joystick x axis
int j1y=A0; //left joystick y axis
int j2x=A1; //right joystick x axis
int j2y=A2; //right joystick y axis
int j2s=2; //right joystick push button



struct package {
int vp1; // stores value of potentiometer 1
int vp2; // stores value of potentiometer 2 and so on...
int vj1x; 
int vj1y;
int vj2x;
int vj2y;
int vtg1;
int vtg2;
int vj2s;
float angX;
float angY;
};


package data; 




void setup() {

 radio.begin();
 radio.openWritingPipe(address);

 radio.setPALevel(RF24_PA_MIN);
radio.stopListening();


Serial.begin(9600);
initialize_MPU6050();
pinMode(tg1,INPUT_PULLUP);
pinMode(tg2,INPUT_PULLUP);
pinMode(p1,INPUT);
pinMode(p2,INPUT);
pinMode(j1x,INPUT);
pinMode(j1y,INPUT);
pinMode(j2y,INPUT);
pinMode(j2s,INPUT_PULLUP);

 data.vj2s = 1;
 data.vtg1 = 1;
 data.vtg2 = 1;


}


void initialize_MPU6050() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

void loop() {
 // Read all analog inputs
 data.vj1x = analogRead(j1x);
 data.vj1y = analogRead(j1y);
 data.vj2x = analogRead(j2x);
 data.vj2y = analogRead(j2y);
 data.vp1 = analogRead(p1);
 data.vp2 = analogRead(p2);
 // Read all digital inputs
 data.vj2s = digitalRead(j2s);
 data.vtg1 = digitalRead(tg1);
 data.vtg2 = digitalRead(tg2);
 //read IMU
read_IMU();  
 radio.write(&data, sizeof(package)); // send data
 
 

 }

void read_IMU() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value


  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom 
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)

  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + 1.85; //// GyroErrorX ~(-1.85)
  GyroY = GyroY - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
data.angX=angleX;
data.angY=angleY;
}

//for troubleshooting
//  void printdata(){

// Serial.print("Tg1=");
// Serial.print(data.vtg1);
// Serial.print("\t");

// Serial.print("Tg2=");
// Serial.print(data.vtg2);
// Serial.print("\t");

// Serial.print("J2S=");
// Serial.print(data.vj2s);
// Serial.print("\t");


// Serial.print("P1=");
// Serial.print(data.vp1);
// Serial.print("\t\t");

// Serial.print("P2=");
// Serial.print(data.vp2);
// Serial.print("\t\t");

// Serial.print("J1X=");
// Serial.print(data.vj1x);
// Serial.print("\t\t");


// Serial.print("J1Y=");
// Serial.print(data.vj1y);
// Serial.print("\t\t");

// Serial.print("J2X=");
// Serial.print(data.vj2x);
// Serial.print("\t\t");


// Serial.print("J2Y=");
// Serial.print(data.vj2y);
// Serial.print("\t\t");

// Serial.print("AngX=");
// Serial.print(data.angX);
// Serial.print("\t\t");

// Serial.print("AngY=");
// Serial.print(data.angY);
// Serial.println("\t\t");
// }
