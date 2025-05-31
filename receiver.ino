#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(A0, A1); // CE, CSN
const byte address[6] = "12345";

int l1 = 13;// Arm 1 lenght
int l2 = 19;// arm 2 length

int x= -10; //initial end point co-ordinates
int y= 20;

Servo s1; //shoulder servo
Servo s2; // elbow servo

Servo s3; //griper rotation
Servo s4; // gripper

Servo base;//base servo for rotation

int la=7; //left motors
int lb=8;

int ra=A2;//right motors
int rb=A3;

int pwml=9;//pwm left motor
int pwmr=10;//right


struct package {
int vp1;
int vp2;
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

s1.attach(2);
s2.attach(4);
s3.attach(5);
s4.attach(6);
base.attach(3);
pinMode(A2,OUTPUT);
pinMode(A3,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(9,OUTPUT);
pinMode(10,OUTPUT);
// analogWrite(pwml,255);
// analogWrite(pwmr,255);
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(0, address);
radio.setPALevel(RF24_PA_MIN);
radio.startListening(); //  Set the module as receiver
}


void front(){
analogWrite(pwml,255);
analogWrite(pwmr,255);
digitalWrite(la,HIGH);
digitalWrite(lb,LOW);
digitalWrite(ra,HIGH);
digitalWrite(rb,LOW);
}



void back(){
analogWrite(pwml,255);
analogWrite(pwmr,255);
digitalWrite(la,LOW);
digitalWrite(lb,HIGH);
digitalWrite(ra,LOW);
digitalWrite(rb,HIGH);
}


void right(){
analogWrite(pwml,255);
analogWrite(pwmr,255);
digitalWrite(la,HIGH);
digitalWrite(lb,LOW);
digitalWrite(ra,LOW);
digitalWrite(rb,HIGH);
}



void left(){

analogWrite(pwml,255);
analogWrite(pwmr,255);
digitalWrite(la,LOW);
digitalWrite(lb,HIGH);
digitalWrite(ra,HIGH);
digitalWrite(rb,LOW);
}


void stop(){
digitalWrite(la,LOW);
digitalWrite(lb,LOW);
digitalWrite(ra,LOW);
digitalWrite(rb,LOW);}


void drivemode()//drive mode arm with no arm function
{
if(data.vj1x>700){

  front();
}

else if(data.vj1x<300){

  back();
}

else if(data.vj2y>700){

  right();
}

else if(data.vj2y<300){
left();
}
}


void armmode() // arm function
{ 

if(data.vj1x>700){

  x++;
}

else if(data.vj1x<300){

  x--;
}
if(data.vj1y>700){

  y++;
}

else if(data.vj1y<300){

  y--;
}
  //angle calculation for shoulder and elbow joint
float theta1, theta2;

    if (inverseKinematics(x, y, theta1, theta2)) // x,y endpoint co-ordinate
     {

      theta1 = degrees(theta1);
      theta2 = degrees(theta2);
      Serial.print(x);
      Serial.print('\t');
      Serial.print(y);
      Serial.print('\t');
      Serial.print(theta1);
      Serial.print('\t');
      Serial.println(theta2);

      s1.write(theta1);
      s2.write(theta2);     

    }else{
      Serial.println("unreachable");
    }

 }

void loop() {
  
  if (radio.available()) {
    radio.read(&data, sizeof(package));

    if(data.vt1){
      armmode();
    }

    else{
      drivemode();
    }


 }
}

bool inverseKinematics(float x, float y, float &theta1, float &theta2) {

  float r = sqrt(x * x + y * y);


  if (r > (l1 + l2) || r < abs(l1 - l2)) {
    return false;  // Position is out of reach
  }

  float cosTheta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  float sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2); // Using the sine identity
  
  theta2 = atan2(sinTheta2, cosTheta2);
  float k1 = l1 + l2 * cos(theta2);
  float k2 = l2 * sin(theta2);
  
  theta1 = atan2(y, x) - atan2(k2, k1);

  return true;
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




