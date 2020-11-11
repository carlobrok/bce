#include "UsSen.h"
#include "BuzzSpeaker.h"

#define reUnTrig 13
#define reUnEch 12

#define liUnTrig 11
#define liUnEch 10

#define reObTrig 9
#define reObEch 8

#define liObTrig 6
#define liObEch 7

#define speakerPin 5

String reUnName = "ReUn";
String liUnName = "LiUn";
String reObName = "ReOb";
String liObName = "LiOb";

String value[3];

unsigned long tdistance;

UltrasonicSensor UsLiOb(liObTrig, liObEch, liObName);
UltrasonicSensor UsReOb(reObTrig, reObEch, reObName);
UltrasonicSensor UsLiUn(liUnTrig, liUnEch, liUnName);
UltrasonicSensor UsReUn(reUnTrig, reUnEch, reUnName);

BuzzSpeaker MainSpeaker(speakerPin);

float dist_l, dist_lm, dist_rm, dist_r;
String str_out = "";

const int buzzer_pin = 5;
unsigned long last_buzzer = 0;


void usSenCheck(double distance)
{
  if (distance != 0)
  {
    Serial.println(distance);
  }
  if (distance < 100 && distance > 0)
  {
    MainSpeaker.playSound(2000, distance, 1);
    Serial.println(distance);
  }
  if (distance == 0)
  {
    Serial.println("timeout");
  }
  tdistance = distance;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
// read distances of ultrasonic sensors
  usSenCheck(UsReOb.distance()); 
  value[0] = tdistance;
  usSenCheck(UsLiOb.distance());
  value[1] = tdistance;
  usSenCheck(UsReUn.distance());
  value[2] = tdistance;
  usSenCheck(UsLiUn.distance());
  value[3] = tdistance;
// write distances to serial port
  
  Serial.print(value[0]+";"+value[1]+";"+value[2]+"3"+value[3]+"\n");
  
  /*str_out = String(dist_l,1) + ";" + String(dist_lm,1) + ";" + String(dist_rm,1) + ";" + String(dist_r,1);
  Serial.println(str_out);*/

}
