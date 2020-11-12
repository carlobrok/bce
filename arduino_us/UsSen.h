#include <Arduino.h>

class UltrasonicSensor
{
private:
  int _trig_pin, _echo_pin;
  String _name;

public:
  UltrasonicSensor(int trig_pin, int echo_pin, String name) : _trig_pin(trig_pin), _echo_pin(echo_pin), _name(name)
  {
    pinMode(_trig_pin, OUTPUT);
    pinMode(_echo_pin, INPUT);
  };

  int distance()
  {
    unsigned long duration;
    double distance;

    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_trig_pin, LOW);

    duration = pulseIn(_echo_pin, HIGH, 117); //117 Microsekunden = timeout zeit für 2m Entfernung
    distance = duration * 0.034 / 2;
    return distance;
    
    
    

    /*if (distance != 0)
    {
      Serial.print(_name);
      Serial.println(distance);
    }
    if (distance = 0)
    {
      Serial.print(_name);
      Serial.println("error");
    }*/
  }

  
};
