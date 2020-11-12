#include <Arduino.h>

class BuzzSpeaker
{
private:
    int _speakerPinOutput;

public:
    BuzzSpeaker(int speakerPinOutput) : _speakerPinOutput(speakerPinOutput){};

    void playSound(int frequency, int duration, int repeat)
    {
        for (int i = 0; i < repeat; i++)
        {
            tone(_speakerPinOutput, 2000);
            delay(duration);
        }
    }
};