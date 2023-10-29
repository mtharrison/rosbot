#include <stdio.h>
#include <wiringPi.h>
#include <string.h>
#include <errno.h>

class Encoder {
  int pin_;
  
public:
  int ticks_;
  int prev_ticks_;
  unsigned long prev_update_time_;

  Encoder(int pin, void (*callback)(void));
  float getRPM();
};
