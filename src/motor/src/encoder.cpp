#include <motor/config.h>
#include <motor/encoder.h>

Encoder::Encoder(int pin, void (*callback)(void)) {
  pin_ = pin;
  prev_update_time_ = micros();
  prev_ticks_ = 0;

  if(wiringPiISR(pin_, INT_EDGE_BOTH, callback) < 0 ) {
    fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno));
    return;
  }
}

float Encoder::getRPM() {
  unsigned long current_time = micros();
  unsigned long dt = current_time - prev_update_time_;
  double dtm = (double)dt / 60000000;
  double delta_ticks = ticks_ - prev_ticks_;

  prev_update_time_ = current_time;
  prev_ticks_ = 0;
  ticks_ = 0;

  return (delta_ticks / COUNTS_PER_REV) / dtm;
}
