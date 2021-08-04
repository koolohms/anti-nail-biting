#ifndef ADAFRUIT_BOARD
#define ADAFRUIT_BOARD

#define CHIRP_RTC_CAL_PULSE_LENGTH  10  //length (duration) of the pulse sent on the INT line to each sensor during calibration of the real-time clock

#define CHIRP_RST_PIN   6   //chirp rst gpio pin
#define CHIRP_PROG_PIN  9   //chirp prog gpio pin
#define CHIRP_INT_PIN   10  //chirp int gpio pin

#define CHIRP_INT_PINS  {CHIRP_INT_PIN}
#define CHIRP_PROG_PINS {CHIRP_PROG_PIN}
#define CHIRP_RST_PINS  {CHIRP_RST_PIN}

#endif