#include <avr/interrupt.h>

#define STATE_ERROR     -1
#define STATE_HOME       0
#define STATE_GPS        1
#define STATE_PULSE      2
#define STATE_TEMP       3
#define STATE_ACCEL      4
#define STATE_TRIP       5
#define STATE_TRIP_RESET 6
#define NUM_STATES       6

extern void button_init(void);
extern ISR(PCINT2_vect);
