//This class contains all the utility methods for printing the track and the car

#include <stdio.h>

#define CLEAR_TRACK "|                    |\r\n"
#define PIT_LANE "|                    ||     |\r\n"

char* computeCurrentCarPosition(int32_t x_axis);

char* performPitStop();
