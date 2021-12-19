#include "track_utils.h"

char* computeCurrentCarPosition(int32_t x_axis) {

	//This method is called inside a mutex so there's no need to protect the sensors struct here

	int value = x_axis;

	if (value < 0 && value > -100) {

		return "|        x       |\r\n";

	} else if (value < -100 && value > -200) {

		return  "|      x         |\r\n";

	} else if (value < -200 && value > -300) {

		return  "|     x          |\r\n";

	} else if (value < -300 && value > -400) {

		return  "|    x           |\r\n";

	} else if (value < -400 && value > -500) {

		return "|    x           |\r\n";

	} else if (value < -500 && value > -600) {

		return "|  x             |\r\n";

	} else if (value < -600 && value > -700) {

		return "| x              |\r\n";

	} else if (value < -700) {

		return "|x               |\r\n";

	} else if (value > 0 && value < 100) {

		return"|        x       |\r\n";

	} else if (value > 100 && value < 200) {

		return "|         x      |\r\n";

	} else if (value > 200 && value < 300) {

		return"|          x     |\r\n";

	} else if (value > 300 && value < 400) {

		return"|           x    |\r\n";

	} else if (value > 400 && value < 500) {

		return"|            x   |\r\n";

	} else if (value > 500 && value < 600) {

		return"|             x  |\r\n";

	} else if (value > 600 && value < 700) {

		return"|              x |\r\n";


	} else if (value > 700) {

		return"|               x|\r\n";
	}

	else {

		return "";
	}
}

char* performPitStop() {

	return PIT_LANE
			"|                ||  x  |\r\n"
			PIT_LANE
			"\n";
}

char* computeTrackOpponent(u_int8_t position) {

	if (position <= 4) {

		return"|  *             |\r\n";

	} else if (position > 4 && position <= 8) {

		return"|     *          |\r\n";

	} else if (position > 8 && position <= 12) {

		return"|          *     |\r\n";

	} else if (position > 12) {

		return"|             *  |\r\n";

	} else {

		return CLEAR_TRACK;
	}
}

char* getClearTrackString() {

	return CLEAR_TRACK;
}
