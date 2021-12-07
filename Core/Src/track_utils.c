#include "track_utils.h"

char* computeCurrentCarPosition(int32_t x_axis) {

	//This method is called inside a mutex so there's no need to protect the sensors struct here

	int value = x_axis;

	if (value < 0 && value > -100) {

		return CLEAR_TRACK
				"|          *         |\r\n"
				CLEAR_TRACK;

	} else if (value < -100 && value > -200) {

		return  CLEAR_TRACK
				"|        *           |\r\n"
				CLEAR_TRACK;

	} else if (value < -200 && value > -300) {

		return  CLEAR_TRACK
				"|       *            |\r\n"
				CLEAR_TRACK;

	} else if (value < -300 && value > -400) {

		return  CLEAR_TRACK
				"|      *             |\r\n"
				CLEAR_TRACK;

	} else if (value < -400 && value > -500) {

		return  CLEAR_TRACK
				"|      *             |\r\n"
				CLEAR_TRACK;

	} else if (value < -500 && value > -600) {

		return  CLEAR_TRACK
				"|    *               |\r\n"
				CLEAR_TRACK;

	} else if (value < -600 && value > -700) {

		return  CLEAR_TRACK
				"|   *                |\r\n"
				CLEAR_TRACK;

	} else if (value < -700 && value > -800) {

		return CLEAR_TRACK
				"|  *                 |\r\n"
				CLEAR_TRACK;

	} else if (value < -800) {

		return  CLEAR_TRACK
				"|*                   |\r\n"
				CLEAR_TRACK;

	} else if (value > 0 && value < 100) {

		return CLEAR_TRACK
				"|          *         |\r\n"
				CLEAR_TRACK;

	} else if (value > 100 && value < 200) {

		return  CLEAR_TRACK
				"|           *        |\r\n"
				CLEAR_TRACK;

	} else if (value > 200 && value < 300) {

		return  CLEAR_TRACK
				"|            *       |\r\n"
				CLEAR_TRACK;

	} else if (value > 300 && value < 400) {

		return  CLEAR_TRACK
				"|             *      |\r\n"
				CLEAR_TRACK;

	} else if (value > 400 && value < 500) {

		return  CLEAR_TRACK
				"|              *     |\r\n"
				CLEAR_TRACK;

	} else if (value > 500 && value < 600) {

		return  CLEAR_TRACK
				"|               *    |\r\n"
				CLEAR_TRACK;

	} else if (value > 600 && value < 700) {

		return  CLEAR_TRACK
				"|                *   |\r\n"
				CLEAR_TRACK;


	} else if (value > 700 && value < 800) {

		return  CLEAR_TRACK
				"|                 *  |\r\n"
				CLEAR_TRACK;

	} else if (value > 800) {

		return  CLEAR_TRACK
				"|                   *|\r\n"
				CLEAR_TRACK;
	}

	else {

		return "";
	}

}
