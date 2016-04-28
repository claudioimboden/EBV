/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

int TextColor;

float bgrImg[IMG_SIZE];
const float avgFac = 0.99;

const int Border = 1;

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}


void SetBackground() {
	int r, c;
//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			bgrImg[r + c] = (float) data.u8TempImage[SENSORIMG][r + c];
		}
	}
}

void ChangeDetection() {
	int r, c;
//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			float pImg = data.u8TempImage[SENSORIMG][r + c];
			float pBgr = bgrImg[r + c];
			float Dif = fabs(pImg - pBgr);
//if the difference is larger than threshold value
			if (Dif > data.ipc.state.nThreshold) {
//set pixel value to 255 in THRESHOLD
				data.u8TempImage[THRESHOLD][r + c] = 255;
			} else {
// update background image
				bgrImg[r + c] = avgFac * bgrImg[r + c]
						+ (1 - avgFac)
								* (float) data.u8TempImage[SENSORIMG][r + c];
// set value for display
				data.u8TempImage[BACKGROUND][r + c] = (unsigned char) bgrImg[r
						+ c];
			}
		}
	}
}

void ResetProcess() {
	//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
	SetBackground();
}

void ProcessFrame() {
	char Text[] = "hallo world";
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for copying sensor image to background image
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG],
		IMG_SIZE);
//
		ChangeDetection();

//		//example for drawing output
//		//draw line
//		DrawLine(10, 100, 200, 20, RED);
//		//draw open rectangle
//		DrawBoundingBox(20, 10, 50, 40, false, GREEN);
//		//draw filled rectangle
//		DrawBoundingBox(80, 100, 110, 120, true, BLUE);
//		DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
}

