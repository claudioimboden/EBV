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

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

int TextColor;

float bgrImg[IMG_SIZE];

const float avgFac = 0.95;

/* skip pixel at border */
const int Border = 2;

/* after this number of steps object is set to background */
const int frgLimit = 100;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

void ChangeDetection();
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void BinningAngle(void);

void ResetProcess() {

}

void ProcessFrame() {

	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {

	} else {
		ChangeDetection();

		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);

		DetectRegions();

		DrawBoundingBoxes();

		BinningAngle();
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
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter in x-direction */
			int32 dx = -(int32) *(p - nc - 1) + (int32) *(p - nc + 1)
					- 2 * (int32) *(p - 1) + 2 * (int32) *(p + 1)
					- (int32) *(p + nc - 1) + (int32) *(p + nc + 1);
			int32 dy = -(int32) *(p - nc - 1) - 2 * (int32) *(p - nc)
					- (int32) *(p - nc + 1) + (int32) *(p + nc - 1)
					+ 2 * (int32) *(p + nc) + (int32) *(p + nc + 1);
			/* check if norm is larger than threshold */
			int32 df2 = dx * dx + dy * dy;
			int32 thr2 = data.ipc.state.nThreshold * data.ipc.state.nThreshold;
			if (df2 > thr2) { //avoid square root

//set pixel value to 255 in THRESHOLD image for gui
				data.u8TempImage[THRESHOLD][r + c] = 255;
			}
//store derivatives (int16 is enough)
			imgDx[r + c] = (int16) dx;
			imgDy[r + c] = (int16) dy;
//possibility to visualize data
			data.u8TempImage[BACKGROUND][r + c] = (uint8) MAX(0,
					MIN(255, 128+dx));
		}
	}
}

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

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, GREEN);
		}
	}
}

void BinningAngle() {
	uint16 c, o;
	float interval[] = { 0, 22.5, 67.5, 112.5, 157.5, 180 };
	uint16 bin[] = { 0, 0, 0, 0 };
	char* binDesc[4] = {"90", "135", "0", "45"};
	int i=0;
	//loop over objects
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			bin[0]=0;
			bin[1]=0;
			bin[2]=0;
			bin[3]=0;
			//get pointer to root run of current object
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
			//loop over runs of current object
			do {
				//loop over pixel of current run
				for (c = currentRun->startColumn; c <= currentRun->endColumn;
						c++) {
					int r = currentRun->row;
					//processing for individual pixel at row r and column c
					double angle = atan2(imgDy[r * nc + c], imgDx[r * nc + c]);

					if (angle < 0)
						angle += 3.141593;
					if (angle > 3.14 || angle < 0.1)
						angle = 0;
					if (angle != 0) {
						angle *= 57.2957;
						if (angle < interval[1]) {
							bin[0] += 1;
						} else if (angle < interval[2]) {
							bin[1] += 1;
						} else if (angle < interval[3]) {
							bin[2] += 1;
						} else if (angle < interval[4]) {
							bin[3] += 1;
						} else if (angle < interval[5]) {
							bin[0] += 1;
						}
					}
					//data.u8TempImage[SENSORIMG][r * nc + c]=0;

//				printf("%f\n", angle);
				}
				currentRun = currentRun->next; //get net run of current object
			} while (currentRun != NULL); //end of current object
//		printf("bin0%d\n", bin[0]);
//		printf("bin1%d\n", bin[1]);
//		printf("bin2%d\n", bin[2]);
//		printf("bin3%d\n", bin[3]);

			int max = 0;
			int maxIndex = 0;
			for (i = 0; i < sizeof(bin); i++) {
				if (bin[i] >= max) {
					max = bin[i];
					maxIndex = i;
				}
			}
			uint16 yPos = (ImgRegions.objects[o].bboxBottom
					- ImgRegions.objects[o].bboxTop) / 2
					+ ImgRegions.objects[o].bboxTop;
			uint16 xPos = (ImgRegions.objects[o].bboxRight
					- ImgRegions.objects[o].bboxLeft) / 2
					+ ImgRegions.objects[o].bboxLeft;
			DrawString(xPos, yPos, strlen(binDesc[maxIndex]), LARGE, GREEN,
					&binDesc[maxIndex][0]);
		}

	}

}
