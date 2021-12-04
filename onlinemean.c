/*
 * Welford's Online algorithm to estimate population mean and variance.
 *
 *  Created on: Feb 6, 2019
 *      Author: Danylo Ulianych
 */

#include <math.h>
#include "onlinemean.h"

void OnlineMean_Init(OnlineMean *oMean) {
	OnlineMean_Reset(oMean);
}

void OnlineMean_Update(OnlineMean *oMean, float newValue) {
	oMean->count++;
	if (oMean->count > 1) {
		float delta = newValue - oMean->mean;
		oMean->mean += delta / oMean->count;
	} else {
		oMean->mean = newValue;
	}
}

float OnlineMean_GetMean(OnlineMean *oMean) {
	return oMean->mean;
}

void OnlineMean_Reset(OnlineMean *oMean) {
	oMean->count = 0;
	oMean->mean = 0.f;
}
