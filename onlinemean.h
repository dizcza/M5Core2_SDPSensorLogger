/*
 * Welford's Online algorithm to estimate population mean and variance.
 *
 *  Created on: Feb 6, 2019
 *      Author: Danylo Ulianych
 */

#ifndef ONLINEMEAN_H_
#define ONLINEMEAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef struct OnlineMean {
	float mean;
	uint32_t count;
} OnlineMean;

void OnlineMean_Init(OnlineMean *oMean);
void OnlineMean_Update(OnlineMean *oMean, float newValue);
float OnlineMean_GetMean(OnlineMean *oMean);
void OnlineMean_Reset(OnlineMean *oMean);

#ifdef __cplusplus
}
#endif

#endif /* ONLINEMEAN_H_ */
