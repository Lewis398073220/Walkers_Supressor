/***************************************************************************
 *
 * Copyright 2015-2023 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#ifndef _ORIENTATION_MATH_H_
#define _ORIENTATION_MATH_H_

#pragma once
#include <math.h>

# define PI           3.1415927

struct QUATERNIONC {
    float q[4];
};

#ifdef __cplusplus

extern "C"
{
#endif
extern void updatequaternionCompC(struct QUATERNIONC *q, float *norm_gyr, float gyr[3],
                                         float acc[3], float deltaT, float alpha, float *phi);
extern void updateQuaternionGyrC(struct QUATERNIONC *q, float *norm_gyr, float gyr[3], float deltaT);

extern struct QUATERNIONC quaternionCInit();

void quaternionCInitWithVal(struct QUATERNIONC *qArray, float q0, float q1, float q2, float q3);

void cloneC(struct QUATERNIONC *qClone, struct QUATERNIONC *qArray);

void setFromAngleAxisC(struct QUATERNIONC *q, float angle, float vx, float vy, float vz);

float lengthC(struct QUATERNIONC *q);

void normalizeC(struct QUATERNIONC *qArray);

void inverseC(struct QUATERNIONC *qRtn, struct QUATERNIONC *qArray);

void multiplyC(struct QUATERNIONC *rtn, struct QUATERNIONC *a, struct QUATERNIONC *b);

/* function to rotate a quaternion by r * q * r^{-1} */
void rotateC(struct QUATERNIONC *rotateRtn, struct QUATERNIONC *r, struct QUATERNIONC *q);

#ifdef __cplusplus
}
#endif
#endif
