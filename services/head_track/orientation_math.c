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
#include "orientation_math.h"

struct QUATERNIONC quaternionCInit(void)
{
    struct QUATERNIONC qArray;
    qArray.q[0] = 1.0;
    qArray.q[1] = 0.0;
    qArray.q[2] = 0.0;
    qArray.q[3] = 0.0;
    return qArray;
}
void quaternionCInitWithVal(struct QUATERNIONC *qArray, float q0, float q1, float q2, float q3)
{
    qArray->q[0] = q0;
    qArray->q[1] = q1;
    qArray->q[2] = q2;
    qArray->q[3] = q3;
}
void cloneC(struct QUATERNIONC *qClone, struct QUATERNIONC *qArray)
{
    qClone->q[0] = qArray->q[0];
    qClone->q[1] = qArray->q[1];
    qClone->q[2] = qArray->q[2];
    qClone->q[3] = qArray->q[3];
}
void setFromAngleAxisC(struct QUATERNIONC *q, float angle, float vx, float vy, float vz)
{
    float deg2rad = PI/180.0;
    quaternionCInitWithVal(q, (float)cos(deg2rad * angle/2),
                            vx * (float)sin(deg2rad * angle/2),
                            vy * (float)sin(deg2rad * angle/2),
                            vz * (float)sin(deg2rad * angle/2));
}

float lengthC(struct QUATERNIONC *q)
{
    return (float)sqrt((q->q[0]) * (q->q[0]) + (q->q[1]) * (q->q[1]) +
                (q->q[2]) * (q->q[2]) + (q->q[3]) * (q->q[3]));
}

void normalizeC(struct QUATERNIONC *qArray)
{
    float l = lengthC(qArray);
    qArray->q[0] = qArray->q[0]/l;
    qArray->q[1] = qArray->q[1]/l;
    qArray->q[2] = qArray->q[2]/l;
    qArray->q[3] = qArray->q[3]/l;
}

void inverseC(struct QUATERNIONC *qRtn, struct QUATERNIONC *qArray)
{
    float l2 = lengthC(qArray) * lengthC(qArray);
    qRtn->q[0] = qArray->q[0]/l2;
    qRtn->q[1] = -qArray->q[1]/l2;
    qRtn->q[2] = -qArray->q[2]/l2;
    qRtn->q[3] = -qArray->q[3]/l2;
}

void multiplyC(struct QUATERNIONC *rtn, struct QUATERNIONC *a, struct QUATERNIONC *b)
{
    quaternionCInitWithVal(rtn,
                            a->q[0]*b->q[0]-a->q[1]*b->q[1]-a->q[2]*b->q[2]-a->q[3]*b->q[3],
                            a->q[0]*b->q[1]+a->q[1]*b->q[0]+a->q[2]*b->q[3]-a->q[3]*b->q[2],
                            a->q[0]*b->q[2]-a->q[1]*b->q[3]+a->q[2]*b->q[0]+a->q[3]*b->q[1],
                            a->q[0]*b->q[3]+a->q[1]*b->q[2]-a->q[2]*b->q[1]+a->q[3]*b->q[0]);
}

/* function to rotate a quaternion by r * q * r^{-1} */
void rotateC(struct QUATERNIONC *rotateRtn, struct QUATERNIONC *r, struct QUATERNIONC *q)
{
    struct QUATERNIONC r_copy = quaternionCInit();
    struct QUATERNIONC prod = quaternionCInit();
    inverseC(&r_copy, r);
    multiplyC(&prod, r,q);
    multiplyC(rotateRtn, &prod, &r_copy);
}

void updateQuaternionGyrC(struct QUATERNIONC *q, float * norm_gyr, float gyr[3], float deltaT)
{
    /*
     * q is the previous quaternion estimate
     * update it to be the new quaternion estimate
     */
    *norm_gyr = (float)sqrt(pow(gyr[0],2.0) +  pow(gyr[1], 2.0) + pow(gyr[2], 2.0));

    if (*norm_gyr < (float)pow(10,-8)) {
        *norm_gyr = 1;
    }
    struct QUATERNIONC q_del;
    cloneC(&q_del, q);
    setFromAngleAxisC(&q_del, deltaT * (*norm_gyr), gyr[0]/(*norm_gyr), gyr[1]/(*norm_gyr), gyr[2]/(*norm_gyr));
    multiplyC(q, q, &q_del);
    normalizeC(q);
}

void updatequaternionCompC(struct QUATERNIONC *q, float * norm_gyr, float gyr[3],
                                  float acc[3], float deltaT, float alpha, float *phi)
{
    struct QUATERNIONC q_new;
    struct QUATERNIONC q_accel;
    struct QUATERNIONC q_accel_w;
    struct QUATERNIONC tilt;

    cloneC(&q_new, q);
    updateQuaternionGyrC(&q_new, norm_gyr, gyr, deltaT);
    quaternionCInitWithVal(&q_accel, 0, acc[0], acc[1], acc[2]);
    quaternionCInitWithVal(&tilt, 1, 0, 0, 0);
    float g_beta = lengthC(&q_accel);

    if(g_beta > 0.97 && g_beta < 1.03) {
        rotateC(&q_accel_w, &q_new, &q_accel);
        normalizeC(&q_accel_w);
        /* (-1, 0, 0) is g */
        *phi = (float)acos(-q_accel_w.q[1]) * 180 / (float)PI;
        float n = (float)sqrt(pow(-q_accel_w.q[3], 2.0) + pow(q_accel_w.q[2], 2.0));
        float ny = -q_accel_w.q[3] / n;
        float nz = q_accel_w.q[2] / n;
        setFromAngleAxisC(&tilt, (1-alpha) * (*phi), 0.0, ny, nz);
    }
    multiplyC(q, &tilt, &q_new);
}
