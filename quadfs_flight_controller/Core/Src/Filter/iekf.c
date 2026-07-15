/**
 ******************************************************************************
 *  QuadFS - quadcopter flight controller built from scratch on STM32
 *
 *  Copyright (C) 2026  Henry Odoemelem <huobuilds@gmail.com>
 *
 *  SPDX-License-Identifier: GPL-3.0-or-later
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/**
 ******************************************************************************
 * IEKF  attitude estimator
 * 
 * Design (unchanged): roll and pitch come from an indirect EKF driven by
 * gyro + accelerometer; the magnetometer is fed to the IEKF with a huge
 * variance so it cannot influence roll/pitch.
 *
 * CLEANUP CHANGELOG
 *  C1  doubles -> float32. Every double op is a softfloat library call on
 *      the Cortex-M4 FPU (single precision only). sqrt -> sqrtf etc.
 *  C2  quaternion normalized after prediction and after correction. Both
 *      steps inflate ||q|| and nothing ever shrank it; the error grows
 *      with rotation rate squared and accumulates for the whole session,
 *      distorting pitch (asin) and the DCM (2q^2-1 terms).
 *  C3  exact exponential map in the prediction (cos/sin closed form)
 *      instead of first order I + W*dt/2; falls back at |w| ~ 0.
 *  C4  process noise scaled by dt, not dt^2. Old form Pt = P+(R*dt)Q(R*dt)^T
 *      gave Qn*dt^2 per step; correct discrete white-noise integration is
 *      Qn*dt. Qn retuned from 0.1 (dt^2 convention) to 4e-4 (dt convention)
 *      so the effective gain is the same as before - the
 *      behavior is preserved, only the formulation is fixed.
 *  C5  removed the P-freeze counter. It was a LOCAL variable reset to 0 on
 *      every call, so P_calc_stop_counter <= 500 was always true and the
 *      freeze never triggered - dead code that only looked load-bearing.
 *      With C4 the covariance converges to a proper steady state anyway.
 *  C6  fixed the conversion constant inside IMU_EKF that was named to_deg
 *      but held the to-RADIANS value. Uses TO_RAD from quadfs_def.h.
 *  C8  norm guards: |mag| ~ 0 previously produced 0/0 = NaN, and even with
 *      mag variance 1e13 a NaN innovation times a tiny gain is still NaN -
 *      one bad mag sample poisoned the whole state permanently.
 *  C9  accelerometer validity gate: the acc innovation is zeroed when |acc|
 *      deviates more than 15 % from the at-rest gravity magnitude (captured
 *      at init, so it works in any units). Stops linear acceleration during
 *      maneuvers from injecting attitude error. Delete the two marked
 *      blocks to restore the old always-trust-acc behavior.
 *  C10 dead globals removed (unused gmps2, duplicate unit constants);
 *      init-only scratch moved to locals; consistent float suffixes.
 *
 ******************************************************************************
 */

#include "iekf.h"
#include <math.h>
#include <stdbool.h>
#include "quadfs_def.h"

enum filter_states
{
    init_filter,
    run_filter,
};

static enum filter_states filter_state = init_filter;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static arm_status status;
static arm_matrix_instance_f32 Ptilde;
static float32_t Ptildeval[9] = {0};
static arm_matrix_instance_f32 KSKT;
static float32_t KSKTval[9] = {0};
static arm_matrix_instance_f32 KS;
static float32_t KSval[18] = {0};
static arm_matrix_instance_f32 devs;
static float32_t devsval[3] = {0};
static arm_matrix_instance_f32 Er;
static float32_t Erval[6] = {0};
static arm_matrix_instance_f32 yhat;
static float32_t yhatval[6] = {0};
static arm_matrix_instance_f32 yekf;
static float32_t yval[6] = {0};
static arm_matrix_instance_f32 Rmv;
static float32_t Rmvval[3] = {0};
static arm_matrix_instance_f32 mv;
static float32_t mvval[3] = {0};
static arm_matrix_instance_f32 Rgv;
static float32_t Rgvval[3] = {0};
static arm_matrix_instance_f32 gv;
static float32_t gvval[3] = {0};
static arm_matrix_instance_f32 KT;
static float32_t KTval[18] = {0};
static arm_matrix_instance_f32 K;
static float32_t Kval[18] = {0};
static arm_matrix_instance_f32 PHT;
static float32_t PHTval[18] = {0};
static arm_matrix_instance_f32 SI;
static float32_t SIval[36] = {0};
static arm_matrix_instance_f32 Sekf;
static float32_t Sekfval[36] = {0};
static arm_matrix_instance_f32 R;
static float32_t Rval[36] = {0};
static arm_matrix_instance_f32 HPHT;
static float32_t HPHTval[36] = {0};
static arm_matrix_instance_f32 H;
static float32_t Hval[18] = {0};
static arm_matrix_instance_f32 HT;
static float32_t HTval[18] = {0};
static arm_matrix_instance_f32 HP;
static float32_t HPval[18] = {0};
static arm_matrix_instance_f32 gMat;
static float32_t gMatval[9] = {0};
static arm_matrix_instance_f32 mMat;
static float32_t mMatval[9] = {0};
static arm_matrix_instance_f32 Rg;
static float32_t Rgval[9] = {0};
static arm_matrix_instance_f32 Rm;
static float32_t Rmval[9] = {0};
static arm_matrix_instance_f32 wR;
static float32_t wRval[16] = {0};
static arm_matrix_instance_f32 qhat;
static float32_t qval[4] = {0};
static arm_matrix_instance_f32 qhatt;
static float32_t qt[4] = {0};
static arm_matrix_instance_f32 Rnb;
static float32_t Rnbval[9];
static arm_matrix_instance_f32 Rbn;
static float32_t Rbnval[9] = {0};
static arm_matrix_instance_f32 mRbn;
static float32_t mRbnval[9] = {0};
static arm_matrix_instance_f32 G;
static float32_t Gval[9] = {0};
static arm_matrix_instance_f32 Q;
static float32_t Qval[9] = {0};
static arm_matrix_instance_f32 GQ;
static float32_t GQval[9] = {0};
static arm_matrix_instance_f32 GT;
static float32_t GTval[9] = {0};
static arm_matrix_instance_f32 Qt;
static float32_t Qtval[9] = {0};
static arm_matrix_instance_f32 P;
static float32_t Pval[9] = {0};
static arm_matrix_instance_f32 Pt;
static float32_t Ptval[9] = {0};

static const float32_t m[3] = {0.4235f, 0.0272f, 0.9055f}; /* NED field*/

/* C9: gravity magnitude in the sensor's own units, captured at init */
static float acc_norm_ref = 1.0f;

static iekf_t iekf_data;

/* tuning - see C4 for the Qn retune */
#define IEKF_ACC_VAR 0.001f
#define IEKF_GYRO_VAR 0.0004f /* dt convention; matches old 0.1f @ dt^2*/
#define IEKF_P0 4.0e-9f       /* initial attitude is known from acc/mag */


// all mag contributions are disabled in the algorithm, 
// so this mag variance value is to keep the matrice valid
#define IEKF_MAG_VAR 1.0f 


static void init_IMU_EKF(float dt, float q0i, float q1i, float q2i, float q3i);
static void IMU_EKF(float magX, float magY, float magZ,
                    float accX, float accY, float accZ,
                    float gyroX, float gyroY, float gyroZ, float dt);
static void orientation_estimation(float sample_time_ms, const imu_t *d);
static float clampf(float x, float min_val, float max_val);
static void normalize_quaternion(volatile float32_t *qv);
static void init_matrices(void);

iekf_t iekf_orientation_estimation(float sample_time_ms, const imu_t imu_data)
{
    imu_t data = imu_data;
    orientation_estimation(sample_time_ms, &data);
    return iekf_data;
}

static void orientation_estimation(float sample_time_ms, const imu_t *d)
{
    const float declination = DECLINATION;
    const float dt = sample_time_ms * 0.001f; /* THIS API IS MILLISECONDS */

    switch (filter_state)
    {
        case init_filter:
        {
            /* estimate orientation from accelerometer and magnetometer */
            float roll_acc = atan2f(d->accYn, d->accZn) * TO_DEG;
            float pitch_acc = atan2f(-d->accXn,
                                    sqrtf(d->accYn * d->accYn + d->accZn * d->accZn)) *
                            TO_DEG;


            /* tilt-compensated heading (C8: guarded against |mag| = 0) */
            float yaw_raw = 0.0f;
            float magNorm = sqrtf(d->magXn * d->magXn + d->magYn * d->magYn + d->magZn * d->magZn);
            if (magNorm > 1e-6f)
            {
                float sr = sinf(roll_acc * TO_RAD), cr = cosf(roll_acc * TO_RAD);
                float sp = sinf(pitch_acc * TO_RAD), cp = cosf(pitch_acc * TO_RAD);
                float num = d->magZn * sr - d->magYn * cr;
                float den = d->magXn * cp + d->magYn * sp * sr + d->magZn * sp * cr;
                yaw_raw = atan2f(num, den) * TO_DEG - declination;
            }

            /* initial quaternion (q0 real part)*/
            float hr = roll_acc * TO_RAD / 2.0f;
            float hp = pitch_acc * TO_RAD / 2.0f;
            float hy = yaw_raw * TO_RAD / 2.0f;
            q0 = cosf(hr) * cosf(hp) * cosf(hy) + sinf(hr) * sinf(hp) * sinf(hy);
            q1 = sinf(hr) * cosf(hp) * cosf(hy) - cosf(hr) * sinf(hp) * sinf(hy);
            q2 = cosf(hr) * sinf(hp) * cosf(hy) + sinf(hr) * cosf(hp) * sinf(hy);
            q3 = cosf(hr) * cosf(hp) * sinf(hy) - sinf(hr) * sinf(hp) * cosf(hy);

            /* C9: board is at rest at init -> |acc| = 1 g in sensor units */
            acc_norm_ref = sqrtf(d->accXn * d->accXn + d->accYn * d->accYn + d->accZn * d->accZn);
            if (acc_norm_ref < 1e-6f)
            {
                acc_norm_ref = 1.0f;
            }

            init_IMU_EKF(dt, q0, q1, q2, q3);

            iekf_data.roll = roll_acc;
            iekf_data.pitch = pitch_acc;
            iekf_data.yaw = yaw_raw;

            filter_state = run_filter;
            break;
        }
        case run_filter:
        {
            /* indirect EKF: roll and pitch */
            IMU_EKF(d->magXn, d->magYn, d->magZn,
                    d->accXn, d->accYn, d->accZn,
                    d->gyroXn, d->gyroYn, d->gyroZn, dt);

            iekf_data.roll = atan2f(2.0f * (qval[0] * qval[1] + qval[2] * qval[3]),
                                    qval[0] * qval[0] - qval[1] * qval[1] - qval[2] * qval[2] + qval[3] * qval[3]) *
                            TO_DEG;

            float sin_pitch = -2.0f * (qval[1] * qval[3] - qval[0] * qval[2]);
            sin_pitch = clampf(sin_pitch, -1.0f, 1.0f); /* avoid nan at 90 deg */
            iekf_data.pitch = asinf(sin_pitch) * TO_DEG;
            iekf_data.yaw = 0.0f;
            break;
        }
        default:
            break;
    }
}

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val)
        return min_val;
    if (x > max_val)
        return max_val;
    return x;
}

/* C2: neither the prediction nor the multiplicative correction preserves
   ||q||, and the correction can only INCREASE it - without this the norm
   error accumulates with every fast rotation and never decays */
static void normalize_quaternion(volatile float32_t *qv)
{
    float n = sqrtf(qv[0] * qv[0] + qv[1] * qv[1] + qv[2] * qv[2] + qv[3] * qv[3]);
    if (n > 1e-6f)
    {
        float inv = 1.0f / n;
        qv[0] *= inv;
        qv[1] *= inv;
        qv[2] *= inv;
        qv[3] *= inv;
    }
    else
    {
        qv[0] = 1.0f;
        qv[1] = 0.0f;
        qv[2] = 0.0f;
        qv[3] = 0.0f;
    }
}

static void init_IMU_EKF(float dt, float q0i, float q1i, float q2i, float q3i)
{
    (void)dt;
    init_matrices();

    /* initial state covariance */
    Pval[0] = IEKF_P0;
    Pval[4] = IEKF_P0;
    Pval[8] = IEKF_P0;

    /* initial quaternion orientation */
    qval[0] = q0i;
    qval[1] = q1i;
    qval[2] = q2i;
    qval[3] = q3i;
    normalize_quaternion(qval);

    /* acc variance */
    Rval[0] = IEKF_ACC_VAR;
    Rval[7] = IEKF_ACC_VAR;
    Rval[14] = IEKF_ACC_VAR;

    // all mag contributions are disabled in the algorithm, 
    // so these mag variance values are to keep the matrice valid
    Rval[21] = IEKF_MAG_VAR;
    Rval[28] = IEKF_MAG_VAR;
    Rval[35] = IEKF_MAG_VAR;

    /* gyro variance (C4: dt convention) */
    Qval[0] = IEKF_GYRO_VAR;
    Qval[4] = IEKF_GYRO_VAR;
    Qval[8] = IEKF_GYRO_VAR;

    /* gravity vector: model expects accZ = +1 at rest */
    gvval[0] = 0.0f;
    gvval[1] = 0.0f;
    gvval[2] = -1.0f;
    gMatval[1] = -gvval[2]; // to +1g
    gMatval[3] = gvval[2];

    /* magnetic field vector and its skew-symmetric matrix */
    mvval[0] = m[0];
    mvval[1] = m[1];
    mvval[2] = m[2];
    mMatval[1] = -mvval[2];
    mMatval[2] = mvval[1];
    mMatval[3] = mvval[2];
    mMatval[5] = -mvval[0];
    mMatval[6] = -mvval[1];
    mMatval[7] = mvval[0];
}

static void IMU_EKF(float magX, float magY, float magZ,
                    float accX, float accY, float accZ,
                    float gyroX, float gyroY, float gyroZ, float dt)
{
    /* C6: this conversion was named to_deg but held the to-radians value */
    float gx = gyroX * TO_RAD;
    float gy = gyroY * TO_RAD;
    float gz = gyroZ * TO_RAD;

    /* Prediction ---------------------------------------------------------
       C3: exact exponential map dq=[cos(|w|dt/2),(w/|w|)sin(|w|dt/2)];
       the old first-order form under-rotates and inflates ||q|| at rate^2 */
    float w_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    float half = w_norm * dt / 2.0f;
    float c = cosf(half);
    float k = (w_norm > 1e-6f) ? (sinf(half) / w_norm) : (dt / 2.0f);

    wRval[0] = c;
    wRval[1] = -k * gx;
    wRval[2] = -k * gy;
    wRval[3] = -k * gz;
    wRval[4] = k * gx;
    wRval[5] = c;
    wRval[6] = k * gz;
    wRval[7] = -k * gy;
    wRval[8] = k * gy;
    wRval[9] = -k * gz;
    wRval[10] = c;
    wRval[11] = k * gx;
    wRval[12] = k * gz;
    wRval[13] = k * gy;
    wRval[14] = -k * gx;
    wRval[15] = c;

    status = arm_mat_mult_f32(&wR, &qhat, &qhatt); /* q_pred = exp(w) q */
    normalize_quaternion(qt);                      /* C2 */

    /* Correction ---------------------------------------------------------*/

    /* rotation matrix from the predicted quaternion */
    Rnbval[0] = 2.0f * qt[0] * qt[0] + 2.0f * qt[1] * qt[1] - 1.0f;
    Rnbval[1] = 2.0f * qt[1] * qt[2] - 2.0f * qt[0] * qt[3];
    Rnbval[2] = 2.0f * qt[1] * qt[3] + 2.0f * qt[0] * qt[2];
    Rnbval[3] = 2.0f * qt[1] * qt[2] + 2.0f * qt[0] * qt[3];
    Rnbval[4] = 2.0f * qt[0] * qt[0] + 2.0f * qt[2] * qt[2] - 1.0f;
    Rnbval[5] = 2.0f * qt[2] * qt[3] - 2.0f * qt[0] * qt[1];
    Rnbval[6] = 2.0f * qt[1] * qt[3] - 2.0f * qt[0] * qt[2];
    Rnbval[7] = 2.0f * qt[2] * qt[3] + 2.0f * qt[0] * qt[1];
    Rnbval[8] = 2.0f * qt[0] * qt[0] + 2.0f * qt[3] * qt[3] - 1.0f;
    status = arm_mat_trans_f32(&Rnb, &Rbn);

    /* state covariance (C4: Q*dt, not Q*dt^2; C5: freeze counter removed) */
    status = arm_mat_trans_f32(&Rnb, &GT);
    status = arm_mat_mult_f32(&Rnb, &Q, &GQ);
    status = arm_mat_mult_f32(&GQ, &GT, &G);
    status = arm_mat_scale_f32(&G, dt, &Qt);
    status = arm_mat_add_f32(&P, &Qt, &Pt);

    /* measurement matrix H */
    status = arm_mat_scale_f32(&Rbn, -1.0f, &mRbn);
    status = arm_mat_mult_f32(&mRbn, &gMat, &Rg);
    status = arm_mat_mult_f32(&Rbn, &mMat, &Rm);
    for (int i = 0; i < 9; i++)
    {
        Hval[i] = Rgval[i];
        // do not use mag, 
        // yaw influences roll and pitch angles, 
        // and we do yaw rate control not yaw angle hold
        // disable mag correction
        Hval[9 + i] = 0;

        //Hval[9 + i] = Rmval[i];
    }
    status = arm_mat_trans_f32(&H, &HT);

    /* Kalman gain */
    status = arm_mat_mult_f32(&H, &Pt, &HP);
    status = arm_mat_mult_f32(&HP, &HT, &HPHT);
    status = arm_mat_add_f32(&HPHT, &R, &Sekf);
    status = arm_mat_inverse_f32(&Sekf, &SI);
    status = arm_mat_mult_f32(&Pt, &HT, &PHT);
    status = arm_mat_mult_f32(&PHT, &SI, &K);
    status = arm_mat_trans_f32(&K, &KT);

    /* measurements (C8: norm guards; C9: acc validity gate) */
    float magNorm = sqrtf(magX * magX + magY * magY + magZ * magZ);
    float accNorm = sqrtf(accX * accX + accY * accY + accZ * accZ);
    float acc_ratio = accNorm / acc_norm_ref;
    bool acc_valid = (accNorm > 1e-6f) && (acc_ratio > 0.85f) && (acc_ratio < 1.15f);
    bool mag_valid = (magNorm > 1e-6f);

    yval[0] = acc_valid ? (accX / accNorm) : 0.0f;
    yval[1] = acc_valid ? (accY / accNorm) : 0.0f;
    yval[2] = acc_valid ? (accZ / accNorm) : 0.0f;
    
    
    // do not use mag, 
    // yaw influences roll and pitch angles, 
    // and we do yaw rate control not yaw angle hold
    // disable mag correction
    yval[3] = 0.0f;
    yval[4] = 0.0f;
    yval[5] = 0.0f;
    // yval[3] = mag_valid ? (magX / magNorm) : 0.0f;
    // yval[4] = mag_valid ? (magY / magNorm) : 0.0f;
    // yval[5] = mag_valid ? (magZ / magNorm) : 0.0f;

    /* estimate */
    status = arm_mat_mult_f32(&mRbn, &gv, &Rgv);
    status = arm_mat_mult_f32(&Rbn, &mv, &Rmv);
    yhatval[0] = Rgvval[0];
    yhatval[1] = Rgvval[1];
    yhatval[2] = Rgvval[2];
   
    // do not use mag, 
    // yaw influences roll and pitch angles, 
    // and we do yaw rate control not yaw angle hold
    // disable mag correction
    yhatval[3] = 0.0f;
    yhatval[4] = 0.0f;
    yhatval[5] = 0.0f;
    // yhatval[3] = Rmvval[0];
    // yhatval[4] = Rmvval[1];
    // yhatval[5] = Rmvval[2];

    /* orientation deviation */
    status = arm_mat_sub_f32(&yekf, &yhat, &Er);
    if (!acc_valid) /* C9: zero the innovation of any invalid sensor */
    {
        Erval[0] = 0.0f;
        Erval[1] = 0.0f;
        Erval[2] = 0.0f;
    }
    if (!mag_valid)
    {
        Erval[3] = 0.0f;
        Erval[4] = 0.0f;
        Erval[5] = 0.0f;
    }
    
    
    // do not use mag, 
    // yaw influences roll and pitch angles, 
    // and we do yaw rate control not yaw angle hold
    // disable mag correction
    Erval[3] = 0.0f;
    Erval[4] = 0.0f;
    Erval[5] = 0.0f;
    status = arm_mat_mult_f32(&K, &Er, &devs);

    /* correct state covariance */
    status = arm_mat_mult_f32(&K, &Sekf, &KS);
    status = arm_mat_mult_f32(&KS, &KT, &KSKT);
    status = arm_mat_sub_f32(&Pt, &KSKT, &Ptilde);
    for (int i = 0; i < 9; i++)
    {
        Pval[i] = Ptildeval[i];
    }

    /* correct quaternion: q = [1, devs/2] (x) q, then C2 normalize */
    qval[0] = qt[0] - qt[1] * devsval[0] / 2.0f - qt[2] * devsval[1] / 2.0f - qt[3] * devsval[2] / 2.0f;
    qval[1] = qt[1] + qt[0] * devsval[0] / 2.0f + qt[3] * devsval[1] / 2.0f - qt[2] * devsval[2] / 2.0f;
    qval[2] = qt[2] - qt[3] * devsval[0] / 2.0f + qt[0] * devsval[1] / 2.0f + qt[1] * devsval[2] / 2.0f;
    qval[3] = qt[3] + qt[2] * devsval[0] / 2.0f - qt[1] * devsval[1] / 2.0f + qt[0] * devsval[2] / 2.0f;
    normalize_quaternion(qval);
}

static void init_matrices(void)
{
    arm_mat_init_f32(&Ptilde, 3, 3, Ptildeval);
    arm_mat_init_f32(&KSKT, 3, 3, KSKTval);
    arm_mat_init_f32(&KS, 3, 6, KSval);
    arm_mat_init_f32(&devs, 3, 1, devsval);
    arm_mat_init_f32(&Er, 6, 1, Erval);
    arm_mat_init_f32(&yhat, 6, 1, yhatval);
    arm_mat_init_f32(&yekf, 6, 1, yval);
    arm_mat_init_f32(&Rmv, 3, 1, Rmvval);
    arm_mat_init_f32(&mv, 3, 1, mvval);
    arm_mat_init_f32(&Rgv, 3, 1, Rgvval);
    arm_mat_init_f32(&gv, 3, 1, gvval);
    arm_mat_init_f32(&KT, 6, 3, KTval);
    arm_mat_init_f32(&K, 3, 6, Kval);
    arm_mat_init_f32(&PHT, 3, 6, PHTval);
    arm_mat_init_f32(&SI, 6, 6, SIval);
    arm_mat_init_f32(&Sekf, 6, 6, Sekfval);
    arm_mat_init_f32(&R, 6, 6, Rval);
    arm_mat_init_f32(&HPHT, 6, 6, HPHTval);
    arm_mat_init_f32(&H, 6, 3, Hval);
    arm_mat_init_f32(&HT, 3, 6, HTval);
    arm_mat_init_f32(&HP, 6, 3, HPval);
    arm_mat_init_f32(&gMat, 3, 3, gMatval);
    arm_mat_init_f32(&mMat, 3, 3, mMatval);
    arm_mat_init_f32(&Rg, 3, 3, Rgval);
    arm_mat_init_f32(&Rm, 3, 3, Rmval);
    arm_mat_init_f32(&wR, 4, 4, wRval);
    arm_mat_init_f32(&qhat, 4, 1, qval);
    arm_mat_init_f32(&qhatt, 4, 1, qt);
    arm_mat_init_f32(&Rnb, 3, 3, Rnbval);
    arm_mat_init_f32(&Rbn, 3, 3, Rbnval);
    arm_mat_init_f32(&mRbn, 3, 3, mRbnval);
    arm_mat_init_f32(&G, 3, 3, Gval);
    arm_mat_init_f32(&Q, 3, 3, Qval);
    arm_mat_init_f32(&GQ, 3, 3, GQval);
    arm_mat_init_f32(&GT, 3, 3, GTval);
    arm_mat_init_f32(&Qt, 3, 3, Qtval);
    arm_mat_init_f32(&P, 3, 3, Pval);
    arm_mat_init_f32(&Pt, 3, 3, Ptval);
}