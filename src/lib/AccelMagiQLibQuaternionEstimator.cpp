#include "AccelMagiQLibQuaternionEstimator.h"
#include <cmath>

using namespace accelmagiqlib;

void QuaternionEstimator::idleUpdate()
{
    // Schedule our next sample.
    uint64_t currentTime = system_timer_current_time_us();
    if (currentTime < updateSampleTimestamp)
    {
        return;
    }
    updateSampleTimestamp = currentTime + uBit.accelerometer.getPeriod() * 1000;

    // Do sampling
    double x;
    double y;
    double z;

    // Update and normalize accelerometer data
    x = uBit.accelerometer.getX();
    y = uBit.accelerometer.getY();
    z = uBit.accelerometer.getZ();
    filterAccel.update(x, y, z);

    // Update and normalize magnetometer data
    x = uBit.compass.getX();
    y = uBit.compass.getY();
    z = uBit.compass.getZ();
    filterMagne.update(x, y, z);
}

#if MICROBIT_CODAL

void QuaternionEstimator::idleCallback()
{
    idleUpdate();
}

#else // MICROBIT_CODAL

void QuaternionEstimator::idleTick()
{
    idleUpdate();
}

#endif // MICROBIT_CODAL

double QuaternionEstimator::getW() const
{
    return qw;
}

double QuaternionEstimator::getX() const
{
    return qx;
}

double QuaternionEstimator::getY() const
{
    return qy;
}

double QuaternionEstimator::getZ() const
{
    return qz;
}

void QuaternionEstimator::setLowPassFilterAlpha(const double alpha)
{
    filterAccel.setAlpha(alpha);
    filterMagne.setAlpha(alpha);
}

void QuaternionEstimator::setCoordinateSystem(const int system)
{
    filterAccel.setCoordinateSystem(system);
    filterMagne.setCoordinateSystem(system);
}

#if ACCELMAGIQ_ESTIMATE_METHOD == ACCELMAGIQ_ESTIMATE_METHOD_SIMPLE
void QuaternionEstimator::estimate()
{
    const double ax = filterAccel.getCoordX();
    const double ay = filterAccel.getCoordY();
    const double az = filterAccel.getCoordZ();

    // Accelerration Only
    double w = std::sqrt((az + 1.0) / 2.0);
    double x = ay / (2.0 * w);
    double y = -ax / (2.0 * w);
    double z = 0.0;

    // normalize
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (0 < norm)
    {
        norm = 1 / norm;
        qw = w * norm;
        qx = x * norm;
        qy = y * norm;
        qz = z * norm;
    }
}
#elif ACCELMAGIQ_ESTIMATE_METHOD == ACCELMAGIQ_ESTIMATE_METHOD_FAMC
void QuaternionEstimator::estimate()
{
    const double ax = filterAccel.getCoordX();
    const double ay = filterAccel.getCoordY();
    const double az = filterAccel.getCoordZ();
    const double mx = filterMagne.getCoordX();
    const double my = filterMagne.getCoordY();
    const double mz = filterMagne.getCoordZ();

    // ---------------------------------------------------------------------------------------------
    // A Simplified Analytic Attitude Determination Algorithm Using Accelerometer and Magnetometer
    // Fast Accelerometer-Magnetometer Combination (FAMC) algorithm by Zhuohua Liu and Jin Wu
    // ---------------------------------------------------------------------------------------------
    // https://github.com/zarathustr/Analytic-AMC/blob/master/FAMC.m

    // Dynamic magnetometer reference vector
    double m_D = ax * mx + ay * my + az * mz;
    double m_N = sqrt(1.0 - m_D * m_D);

    // Parameters
    double B11 = (m_N * mx) / 2.0;
    double B13 = ax / 2.0 + (m_D * mx) / 2.0;
    double B21 = (m_N * my) / 2.0;
    double B23 = ay / 2.0 + (m_D * my) / 2.0;
    double B31 = (m_N * mz) / 2.0;
    double B33 = az / 2.0 + (m_D * mz) / 2.0;

    double tau = B13 + B31;

    // First Row
    double p1 = B33 - B11 + 1.0;
    double A11 = -1.0 / p1;
    double A12 = B21 / p1;
    double A13 = tau / p1;

    // Second Row
    double p2 = -B21 * B21 / p1 + B11 + B33 + 1.0;
    double A21 = -B21 / (p1 * p2);
    double A22 = -1.0 / p2;
    double A23 = (B23 + B21 * tau / p1) / p2;

    // Third Row
    double p3 = p1 - 2.0 + tau * tau / p1 + A23 * A23 * p2;
    double A31 = (tau / p1 + B21 * A23 / p1) / p3;
    double A32 = A23 / p3;
    double A33 = 1.0 / p3;

    // Quaternion Elements
    double w = -1.0;
    double x = B23 * (A11 + A12 * (A21 + A23 * A31) + A13 * A31) - (B13 - B31) * (A21 + A23 * A31) - A31 * B21;
    double y = B23 * (A12 * (A22 + A23 * A32) + A13 * A32) - (B13 - B31) * (A22 + A23 * A32) - A32 * B21;
    double z = B23 * (A13 * A33 + A12 * A23 * A33) - A33 * B21 - A23 * A33 * (B13 - B31);

    // Normalize the quaternion
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (0 < norm)
    {
        norm = 1 / norm;
        qw = w * norm;
        qx = x * norm;
        qy = y * norm;
        qz = z * norm;
    }
}
#else
#error "Invalid ACCELMAGIQ_ESTIMATE_METHOD"
#endif
