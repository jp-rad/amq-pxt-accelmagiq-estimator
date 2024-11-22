#ifndef ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
#define ACCELMAGIQLIB_QUATERNIONESTIMATOR_H

#include "pxt.h"
#include "AccelMagiQLibConfig.h"
#include "AccelMagiQLibCoordinateSpaceFilter.h"

namespace accelmagiqlib
{
    /**
     * @class QuaternionEstimator
     * @brief This class estimates quaternion orientation based on accelerometer and magnetometer data.
     */
    class QuaternionEstimator : public MicroBitComponent
    {
    public:
        /**
         * @brief Constructor to initialize the QuaternionEstimator.
         */
        QuaternionEstimator()
            : updateSampleTimestamp(0),
              filterAccel(), filterMagne(),
              qw(1.0), qx(0.0), qy(0.0), qz(0.0)
        {
            fiber_add_idle_component(this);
        }

    private:
        // next sample timestamp scheduled
        uint64_t updateSampleTimestamp;

        /**
         * @brief Periodic callback from MicroBit scheduler.
         */
        void idleUpdate();

    public:
#if MICROBIT_CODAL

        /**
         * @brief Periodic callback from MicroBit scheduler.
         */
        virtual void idleCallback();

#else // MICROBIT_CODAL

        /**
         * @brief Periodic callback from MicroBit scheduler.
         */
        virtual void idleTick();

#endif // MICROBIT_CODAL

    public:
        // Getters for quaternion components
        /**
         * @brief Get the W component of the quaternion.
         * @return The W component of the quaternion.
         */
        double getW() const;

        /**
         * @brief Get the X component of the quaternion.
         * @return The X component of the quaternion.
         */
        double getX() const;

        /**
         * @brief Get the Y component of the quaternion.
         * @return The Y component of the quaternion.
         */
        double getY() const;

        /**
         * @brief Get the Z component of the quaternion.
         * @return The Z component of the quaternion.
         */
        double getZ() const;

        /**
         * @brief Set the alpha value for the low-pass filters.
         * @param alpha The new alpha value. Should be in the range of 0.0 to 1.0.
         */
        void setLowPassFilterAlpha(const double alpha);

    public:
        /**
         * @brief Sets the coordinate system for the filter.
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_RAW: 0
         *               - COORDINATE_SYSTEM_BASIC: 1
         *               - COORDINATE_SYSTEM_TILT: 2
         */
        void setCoordinateSystem(const int system);

        /**
         * @brief Perform the quaternion estimation.
         *
         * This function calculates the quaternion based on the current sensor data and the selected estimation method.
         */
        void estimate();

    private:
        // Acceleration filter
        CoordinateSpaceFilter filterAccel; /**< Filter for accelerometer data */

        // Magnetic force filter
        CoordinateSpaceFilter filterMagne; /**< Filter for magnetometer data */

        // Quaternion (normalized)
        double qw; /**< W component of the quaternion */
        double qx; /**< X component of the quaternion */
        double qy; /**< Y component of the quaternion */
        double qz; /**< Z component of the quaternion */
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_QUATERNIONESTIMATOR_H
