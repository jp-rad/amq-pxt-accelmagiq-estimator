#ifndef ACCELMAGIQLIB_COORDINATESPACEFILTER_H
#define ACCELMAGIQLIB_COORDINATESPACEFILTER_H

#include "AccelMagiQLibConfig.h"
#include <cmath>

namespace accelmagiqlib
{

    /**
     * @class CoordinateSpaceFilter
     * @brief A class to filter and manage coordinates with low-pass filtering and different coordinate systems.
     */
    class CoordinateSpaceFilter
    {
    private:
        int currentSystem; /**< The current coordinate system being used */

        // Low-pass filter parameters
        double alphaX;         /**< Alpha value for low-pass filter on X-axis */
        double oneMinusAlphaX; /**< Precomputed (1.0 - alphaX) value */
        double alphaY;         /**< Alpha value for low-pass filter on Y-axis */
        double oneMinusAlphaY; /**< Precomputed (1.0 - alphaY) value */
        double alphaZ;         /**< Alpha value for low-pass filter on Z-axis */
        double oneMinusAlphaZ; /**< Precomputed (1.0 - alphaZ) value */
        double prevX;          /**< Previous filtered value for X-axis */
        double prevY;          /**< Previous filtered value for Y-axis */
        double prevZ;          /**< Previous filtered value for Z-axis */

        // Normalized coordinates
        double rawX; /**< Normalized X coordinate */
        double rawY; /**< Normalized Y coordinate */
        double rawZ; /**< Normalized Z coordinate */

    public:
        /**
         * Constructor to initialize the CoordinateSpaceFilter with initial coordinates
         * and an optional alpha value for the low-pass filter.
         *
         * @param x Initial X coordinate (default is 0.0).
         * @param y Initial Y coordinate (default is 0.0).
         * @param z Initial Z coordinate (default is 0.0).
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_RAW: 0
         *               - COORDINATE_SYSTEM_BASIC: 1
         *               - COORDINATE_SYSTEM_TILT: 2
         * @param alpha Alpha value for the low-pass filter.
         */
        CoordinateSpaceFilter(const double x = 0.0, const double y = 0.0, const double z = 0.0,
                              const int system = ACCELMAGIQ_DEFAULT_COORDINATE_SYSTEM,
                              const double alpha = ACCELMAGIQ_DEFAULT_LOW_PASS_FILTER_ALPHA)
            : currentSystem(system),
              alphaX(alpha), oneMinusAlphaX(1.0 - alpha),
              alphaY(alpha), oneMinusAlphaY(1.0 - alpha),
              alphaZ(alpha), oneMinusAlphaZ(1.0 - alpha),
              prevX(x), prevY(y), prevZ(z),
              rawX(0.0), rawY(0.0), rawZ(0.0)
        {
            update(x, y, z);
        }

        /**
         * Sets the coordinate system for the filter.
         *
         * @param system The coordinate system to use:
         *               - COORDINATE_SYSTEM_RAW: 0
         *               - COORDINATE_SYSTEM_BASIC: 1
         *               - COORDINATE_SYSTEM_TILT: 2
         */
        void setCoordinateSystem(const int system);

        /**
         * Gets the X coordinate based on the current coordinate system.
         *
         * @return The X coordinate value.
         */
        double getCoordX() const;

        /**
         * Gets the Y coordinate based on the current coordinate system.
         *
         * @return The Y coordinate value.
         */
        double getCoordY() const;

        /**
         * Gets the Z coordinate based on the current coordinate system.
         *
         * @return The Z coordinate value.
         */
        double getCoordZ() const;

        /**
         * Updates the coordinate values with new data.
         *
         * @param x The new X value.
         * @param y The new Y value.
         * @param z The new Z value.
         */
        void update(const double x, const double y, const double z);

        /**
         * Sets the alpha value for the low-pass filter.
         *
         * @param alpha The new alpha value.
         */
        void setAlpha(const double alpha);

    private:
        /**
         * Applies a low-pass filter to the given value.
         *
         * @param newValue The new value to filter.
         * @param oldValue The previous filtered value.
         * @param alpha The alpha value for the filter.
         * @param oneMinusAlpha Precomputed (1.0 - alpha) value.
         * @return The filtered value.
         */
        inline double lowPassFilter(double newValue, double &oldValue, double alpha, double oneMinusAlpha);
    };

} // namespace accelmagiqlib

#endif // ACCELMAGIQLIB_COORDINATESPACEFILTER_H
