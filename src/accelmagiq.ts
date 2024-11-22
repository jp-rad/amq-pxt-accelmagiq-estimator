/**
 * A Simplified Analytic Attitude Determination Algorithm
 * using Accelerometer and Magnetometer on micro:bit.
 * 
 * It's like magic! This algorithm turns raw data into accurate and efficient quaternion estimations,
 * transforming your projects and making you go, "Wow!" Using the handy micro:bit, it feels like trying
 * out quaternions is pure magic. AccelMagiQ brings a touch of enchantment to the technical world.
 * We hope this helps you in your learning journey and sparks your curiosity about the fascinating
 * world of quaternions.
 * 
 * "AccelMagiQ" refers to this amazing algorithm that combines accelerometer and magnetometer data to
 * create precise quaternion calculations. It's a playful blend of 'acceleration', 'magnetometer', and 'magic' with
 * quaternions ('Q'), making advanced concepts seem like magic!
 */
//% block="AccelMagiQ"
//% weight=95 color=#4b0082 icon="\uf1d8"
//% groups="['Quaternion', 'EulerAngles', 'Sensor', 'Service']"
namespace accelmagiq {
    /**
     * Define coordinate system enum
     */
    export enum CoordinateSystem {
        /**
         * RAW: a raw coordinate system (North: A-button, upside-down)
         */
        //% block="RAW"
        RAW = 0,
        /**
         * BASIC: a non-tilt compensated bearing of the device (North: logo mark)
         */
        //% block="BASIC"
        BASIC = 1,
        /**
         * TILT: a tilt compensated bearing of the device (North: back side)
         */
        //% block="TILT"
        TILT = 2,
    }

    // Quaternion for simulator
    let q_ = [1.0, 0.0, 0.0, 0.0];

    //% shim=accelmagiq::estimate_
    function estimate_(): void {
        // for simulator
        let x = input.acceleration(Dimension.X);
        let y = input.acceleration(Dimension.Y);
        let z = input.acceleration(Dimension.Z);
        let norm = Math.sqrt(x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            x = x * norm;
            y = y * norm;
            z = z * norm;
        }
        const ax = y;
        const ay = x;
        const az = -z;

        let w = Math.sqrt((az + 1.0) / 2.0)
        x = ay / (2.0 * w)
        y = -ax / (2.0 * w)
        z = 0.0
        norm = Math.sqrt(w * w + x * x + y * y + z * z)
        if (0 < norm) {
            norm = 1 / norm;
            w *= norm;
            x *= norm;
            y *= norm;
            z *= norm;
            q_ = [w, x, y, z];
        }
    }

    //% shim=accelmagiq::getW_
    function getW_(): number {
        // for simulator
        return q_[0];
    }

    //% shim=accelmagiq::getX_
    function getX_(): number {
        // for simulator
        return q_[1];
    }

    //% shim=accelmagiq::getY_
    function getY_(): number {
        // for simulator
        return q_[2];
    }

    //% shim=accelmagiq::getZ_
    export function getZ_(): number {
        // for simulator
        return q_[3];
    }

    /**
     * Estimates the current quaternion.
     * @returns An array containing the quaternion components [w, x, y, z].
     */
    //% block="estimate quaternion"
    //% group="Sensor"
    //% weight=105
    export function estimate(): number[] {
        estimate_();
        return [getW_(), getX_(), getY_(), getZ_()];
    }

    /**
     * Sets the coordinate system.
     * @param system the coordinate system to use: RAW, BASIC, TILT
     */
    //% block="set coordinate system %system"
    //% group="Sensor"
    //% weight=104
    //% shim=accelmagiq::setCoordinateSystem
    export function setCoordinateSystem(system: CoordinateSystem) {
        // alpha for simulator
        console.log("accelmagiq.setCoordinateSystem()");
    }

    /**
     * Sets the alpha value for the low-pass filter.
     * @param alpha The new alpha value (between 0.0 and 1.0).
     */
    //% block="set alpha %alpha"
    //% group="Sensor"
    //% weight=103
    //% alpha.defl=0.3
    //% advanced=true
    //% shim=accelmagiq::setLowPassFilterAlpha
    export function setLowPassFilterAlpha(alpha: number): void {
        // for simulator
        console.log("accelmagiq.setLowPassFilterAlpha()");
    }

}
