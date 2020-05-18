package elevon.adirs

import std.math.*
import elevon.core.*

/*
 * Strapdown Inertial Reference System (IRS) attitude and acceleration integrator.
 */
public class InertialReferenceSystem {
    public var attitude: Quaternion
    public var bodyAngularRates: Vector3
    public var bodyAccelerations: Vector3
    public var eulerRollDeg: Float64
    public var eulerPitchDeg: Float64
    public var eulerHeadingDeg: Float64
    public var loadFactorNz: Float64
    public var isInitialized: Bool

    public static let G0: Float64 = 9.80665
    public static let RAD_TO_DEG: Float64 = 57.29577951308232

    public init() {
        this.attitude = Quaternion.identity()
        this.bodyAngularRates = Vector3.zero()
        this.bodyAccelerations = Vector3.zero()
        this.eulerRollDeg = 0.0
        this.eulerPitchDeg = 0.0
        this.eulerHeadingDeg = 0.0
        this.loadFactorNz = 1.0 // 1g straight and level
        this.isInitialized = false
    }

    public func setInitialAttitude(rollDeg: Float64, pitchDeg: Float64, headingDeg: Float64) {
        let rollRad = rollDeg / RAD_TO_DEG
        let pitchRad = pitchDeg / RAD_TO_DEG
        let headingRad = headingDeg / RAD_TO_DEG
        this.attitude = Quaternion.fromEulerAngles(rollRad, pitchRad, headingRad)
        this.eulerRollDeg = rollDeg
        this.eulerPitchDeg = pitchDeg
        this.eulerHeadingDeg = headingDeg
        this.isInitialized = true
    }

    /*
     * Integrates IMU gyro rates and accelerations over time step dtSec.
     */
    public func update(gyroRatesRadS: Vector3, accelMS2: Vector3, dtSec: Float64) {
        this.bodyAngularRates = gyroRatesRadS
        this.bodyAccelerations = accelMS2

        // Quaternion kinematic integration: q_dot = 0.5 * q * omega
        let omegaQuat = Quaternion(0.0, gyroRatesRadS.x, gyroRatesRadS.y, gyroRatesRadS.z)
        let qDot = this.attitude.multiply(omegaQuat)

        let halfDt = 0.5 * dtSec
        let updatedQ = Quaternion(
            this.attitude.w + (qDot.w * halfDt),
            this.attitude.x + (qDot.x * halfDt),
            this.attitude.y + (qDot.y * halfDt),
            this.attitude.z + (qDot.z * halfDt)
        )
        this.attitude = updatedQ.normalized()

        // Extract Euler angles
        let (rRad, pRad, yRad) = this.attitude.toEulerAngles()
        this.eulerRollDeg = rRad * RAD_TO_DEG
        this.eulerPitchDeg = pRad * RAD_TO_DEG
        this.eulerHeadingDeg = yRad * RAD_TO_DEG

        // Normal load factor Nz in g units (positive upwards along -Z body axis)
        // Standard accelerometer reads -g when sitting level on runway
        this.loadFactorNz = -accelMS2.z / G0
    }
}
