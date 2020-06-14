package elevon.fcl

import std.math.*

/*
 * Directional flight control law providing automatic Dutch roll damping,
 * turn coordination, and pilot rudder pedal scheduling.
 */
public class DirectionalFlightLaw {
    public var washoutTimeConstantSec: Float64
    public var washoutState: Float64
    public var previousYawRateRadS: Float64
    public var yawDamperGain: Float64
    public var turnCoordGain: Float64
    public var maxRudderDeflectionDeg: Float64

    public static let G0: Float64 = 9.80665
    public static let DEG_TO_RAD: Float64 = 0.017453292519943295
    public static let RAD_TO_DEG: Float64 = 57.29577951308232

    public init() {
        this.washoutTimeConstantSec = 2.5
        this.washoutState = 0.0
        this.previousYawRateRadS = 0.0
        this.yawDamperGain = 1.85
        this.turnCoordGain = 1.0
        this.maxRudderDeflectionDeg = 30.0
    }

    /*
     * Washout filter step: y[k] = (tau / (tau + dt)) * (y[k-1] + r[k] - r[k-1])
     */
    public func filterWashout(yawRateRadS: Float64, dtSec: Float64): Float64 {
        let alpha = this.washoutTimeConstantSec / (this.washoutTimeConstantSec + dtSec)
        let deltaR = yawRateRadS - this.previousYawRateRadS
        this.washoutState = alpha * (this.washoutState + deltaR)
        this.previousYawRateRadS = yawRateRadS
        return this.washoutState
    }

    /*
     * Calculates kinematic turn coordination yaw rate:
     * r_coord = (g0 / V_tas) * sin(bank) * cos(pitch)
     */
    public func computeTurnCoordinationRate(
        bankDeg: Float64,
        pitchDeg: Float64,
        tasMps: Float64
    ): Float64 {
        if (tasMps < 25.0) {
            return 0.0
        }
        let bankRad = bankDeg * DEG_TO_RAD
        let pitchRad = pitchDeg * DEG_TO_RAD
        return (G0 / tasMps) * sin(bankRad) * cos(pitchRad)
    }

    /*
     * Computes commanded rudder deflection in degrees.
     */
    public func step(
        rudderPedalNorm: Float64,
        yawRateRadS: Float64,
        bankDeg: Float64,
        pitchDeg: Float64,
        tasMps: Float64,
        dtSec: Float64
    ): Float64 {
        // 1. Yaw damper: suppress oscillatory yaw rate
        let washedOutYawRate = this.filterWashout(yawRateRadS, dtSec)
        let yawDamperCmdDeg = -this.yawDamperGain * (washedOutYawRate * RAD_TO_DEG)

        // 2. Turn coordination: automatic rudder to zero sideslip in bank
        let rCoordRadS = this.computeTurnCoordinationRate(bankDeg, pitchDeg, tasMps)
        let coordErrorRadS = rCoordRadS - yawRateRadS
        let turnCoordCmdDeg = this.turnCoordGain * (coordErrorRadS * RAD_TO_DEG)

        // 3. Pilot pedal authority (dynamically scheduled by airspeed)
        let pedalSensitivity = (tasMps > 100.0) ? (100.0 / tasMps) : 1.0
        let pilotPedalCmdDeg = rudderPedalNorm * this.maxRudderDeflectionDeg * pedalSensitivity

        var totalRudder = pilotPedalCmdDeg + yawDamperCmdDeg + turnCoordCmdDeg
        if (totalRudder > this.maxRudderDeflectionDeg) {
            totalRudder = this.maxRudderDeflectionDeg
        } else if (totalRudder < -this.maxRudderDeflectionDeg) {
            totalRudder = -this.maxRudderDeflectionDeg
        }

        return totalRudder
    }
}
