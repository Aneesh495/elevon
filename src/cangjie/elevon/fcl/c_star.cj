package elevon.fcl

import std.math.*
import elevon.core.*
import elevon.adirs.*

/*
 * C* (C-Star) longitudinal flight control law.
 * Blends pitch rate q and normal load factor Nz across the airspeed envelope.
 */
public class CStarController {
    public var kp: Float64
    public var ki: Float64
    public var integratorState: Float64
    public var crossoverSpeedKnots: Float64
    public var maxElevatorDeflectionDeg: Float64

    public static let G0: Float64 = 9.80665
    public static let KNOTS_TO_MPS: Float64 = 0.51444444

    public init(kp: Float64, ki: Float64, crossoverSpeedKnots: Float64) {
        this.kp = kp
        this.ki = ki
        this.integratorState = 0.0
        this.crossoverSpeedKnots = crossoverSpeedKnots
        this.maxElevatorDeflectionDeg = 30.0
    }

    public static func defaultController(): CStarController {
        return CStarController(8.5, 3.2, 210.0)
    }

    public func reset() {
        this.integratorState = 0.0
    }

    /*
     * Calculates the blended C* feedback parameter:
     * C*_fb = (Nz - 1.0) + (V_co / g0) * q
     */
    public func computeCStarFeedback(nzLoadFactor: Float64, pitchRateRadS: Float64): Float64 {
        let vCoMps = this.crossoverSpeedKnots * KNOTS_TO_MPS
        let velocityFactor = vCoMps / G0
        let deltaNz = nzLoadFactor - 1.0 // Excess g over 1g trim
        return deltaNz + (velocityFactor * pitchRateRadS)
    }

    /*
     * Translates normalized pilot pitch stick (-1.0 to +1.0) into demanded C* (g-equivalent).
     * Full aft stick (+1.0) demands +2.5g (delta +1.5g).
     * Full forward stick (-1.0) demands -1.0g (delta -2.0g).
     */
    public func computeCStarDemand(pitchStickNorm: Float64): Float64 {
        if (pitchStickNorm >= 0.0) {
            return pitchStickNorm * 1.5 // Up to +1.5g excess
        } else {
            return pitchStickNorm * 2.0 // Down to -2.0g excess
        }
    }

    /*
     * Updates C* controller step over dtSec, producing commanded elevator deflection in degrees.
     */
    public func step(
        pitchStickNorm: Float64,
        nzLoadFactor: Float64,
        pitchRateRadS: Float64,
        dtSec: Float64
    ): Float64 {
        let cStarDem = this.computeCStarDemand(pitchStickNorm)
        let cStarFb = this.computeCStarFeedback(nzLoadFactor, pitchRateRadS)
        let error = cStarDem - cStarFb

        // Proportional term
        let pTerm = this.kp * error

        // Anti-windup clamping on integral term
        this.integratorState += this.ki * error * dtSec
        if (this.integratorState > this.maxElevatorDeflectionDeg) {
            this.integratorState = this.maxElevatorDeflectionDeg
        } else if (this.integratorState < -this.maxElevatorDeflectionDeg) {
            this.integratorState = -this.maxElevatorDeflectionDeg
        }

        var totalDemand = pTerm + this.integratorState
        if (totalDemand > this.maxElevatorDeflectionDeg) {
            totalDemand = this.maxElevatorDeflectionDeg
        } else if (totalDemand < -this.maxElevatorDeflectionDeg) {
            totalDemand = -this.maxElevatorDeflectionDeg
        }

        return totalDemand
    }
}
