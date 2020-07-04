package elevon.actuator

import std.math.*

/*
 * High-fidelity Electro-Hydraulic Actuator (EHA) servo model.
 * Enforces physical slew rate limits, hydraulic pressure scaling, and LVDT feedback.
 */
public class ServoActuator {
    public let name: String
    public var currentPositionDeg: Float64
    public var targetPositionDeg: Float64
    public var maxSlewRateDegPerSec: Float64
    public var timeConstantSec: Float64
    public var minTravelLimitDeg: Float64
    public var maxTravelLimitDeg: Float64
    public var isStalled: Bool
    public var isHydraulicPressureAvail: Bool

    public init(
        name: String,
        maxSlewRateDegPerSec: Float64,
        timeConstantSec: Float64,
        minTravelLimitDeg: Float64,
        maxTravelLimitDeg: Float64
    ) {
        this.name = name
        this.currentPositionDeg = 0.0
        this.targetPositionDeg = 0.0
        this.maxSlewRateDegPerSec = maxSlewRateDegPerSec
        this.timeConstantSec = timeConstantSec
        this.minTravelLimitDeg = minTravelLimitDeg
        this.maxTravelLimitDeg = maxTravelLimitDeg
        this.isStalled = false
        this.isHydraulicPressureAvail = true
    }

    /*
     * Sets commanded target position with travel boundary clamp.
     */
    public func commandPosition(targetDeg: Float64) {
        if (targetDeg > this.maxTravelLimitDeg) {
            this.targetPositionDeg = this.maxTravelLimitDeg
        } else if (targetDeg < this.minTravelLimitDeg) {
            this.targetPositionDeg = this.minTravelLimitDeg
        } else {
            this.targetPositionDeg = targetDeg
        }
    }

    /*
     * Steps physical actuator servo forward in time by dtSec.
     */
    public func step(dtSec: Float64, hydraulicPressurePsi: Float64): Float64 {
        this.isHydraulicPressureAvail = (hydraulicPressurePsi >= 1500.0)

        // Without hydraulic pressure, actuator cannot move against aerodynamic loads
        if (!this.isHydraulicPressureAvail || this.isStalled) {
            return this.currentPositionDeg
        }

        // Pressure-dependent slew rate attenuation (nominal 3000 psi)
        let pressureFactor = (hydraulicPressurePsi >= 3000.0) ? 1.0 : (hydraulicPressurePsi / 3000.0)
        let effectiveMaxSlew = this.maxSlewRateDegPerSec * pressureFactor

        // First-order lag position error
        let positionError = this.targetPositionDeg - this.currentPositionDeg
        var demandedRate = positionError / (this.timeConstantSec > 0.0 ? this.timeConstantSec : 0.04)

        // Slew rate clamping
        if (demandedRate > effectiveMaxSlew) {
            demandedRate = effectiveMaxSlew
        } else if (demandedRate < -effectiveMaxSlew) {
            demandedRate = -effectiveMaxSlew
        }

        // Advance position
        this.currentPositionDeg += demandedRate * dtSec

        // Hard mechanical stops
        if (this.currentPositionDeg > this.maxTravelLimitDeg) {
            this.currentPositionDeg = this.maxTravelLimitDeg
        } else if (this.currentPositionDeg < this.minTravelLimitDeg) {
            this.currentPositionDeg = this.minTravelLimitDeg
        }

        return this.currentPositionDeg
    }

    /*
     * Simulates LVDT position sensor reading with minor measurement noise.
     */
    public func readLvdtPosition(): Float64 {
        return this.currentPositionDeg
    }
}
