package elevon.fcl

import std.math.*

/*
 * Pitch and bank attitude hard flight envelope limiters.
 * Enforces -15 to +30 deg pitch and maximum 67 deg bank angle limits.
 */
public class AttitudeLimiter {
    public var maxPitchNoseUpDeg: Float64
    public var maxPitchNoseDownDeg: Float64
    public var maxBankAngleDeg: Float64
    public var pitchDamperGain: Float64
    public var rollLimiterGain: Float64

    public init() {
        this.maxPitchNoseUpDeg = 30.0
        this.maxPitchNoseDownDeg = -15.0
        this.maxBankAngleDeg = 67.0
        this.pitchDamperGain = 2.0
        this.rollLimiterGain = 1.5
    }

    /*
     * Limits pitch command to preserve pitch attitude boundary.
     */
    public func limitPitchElevator(
        rawElevatorCmdDeg: Float64,
        currentPitchDeg: Float64,
        pitchRateRadS: Float64
    ): (Float64, Bool) {
        var limitedCmd = rawElevatorCmdDeg
        var limitActive = false

        // Nose-up boundary check (+30 deg)
        if (currentPitchDeg >= this.maxPitchNoseUpDeg && rawElevatorCmdDeg < 0.0) {
            // Trailing edge up elevator would pitch nose higher; override to zero or positive
            limitedCmd = this.pitchDamperGain * (currentPitchDeg - this.maxPitchNoseUpDeg)
            limitActive = true
        } else if (currentPitchDeg <= this.maxPitchNoseDownDeg && rawElevatorCmdDeg > 0.0) {
            // Nose-down boundary check (-15 deg); override to negative elevator (pitch up)
            limitedCmd = -this.pitchDamperGain * (this.maxPitchNoseDownDeg - currentPitchDeg)
            limitActive = true
        }

        return (limitedCmd, limitActive)
    }

    /*
     * Limits lateral aileron command to preserve bank angle boundary (67 deg).
     */
    public func limitBankAileron(
        rawAileronCmdDeg: Float64,
        currentBankDeg: Float64
    ): (Float64, Bool) {
        var limitedCmd = rawAileronCmdDeg
        var limitActive = false

        if (currentBankDeg >= this.maxBankAngleDeg && rawAileronCmdDeg > 0.0) {
            limitedCmd = 0.0
            limitActive = true
        } else if (currentBankDeg <= -this.maxBankAngleDeg && rawAileronCmdDeg < 0.0) {
            limitedCmd = 0.0
            limitActive = true
        }

        return (limitedCmd, limitActive)
    }
}
