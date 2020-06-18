package elevon.fcl

import std.math.*

/*
 * High-Angle-of-Attack (AOA) flight envelope protection governor.
 * Prevents aerodynamic stall via alpha-protection, alpha-floor trigger, and alpha-max clamp.
 */
public class AlphaProtectionGovernor {
    public var alphaProtDeg: Float64
    public var alphaFloorDeg: Float64
    public var alphaMaxDeg: Float64
    public var alphaStallDeg: Float64
    public var kAlpha: Float64

    public var isAlphaProtActive: Bool
    public var isAlphaFloorTriggered: Bool

    public init() {
        this.alphaProtDeg = 11.2
        this.alphaFloorDeg = 12.8
        this.alphaMaxDeg = 15.0
        this.alphaStallDeg = 14.5
        this.kAlpha = 4.5
        this.isAlphaProtActive = false
        this.isAlphaFloorTriggered = false
    }

    /*
     * Evaluates high-AOA protection and generates nose-down elevator override.
     */
    public func evaluate(currentAlphaDeg: Float64, pilotPitchStickNorm: Float64): (Float64, Bool, Bool) {
        // Trigger alpha floor if alpha exceeds 12.8 deg
        if (currentAlphaDeg >= this.alphaFloorDeg) {
            this.isAlphaFloorTriggered = true
        } else if (currentAlphaDeg < (this.alphaProtDeg - 1.0)) {
            this.isAlphaFloorTriggered = false
        }

        // Check if alpha protection is engaged
        if (currentAlphaDeg >= this.alphaProtDeg) {
            this.isAlphaProtActive = true

            // Stick demand now requests alpha between alphaProt and alphaMax
            let commandedAlpha = this.alphaProtDeg + (pilotPitchStickNorm > 0.0 ? (pilotPitchStickNorm * (this.maxAlphaDeg() - this.alphaProtDeg)) : 0.0)
            let alphaExceedance = currentAlphaDeg - commandedAlpha

            // If actual alpha exceeds target, apply strong nose-down elevator command
            var elevatorOverrideDeg: Float64 = 0.0
            if (alphaExceedance > 0.0) {
                elevatorOverrideDeg = this.kAlpha * alphaExceedance
            }

            return (elevatorOverrideDeg, true, this.isAlphaFloorTriggered)
        } else {
            this.isAlphaProtActive = false
            return (0.0, false, this.isAlphaFloorTriggered)
        }
    }

    public func maxAlphaDeg(): Float64 {
        return this.alphaMaxDeg
    }
}
