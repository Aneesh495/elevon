package elevon.fcl

import std.math.*

/*
 * Structural load factor (g-protection) governor.
 * Protects wing spars and airframe from exceeding +2.5g / -1.0g limits.
 */
public class GLimiterGovernor {
    public var maxCleanPositiveG: Float64
    public var maxCleanNegativeG: Float64
    public var maxFlapsPositiveG: Float64
    public var maxFlapsNegativeG: Float64
    public var kGLimit: Float64
    public var isPositiveGLimitActive: Bool
    public var isNegativeGLimitActive: Bool

    public init() {
        this.maxCleanPositiveG = 2.5
        this.maxCleanNegativeG = -1.0
        this.maxFlapsPositiveG = 2.0
        this.maxFlapsNegativeG = 0.0
        this.kGLimit = 6.0
        this.isPositiveGLimitActive = false
        this.isNegativeGLimitActive = false
    }

    /*
     * Clamps elevator deflection command to prevent structural g-exceedance.
     */
    public func enforce(
        rawElevatorCmdDeg: Float64,
        currentNz: Float64,
        flapsExtended: Bool
    ): (Float64, Bool, Bool) {
        let posLimit = flapsExtended ? this.maxFlapsPositiveG : this.maxCleanPositiveG
        let negLimit = flapsExtended ? this.maxFlapsNegativeG : this.maxCleanNegativeG

        var limitedCmd = rawElevatorCmdDeg
        this.isPositiveGLimitActive = false
        this.isNegativeGLimitActive = false

        if (currentNz > posLimit) {
            let exceedance = currentNz - posLimit
            limitedCmd += this.kGLimit * exceedance // Push elevator down (pitch down)
            this.isPositiveGLimitActive = true
        } else if (currentNz < negLimit) {
            let exceedance = negLimit - currentNz
            limitedCmd -= this.kGLimit * exceedance // Pull elevator up (pitch up)
            this.isNegativeGLimitActive = true
        }

        return (limitedCmd, this.isPositiveGLimitActive, this.isNegativeGLimitActive)
    }
}
