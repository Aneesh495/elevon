package elevon.fcl

import std.math.*

/*
 * High-Speed flight envelope protection governor (VMO / MMO).
 * Automatically commands positive pitch-up and inhibits nose-down trim.
 */
public class HighSpeedProtectionGovernor {
    public var vMoKnots: Float64
    public var mMoMach: Float64
    public var vTriggerKnots: Float64
    public var mTriggerMach: Float64
    public var kSpeedPitchUp: Float64
    public var isHighSpeedProtActive: Bool

    public init() {
        this.vMoKnots = 340.0
        this.mMoMach = 0.82
        this.vTriggerKnots = 346.0 // VMO + 6 knots
        this.mTriggerMach = 0.83   // MMO + 0.01
        this.kSpeedPitchUp = 0.85  // deg elevator per knot exceedance
        this.isHighSpeedProtActive = false
    }

    /*
     * Evaluates high-speed boundary and generates nose-up pitch override.
     */
    public func evaluate(
        currentCasKnots: Float64,
        currentMach: Float64,
        pilotPitchStickNorm: Float64
    ): (Float64, Bool) {
        let casExceedance = currentCasKnots - this.vTriggerKnots
        let machExceedance = (currentMach - this.mTriggerMach) * 500.0 // Scaled to equivalent knots

        let worstExceedance = (casExceedance > machExceedance) ? casExceedance : machExceedance

        if (worstExceedance > 0.0) {
            this.isHighSpeedProtActive = true

            // Negative elevator deflection corresponds to nose-up pitch
            var pitchUpOverrideDeg = -this.kSpeedPitchUp * worstExceedance

            // Attenuate pilot forward stick command (cannot push nose down past VMO)
            if (pilotPitchStickNorm < 0.0) {
                pitchUpOverrideDeg += pilotPitchStickNorm * 5.0
            }

            return (pitchUpOverrideDeg, true)
        } else {
            this.isHighSpeedProtActive = false
            return (0.0, false)
        }
    }
}
