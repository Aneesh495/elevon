package elevon.adirs

import std.math.*

/*
 * Correction result for aerodynamic angle of attack and sideslip vanes.
 */
public struct AerodynamicAngles {
    public var alphaTrueDeg: Float64
    public var betaTrueDeg: Float64
    public var leftVaneValid: Bool
    public var rightVaneValid: Bool
    public var vaneDisagree: Bool

    public init(
        alphaTrueDeg: Float64,
        betaTrueDeg: Float64,
        leftVaneValid: Bool,
        rightVaneValid: Bool,
        vaneDisagree: Bool
    ) {
        this.alphaTrueDeg = alphaTrueDeg
        this.betaTrueDeg = betaTrueDeg
        this.leftVaneValid = leftVaneValid
        this.rightVaneValid = rightVaneValid
        this.vaneDisagree = vaneDisagree
    }
}

/*
 * Aerodynamic position error correction and dual-vane cross-checking engine.
 */
public class VaneCorrector {
    public static let VANE_ARM_METERS: Float64 = 14.5 // Distance from CG to nose vanes
    public static let RAD_TO_DEG: Float64 = 57.29577951308232

    /*
     * Compensates local vane readings for fuselage upwash, dynamic pitch rate, and Mach effects.
     */
    public static func correctAngles(
        alphaLeftLocalDeg: Float64, validLeft: Bool,
        alphaRightLocalDeg: Float64, validRight: Bool,
        mach: Float64,
        pitchRateRadS: Float64,
        tasMps: Float64,
        flapsExtended: Bool
    ): AerodynamicAngles {
        // Dynamic pitch-rate induced alpha correction: delta_alpha = (L_arm * q) / V_tas
        var pitchRateCorrDeg: Float64 = 0.0
        if (tasMps > 20.0) {
            let inducedAlphaRad = (VANE_ARM_METERS * pitchRateRadS) / tasMps
            pitchRateCorrDeg = inducedAlphaRad * RAD_TO_DEG
        }

        let flapOffsetDeg = flapsExtended ? -1.2 : 0.0
        let machCorrection = -0.45 * (mach > 0.4 ? (mach - 0.4) : 0.0)

        // Case 1: Both left and right vanes valid
        if (validLeft && validRight) {
            let diff = abs(alphaLeftLocalDeg - alphaRightLocalDeg)
            let disagree = (diff > 3.0) // 3 degree disagreement threshold

            // Symmetrical average eliminates sideslip cross-coupling
            let alphaRawAvg = (alphaLeftLocalDeg + alphaRightLocalDeg) * 0.5
            let alphaTrue = (alphaRawAvg * 0.94) + flapOffsetDeg + machCorrection - pitchRateCorrDeg

            // Differential vane angle estimates sideslip beta
            let betaEstimate = (alphaLeftLocalDeg - alphaRightLocalDeg) * 0.65

            return AerodynamicAngles(alphaTrue, betaEstimate, true, true, disagree)
        }

        // Case 2: Left vane only
        if (validLeft) {
            let alphaTrue = (alphaLeftLocalDeg * 0.94) + flapOffsetDeg + machCorrection - pitchRateCorrDeg
            return AerodynamicAngles(alphaTrue, 0.0, true, false, false)
        }

        // Case 3: Right vane only
        if (validRight) {
            let alphaTrue = (alphaRightLocalDeg * 0.94) + flapOffsetDeg + machCorrection - pitchRateCorrDeg
            return AerodynamicAngles(alphaTrue, 0.0, false, true, false)
        }

        // Case 4: No valid vanes
        return AerodynamicAngles(0.0, 0.0, false, false, true)
    }
}
