package elevon.fcl

import std.math.*

/*
 * Lateral roll rate demand and bank angle attitude hold controller.
 */
public class LateralFlightLaw {
    public var maxRollRateDegS: Float64
    public var kpRoll: Float64
    public var kdRoll: Float64
    public var spiralRestoreGain: Float64
    public var maxAileronDeflectionDeg: Float64

    public init() {
        this.maxRollRateDegS = 15.0     // Maximum roll rate in Normal Law (deg/s)
        this.kpRoll = 1.45              // Roll rate tracking gain
        this.kdRoll = 0.12              // Roll acceleration damping gain
        this.spiralRestoreGain = 0.35   // Positive spiral stability restore gain past 33 deg
        this.maxAileronDeflectionDeg = 25.0
    }

    /*
     * Calculates commanded differential aileron deflection in degrees.
     */
    public func step(
        rollStickNorm: Float64,
        currentBankDeg: Float64,
        currentRollRateDegS: Float64,
        dtSec: Float64
    ): Float64 {
        var demandedRollRate = rollStickNorm * this.maxRollRateDegS

        // Spiral stability restoration when stick is released past 33 deg bank
        if (abs(rollStickNorm) < 0.05) {
            if (currentBankDeg > 33.0) {
                demandedRollRate = -this.spiralRestoreGain * (currentBankDeg - 33.0)
            } else if (currentBankDeg < -33.0) {
                demandedRollRate = -this.spiralRestoreGain * (currentBankDeg + 33.0)
            } else {
                demandedRollRate = 0.0 // True bank angle hold
            }
        }

        let rollRateError = demandedRollRate - currentRollRateDegS
        var aileronCmd = (this.kpRoll * rollRateError) - (this.kdRoll * (currentRollRateDegS / (dtSec > 0.0 ? dtSec : 0.01)))

        // Clamp to mechanical aileron limits
        if (aileronCmd > this.maxAileronDeflectionDeg) {
            aileronCmd = this.maxAileronDeflectionDeg
        } else if (aileronCmd < -this.maxAileronDeflectionDeg) {
            aileronCmd = -this.maxAileronDeflectionDeg
        }

        return aileronCmd
    }
}
