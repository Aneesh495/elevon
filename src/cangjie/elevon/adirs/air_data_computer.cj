package elevon.adirs

import std.math.*
import elevon.core.*

/*
 * Consolidated Air Data Computer (ADC) output state.
 */
public struct AirDataOutput {
    public var staticPressurePa: Float64
    public var totalPressurePa: Float64
    public var impactPressurePa: Float64
    public var pressureAltitudeMeters: Float64
    public var calibratedAirspeedMps: Float64
    public var calibratedAirspeedKnots: Float64
    public var trueAirspeedMps: Float64
    public var machNumber: Float64
    public var staticAirTempKelvin: Float64
    public var dynamicPressurePa: Float64
    public var isValid: Bool

    public init(
        staticPressurePa: Float64,
        totalPressurePa: Float64,
        impactPressurePa: Float64,
        pressureAltitudeMeters: Float64,
        calibratedAirspeedMps: Float64,
        calibratedAirspeedKnots: Float64,
        trueAirspeedMps: Float64,
        machNumber: Float64,
        staticAirTempKelvin: Float64,
        dynamicPressurePa: Float64,
        isValid: Bool
    ) {
        this.staticPressurePa = staticPressurePa
        this.totalPressurePa = totalPressurePa
        this.impactPressurePa = impactPressurePa
        this.pressureAltitudeMeters = pressureAltitudeMeters
        this.calibratedAirspeedMps = calibratedAirspeedMps
        this.calibratedAirspeedKnots = calibratedAirspeedKnots
        this.trueAirspeedMps = trueAirspeedMps
        this.machNumber = machNumber
        this.staticAirTempKelvin = staticAirTempKelvin
        this.dynamicPressurePa = dynamicPressurePa
        this.isValid = isValid
    }
}

/*
 * Air Data Computer (ADC) core calculation pipeline.
 */
public class AirDataComputer {
    public static let RECOVERY_FACTOR: Float64 = 0.98

    /*
     * Executes ADC pipeline translating raw transducer inputs into calibrated air data.
     */
    public static func compute(
        pStaticPa: Float64,
        pTotalPa: Float64,
        tatKelvin: Float64
    ): AirDataOutput {
        let valid = PitotStaticCalculator.validatePressures(pStaticPa, pTotalPa)
        if (!valid) {
            return AirDataOutput(pStaticPa, pTotalPa, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, tatKelvin, 0.0, false)
        }

        let qc = (pTotalPa >= pStaticPa) ? (pTotalPa - pStaticPa) : 0.0
        let altMeters = IsaAtmosphere.altitudeFromPressure(pStaticPa)
        let casMps = PitotStaticCalculator.computeCas(qc)
        let casKnots = PitotStaticCalculator.mpsToKnots(casMps)

        // Subsonic Mach calculation: M = sqrt(5 * ((Pt/Ps)^(2/7) - 1))
        var mach: Float64 = 0.0
        if (pStaticPa > 0.0 && pTotalPa > pStaticPa) {
            let pressureRatio = pTotalPa / pStaticPa
            let term = pow(pressureRatio, 2.0 / 7.0) - 1.0
            if (term > 0.0) {
                mach = sqrt(5.0 * term)
            }
        }

        // Static Air Temperature: SAT = TAT / (1 + r * ((gamma - 1) / 2) * M^2)
        let tempDivisor = 1.0 + (RECOVERY_FACTOR * 0.2 * mach * mach)
        let satKelvin = (tatKelvin > 100.0) ? (tatKelvin / tempDivisor) : 288.15

        // Speed of sound at SAT: a = sqrt(gamma * R * SAT)
        let aMps = IsaAtmosphere.speedOfSound(satKelvin)
        let tasMps = mach * aMps

        // Dynamic pressure: qbar = 0.5 * gamma * Ps * M^2
        let qbarPa = 0.7 * pStaticPa * mach * mach

        return AirDataOutput(
            pStaticPa,
            pTotalPa,
            qc,
            altMeters,
            casMps,
            casKnots,
            tasMps,
            mach,
            satKelvin,
            qbarPa,
            true
        )
    }
}
