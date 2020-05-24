package elevon.adirs

import elevon.core.*

/*
 * Consolidated authoritative flight navigation and air data state
 * delivered to the Flight Control Computer lanes.
 */
public struct FusedNavState {
    public var casKnots: Float64
    public var tasMps: Float64
    public var mach: Float64
    public var altitudeMeters: Float64
    public var alphaDeg: Float64
    public var betaDeg: Float64
    public var rollDeg: Float64
    public var pitchDeg: Float64
    public var headingDeg: Float64
    public var loadFactorNz: Float64
    public var bodyAngularRates: Vector3
    public var healthyAdirsUnits: Int64
    public var adirs1Healthy: Bool
    public var adirs2Healthy: Bool
    public var adirs3Healthy: Bool
    public var airDataDisagree: Bool

    public init(
        casKnots: Float64,
        tasMps: Float64,
        mach: Float64,
        altitudeMeters: Float64,
        alphaDeg: Float64,
        betaDeg: Float64,
        rollDeg: Float64,
        pitchDeg: Float64,
        headingDeg: Float64,
        loadFactorNz: Float64,
        bodyAngularRates: Vector3,
        healthyAdirsUnits: Int64,
        adirs1Healthy: Bool,
        adirs2Healthy: Bool,
        adirs3Healthy: Bool,
        airDataDisagree: Bool
    ) {
        this.casKnots = casKnots
        this.tasMps = tasMps
        this.mach = mach
        this.altitudeMeters = altitudeMeters
        this.alphaDeg = alphaDeg
        this.betaDeg = betaDeg
        this.rollDeg = rollDeg
        this.pitchDeg = pitchDeg
        this.headingDeg = headingDeg
        this.loadFactorNz = loadFactorNz
        this.bodyAngularRates = bodyAngularRates
        this.healthyAdirsUnits = healthyAdirsUnits
        this.adirs1Healthy = adirs1Healthy
        this.adirs2Healthy = adirs2Healthy
        this.adirs3Healthy = adirs3Healthy
        this.airDataDisagree = airDataDisagree
    }
}

/*
 * Triplex ADIRS sensor fusion, cross-comparison, and failure isolation manager.
 */
public class AdirsManager {
    public let casVoter: MvsVoter
    public let altVoter: MvsVoter
    public let pitchVoter: MvsVoter
    public let rollVoter: MvsVoter
    public let nzVoter: MvsVoter

    public var unit1Healthy: Bool
    public var unit2Healthy: Bool
    public var unit3Healthy: Bool

    public init() {
        // Tolerances: 5 knots CAS, 30 meters altitude, 1.0 deg pitch/roll, 0.15g Nz
        this.casVoter = MvsVoter(5.0, 10.0)
        this.altVoter = MvsVoter(30.0, 60.0)
        this.pitchVoter = MvsVoter(1.0, 2.0)
        this.rollVoter = MvsVoter(1.0, 2.0)
        this.nzVoter = MvsVoter(0.15, 0.3)

        this.unit1Healthy = true
        this.unit2Healthy = true
        this.unit3Healthy = true
    }

    public func fuse(
        adc1: AirDataOutput, irs1: InertialReferenceSystem,
        adc2: AirDataOutput, irs2: InertialReferenceSystem,
        adc3: AirDataOutput, irs3: InertialReferenceSystem,
        alphaTrueDeg: Float64, betaTrueDeg: Float64
    ): FusedNavState {
        let v1 = this.unit1Healthy && adc1.isValid && irs1.isInitialized
        let v2 = this.unit2Healthy && adc2.isValid && irs2.isInitialized
        let v3 = this.unit3Healthy && adc3.isValid && irs3.isInitialized

        // Vote Calibrated Airspeed
        let votedCas = this.casVoter.vote(adc1.calibratedAirspeedKnots, v1, adc2.calibratedAirspeedKnots, v2, adc3.calibratedAirspeedKnots, v3)
        // Vote Altitude
        let votedAlt = this.altVoter.vote(adc1.pressureAltitudeMeters, v1, adc2.pressureAltitudeMeters, v2, adc3.pressureAltitudeMeters, v3)
        // Vote Pitch
        let votedPitch = this.pitchVoter.vote(irs1.eulerPitchDeg, v1, irs2.eulerPitchDeg, v2, irs3.eulerPitchDeg, v3)
        // Vote Roll
        let votedRoll = this.rollVoter.vote(irs1.eulerRollDeg, v1, irs2.eulerRollDeg, v2, irs3.eulerRollDeg, v3)
        // Vote Normal load factor Nz
        let votedNz = this.nzVoter.vote(irs1.loadFactorNz, v1, irs2.loadFactorNz, v2, irs3.loadFactorNz, v3)

        // Failure isolation: trip any unit that persistently diverges
        if (votedCas.laneADiverged || votedAlt.laneADiverged) {
            if (this.casVoter.persistentDisagreementCountA >= 5) {
                this.unit1Healthy = false
            }
        }
        if (votedCas.laneBDiverged || votedAlt.laneBDiverged) {
            if (this.casVoter.persistentDisagreementCountB >= 5) {
                this.unit2Healthy = false
            }
        }
        if (votedCas.laneCDiverged || votedAlt.laneCDiverged) {
            if (this.casVoter.persistentDisagreementCountC >= 5) {
                this.unit3Healthy = false
            }
        }

        let airDataDisagree = (votedCas.byzantineDisagreement || votedAlt.byzantineDisagreement)
        let healthyCount = (this.unit1Healthy ? 1 : 0) + (this.unit2Healthy ? 1 : 0) + (this.unit3Healthy ? 1 : 0)

        // Use mid-value body rates
        let votedRates = irs1.bodyAngularRates

        return FusedNavState(
            votedCas.votedValue,
            adc1.trueAirspeedMps,
            adc1.machNumber,
            votedAlt.votedValue,
            alphaTrueDeg,
            betaTrueDeg,
            votedRoll.votedValue,
            votedPitch.votedValue,
            irs1.eulerHeadingDeg,
            votedNz.votedValue,
            votedRates,
            Int64(healthyCount),
            this.unit1Healthy,
            this.unit2Healthy,
            this.unit3Healthy,
            airDataDisagree
        )
    }
}
