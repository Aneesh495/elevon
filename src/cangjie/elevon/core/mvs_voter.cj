package elevon.core

/*
 * Mid-Value Selection (MVS) Voting result structure containing the voted scalar,
 * active voting tier, and per-channel divergence flags.
 */
public struct MvsResult {
    public var votedValue: Float64
    public var healthyChannelCount: Int64
    public var laneADiverged: Bool
    public var laneBDiverged: Bool
    public var laneCDiverged: Bool
    public var byzantineDisagreement: Bool

    public init(
        votedValue: Float64,
        healthyChannelCount: Int64,
        laneADiverged: Bool,
        laneBDiverged: Bool,
        laneCDiverged: Bool,
        byzantineDisagreement: Bool
    ) {
        this.votedValue = votedValue
        this.healthyChannelCount = healthyChannelCount
        this.laneADiverged = laneADiverged
        this.laneBDiverged = laneBDiverged
        this.laneCDiverged = laneCDiverged
        this.byzantineDisagreement = byzantineDisagreement
    }
}

/*
 * Mid-Value Selection (MVS) voter for triplex redundant analog signals.
 */
public class MvsVoter {
    public var thresholdTriplex: Float64
    public var thresholdDuplex: Float64
    public var persistentDisagreementCountA: UInt32
    public var persistentDisagreementCountB: UInt32
    public var persistentDisagreementCountC: UInt32
    public let persistLimit: UInt32

    public init(thresholdTriplex: Float64, thresholdDuplex: Float64) {
        this.thresholdTriplex = thresholdTriplex
        this.thresholdDuplex = thresholdDuplex
        this.persistentDisagreementCountA = 0
        this.persistentDisagreementCountB = 0
        this.persistentDisagreementCountC = 0
        this.persistLimit = 5 // Trips channel after 5 consecutive minor cycles
    }

    private func absDiff(a: Float64, b: Float64): Float64 {
        return (a >= b) ? (a - b) : (b - a)
    }

    /*
     * Core MVS voting computation across three input channels.
     */
    public func vote(
        valA: Float64, validA: Bool,
        valB: Float64, validB: Bool,
        valC: Float64, validC: Bool
    ): MvsResult {
        // Case 1: All three channels are valid
        if (validA && validB && validC) {
            let diffAB = this.absDiff(valA, valB)
            let diffBC = this.absDiff(valB, valC)
            let diffCA = this.absDiff(valC, valA)

            var divA = false
            var divB = false
            var divC = false

            // Check if one channel is an outlier
            if (diffAB > this.thresholdTriplex && diffCA > this.thresholdTriplex && diffBC <= this.thresholdTriplex) {
                divA = true
                this.persistentDisagreementCountA += 1
            } else {
                this.persistentDisagreementCountA = 0
            }

            if (diffAB > this.thresholdTriplex && diffBC > this.thresholdTriplex && diffCA <= this.thresholdTriplex) {
                divB = true
                this.persistentDisagreementCountB += 1
            } else {
                this.persistentDisagreementCountB = 0
            }

            if (diffBC > this.thresholdTriplex && diffCA > this.thresholdTriplex && diffAB <= this.thresholdTriplex) {
                divC = true
                this.persistentDisagreementCountC += 1
            } else {
                this.persistentDisagreementCountC = 0
            }

            let byzantine = (diffAB > this.thresholdTriplex && diffBC > this.thresholdTriplex && diffCA > this.thresholdTriplex)

            // Select median value
            var median = valA
            if ((valA <= valB && valB <= valC) || (valC <= valB && valB <= valA)) {
                median = valB
            } else if ((valB <= valA && valA <= valC) || (valC <= valA && valA <= valB)) {
                median = valA
            } else {
                median = valC
            }

            return MvsResult(median, 3, divA, divB, divC, byzantine)
        }

        // Case 2: Exactly two channels valid (duplex cross-check)
        if (validA && validB && !validC) {
            let diff = this.absDiff(valA, valB)
            let byzantine = diff > this.thresholdDuplex
            return MvsResult((valA + valB) * 0.5, 2, false, false, true, byzantine)
        }
        if (validB && validC && !validA) {
            let diff = this.absDiff(valB, valC)
            let byzantine = diff > this.thresholdDuplex
            return MvsResult((valB + valC) * 0.5, 2, true, false, false, byzantine)
        }
        if (validC && validA && !validB) {
            let diff = this.absDiff(valC, valA)
            let byzantine = diff > this.thresholdDuplex
            return MvsResult((valC + valA) * 0.5, 2, false, true, false, byzantine)
        }

        // Case 3: Exactly one channel valid (simplex fallback)
        if (validA) {
            return MvsResult(valA, 1, false, true, true, false)
        }
        if (validB) {
            return MvsResult(valB, 1, true, false, true, false)
        }
        if (validC) {
            return MvsResult(valC, 1, true, true, false, false)
        }

        // Case 4: No valid channels (total loss)
        return MvsResult(0.0, 0, true, true, true, true)
    }
}
