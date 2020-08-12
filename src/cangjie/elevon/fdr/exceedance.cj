package elevon.fdr

import std.collection.*
import elevon.core.*
import elevon.adirs.*
import elevon.fcl.*

/*
 * Classification of safety-critical flight exceedance events.
 */
public enum ExceedanceType {
    | ExcessiveBankAngle
    | ExtremePitchAttitude
    | HighLoadFactor
    | StallWarningActive
    | OverspeedVmoMmo
    | ExcessiveSinkRate
    | TmrLaneTrip

    public func toString(): String {
        match (this) {
            case ExcessiveBankAngle => "ExcessiveBankAngle"
            case ExtremePitchAttitude => "ExtremePitchAttitude"
            case HighLoadFactor => "HighLoadFactor"
            case StallWarningActive => "StallWarningActive"
            case OverspeedVmoMmo => "OverspeedVmoMmo"
            case ExcessiveSinkRate => "ExcessiveSinkRate"
            case TmrLaneTrip => "TmrLaneTrip"
        }
    }
}

/*
 * Snapshot event descriptor captured for safety incident investigation.
 */
public struct ExceedanceEvent {
    public var eventType: ExceedanceType
    public var timestampUs: UInt64
    public var triggerValue: Float64
    public var thresholdValue: Float64
    public var snapshotRecordCount: Int64

    public init(
        eventType: ExceedanceType,
        timestampUs: UInt64,
        triggerValue: Float64,
        thresholdValue: Float64,
        snapshotRecordCount: Int64
    ) {
        this.eventType = eventType
        this.timestampUs = timestampUs
        this.triggerValue = triggerValue
        this.thresholdValue = thresholdValue
        this.snapshotRecordCount = snapshotRecordCount
    }
}

/*
 * Automated exceedance detection and snapshot capture engine.
 */
public class ExceedanceDetector {
    public var totalExceedancesDetected: UInt64
    public var events: ArrayList<ExceedanceEvent>

    public init() {
        this.totalExceedancesDetected = 0
        this.events = ArrayList<ExceedanceEvent>()
    }

    /*
     * Evaluates instantaneous flight state against safety thresholds.
     */
    public func evaluate(
        token: TimeToken,
        nav: FusedNavState,
        cmds: FlightLawCommands,
        votedDemands: VotedSurfaceDemands
    ): Option<ExceedanceEvent> {
        // 1. Bank angle limit (+/- 45 deg advisory exceedance)
        if (abs(nav.rollDeg) > 45.0) {
            return this.recordEvent(ExcessiveBankAngle, token.timestampUs, abs(nav.rollDeg), 45.0)
        }

        // 2. Pitch attitude limits (-12 deg to +25 deg exceedance)
        if (nav.pitchDeg > 25.0 || nav.pitchDeg < -12.0) {
            return this.recordEvent(ExtremePitchAttitude, token.timestampUs, nav.pitchDeg, 25.0)
        }

        // 3. Load factor limits (+2.2g or -0.5g exceedance)
        if (nav.loadFactorNz > 2.2 || nav.loadFactorNz < -0.5) {
            return this.recordEvent(HighLoadFactor, token.timestampUs, nav.loadFactorNz, 2.2)
        }

        // 4. Stall warning / alpha protection trigger
        if (cmds.alphaProtActive) {
            return this.recordEvent(StallWarningActive, token.timestampUs, nav.alphaDeg, 11.2)
        }

        // 5. Overspeed trigger
        if (cmds.highSpeedActive) {
            return this.recordEvent(OverspeedVmoMmo, token.timestampUs, nav.casKnots, 340.0)
        }

        // 6. TMR lane trip
        if (votedDemands.activeHealthyLanes < 3) {
            return this.recordEvent(TmrLaneTrip, token.timestampUs, Float64(votedDemands.activeHealthyLanes), 3.0)
        }

        return None
    }

    private func recordEvent(
        eventType: ExceedanceType,
        timestampUs: UInt64,
        val: Float64,
        threshold: Float64
    ): Option<ExceedanceEvent> {
        this.totalExceedancesDetected += 1
        let ev = ExceedanceEvent(eventType, timestampUs, val, threshold, 200)
        this.events.append(ev)
        return Some(ev)
    }
}
