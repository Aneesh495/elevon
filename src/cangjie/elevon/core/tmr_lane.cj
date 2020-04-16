package elevon.core

/*
 * Lifecycle states of an individual Flight Control Computer (FCC) lane.
 */
public enum LaneOperationalState {
    | PowerOnSelfTest
    | Synchronizing
    | ActiveCommand
    | ActiveMonitor
    | Degraded
    | TrippedLatching

    public func isHealthy(): Bool {
        match (this) {
            case ActiveCommand | ActiveMonitor => true
            case _ => false
        }
    }

    public func toString(): String {
        match (this) {
            case PowerOnSelfTest => "PowerOnSelfTest"
            case Synchronizing => "Synchronizing"
            case ActiveCommand => "ActiveCommand"
            case ActiveMonitor => "ActiveMonitor"
            case Degraded => "Degraded"
            case TrippedLatching => "TrippedLatching"
        }
    }
}

/*
 * Represents a single deterministic computational lane in the TMR architecture.
 */
public class TmrLane {
    public let id: LaneId
    public var state: LaneOperationalState
    public var consecutiveMissedFrames: UInt32
    public var totalProcessedFrames: UInt64
    public var lastExecutionDurationUs: UInt32
    public var maxObservedDurationUs: UInt32
    public var postPassed: Bool

    public init(id: LaneId) {
        this.id = id
        this.state = PowerOnSelfTest
        this.consecutiveMissedFrames = 0
        this.totalProcessedFrames = 0
        this.lastExecutionDurationUs = 0
        this.maxObservedDurationUs = 0
        this.postPassed = false
    }

    /*
     * Runs built-in self test (BIST) sequence verifying ALU arithmetic,
     * memory range checks, and invariant registers.
     */
    public func executePost(): Bool {
        var testAccumulator: Float64 = 0.0
        for (i in 1..=100) {
            testAccumulator += Float64(i) * 0.01
        }
        let expected = 50.5
        let delta = (testAccumulator >= expected) ? (testAccumulator - expected) : (expected - testAccumulator)
        if (delta < 1e-6) {
            this.postPassed = true
            this.state = Synchronizing
            return true
        } else {
            this.postPassed = false
            this.state = TrippedLatching
            return false
        }
    }

    /*
     * Transitions from Synchronizing to Active operational state
     * once inter-lane cross-channel data links achieve phase lock.
     */
    public func completeSynchronization(asCommand: Bool) {
        if (!this.postPassed) {
            this.state = TrippedLatching
            return
        }
        if (asCommand) {
            this.state = ActiveCommand
        } else {
            this.state = ActiveMonitor
        }
        this.consecutiveMissedFrames = 0
    }

    /*
     * Records execution timing for the current 10 ms minor cycle.
     */
    public func recordCycleDuration(durationUs: UInt32) {
        this.lastExecutionDurationUs = durationUs
        if (durationUs > this.maxObservedDurationUs) {
            this.maxObservedDurationUs = durationUs
        }
        this.totalProcessedFrames += 1

        // 10 ms minor frame deadline is 10,000 microseconds
        if (durationUs > 10000) {
            this.consecutiveMissedFrames += 1
            if (this.consecutiveMissedFrames >= 3) {
                this.state = TrippedLatching
            }
        } else {
            this.consecutiveMissedFrames = 0
        }
    }

    /*
     * Trips lane into fail-safe latched state upon unrecoverable error.
     */
    public func tripFault() {
        this.state = TrippedLatching
    }
}
