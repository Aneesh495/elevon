package elevon.core

/*
 * Majority voting result for discrete boolean signals.
 */
public struct DiscreteVoteResult {
    public var votedState: Bool
    public var unanimous: Bool
    public var laneADisagreed: Bool
    public var laneBDisagreed: Bool
    public var laneCDisagreed: Bool

    public init(
        votedState: Bool,
        unanimous: Bool,
        laneADisagreed: Bool,
        laneBDisagreed: Bool,
        laneCDisagreed: Bool
    ) {
        this.votedState = votedState
        this.unanimous = unanimous
        this.laneADisagreed = laneADisagreed
        this.laneBDisagreed = laneBDisagreed
        this.laneCDisagreed = laneCDisagreed
    }
}

/*
 * 2-out-of-3 (2oo3) majority voter with latching fault accumulator.
 */
public class DiscreteMajorityVoter {
    public var faultCountA: UInt32
    public var faultCountB: UInt32
    public var faultCountC: UInt32
    public let latchLimit: UInt32

    public init(latchLimit: UInt32) {
        this.faultCountA = 0
        this.faultCountB = 0
        this.faultCountC = 0
        this.latchLimit = latchLimit
    }

    public func vote(
        sigA: Bool, validA: Bool,
        sigB: Bool, validB: Bool,
        sigC: Bool, validC: Bool,
        failSafeDefault: Bool
    ): DiscreteVoteResult {
        // Case 1: All three channels valid
        if (validA && validB && validC) {
            // Majority logic: (A and B) or (B and C) or (C and A)
            let voted = (sigA && sigB) || (sigB && sigC) || (sigC && sigA)
            let disA = (sigA != voted)
            let disB = (sigB != voted)
            let disC = (sigC != voted)
            let unanimous = (!disA && !disB && !disC)

            if (disA) { this.faultCountA += 1 } else { this.faultCountA = 0 }
            if (disB) { this.faultCountB += 1 } else { this.faultCountB = 0 }
            if (disC) { this.faultCountC += 1 } else { this.faultCountC = 0 }

            return DiscreteVoteResult(voted, unanimous, disA, disB, disC)
        }

        // Case 2: Duplex cross-check (two channels valid)
        if (validA && validB && !validC) {
            if (sigA == sigB) {
                return DiscreteVoteResult(sigA, true, false, false, true)
            } else {
                return DiscreteVoteResult(failSafeDefault, false, true, true, true)
            }
        }
        if (validB && validC && !validA) {
            if (sigB == sigC) {
                return DiscreteVoteResult(sigB, true, true, false, false)
            } else {
                return DiscreteVoteResult(failSafeDefault, false, true, true, true)
            }
        }
        if (validC && validA && !validB) {
            if (sigC == sigA) {
                return DiscreteVoteResult(sigC, true, false, true, false)
            } else {
                return DiscreteVoteResult(failSafeDefault, false, true, true, true)
            }
        }

        // Case 3: Simplex (one channel valid)
        if (validA) { return DiscreteVoteResult(sigA, true, false, true, true) }
        if (validB) { return DiscreteVoteResult(sigB, true, true, false, true) }
        if (validC) { return DiscreteVoteResult(sigC, true, true, true, false) }

        // Case 4: No valid channel, output fail-safe default
        return DiscreteVoteResult(failSafeDefault, false, true, true, true)
    }

    public func isLaneLatchedFault(lane: LaneId): Bool {
        match (lane) {
            case LaneA => this.faultCountA >= this.latchLimit
            case LaneB => this.faultCountB >= this.latchLimit
            case LaneC => this.faultCountC >= this.latchLimit
        }
    }
}
