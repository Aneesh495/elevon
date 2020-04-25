package elevon.core

/*
 * Consolidated voted flight control surface demands from the TMR engine.
 */
public struct VotedSurfaceDemands {
    public var elevatorDeg: Float64
    public var aileronDeg: Float64
    public var rudderDeg: Float64
    public var thsDeg: Float64
    public var votedLawMode: FlightLawMode
    public var activeHealthyLanes: Int64
    public var laneAHealth: Bool
    public var laneBHealth: Bool
    public var laneCHealth: Bool

    public init(
        elevatorDeg: Float64,
        aileronDeg: Float64,
        rudderDeg: Float64,
        thsDeg: Float64,
        votedLawMode: FlightLawMode,
        activeHealthyLanes: Int64,
        laneAHealth: Bool,
        laneBHealth: Bool,
        laneCHealth: Bool
    ) {
        this.elevatorDeg = elevatorDeg
        this.aileronDeg = aileronDeg
        this.rudderDeg = rudderDeg
        this.thsDeg = thsDeg
        this.votedLawMode = votedLawMode
        this.activeHealthyLanes = activeHealthyLanes
        this.laneAHealth = laneAHealth
        this.laneBHealth = laneBHealth
        this.laneCHealth = laneCHealth
    }
}

/*
 * Unified Triple-Modular Redundancy (TMR) coordination and voting engine.
 */
public class TmrEngine {
    public let laneA: TmrLane
    public let laneB: TmrLane
    public let laneC: TmrLane

    public let elevatorVoter: MvsVoter
    public let aileronVoter: MvsVoter
    public let rudderVoter: MvsVoter
    public let thsVoter: MvsVoter

    public var totalVotingCycles: UInt64
    public var byzantineDisagreementCount: UInt64

    public init() {
        this.laneA = TmrLane(LaneA)
        this.laneB = TmrLane(LaneB)
        this.laneC = TmrLane(LaneC)

        // Standard tolerance thresholds: 1.5 deg for surfaces, 0.5 deg for THS
        this.elevatorVoter = MvsVoter(1.5, 2.5)
        this.aileronVoter = MvsVoter(1.5, 2.5)
        this.rudderVoter = MvsVoter(1.5, 2.5)
        this.thsVoter = MvsVoter(0.5, 1.0)

        this.totalVotingCycles = 0
        this.byzantineDisagreementCount = 0
    }

    public func initialize(): Bool {
        let pA = this.laneA.executePost()
        let pB = this.laneB.executePost()
        let pC = this.laneC.executePost()

        if (pA && pB && pC) {
            this.laneA.completeSynchronization(true)  // Lane A is command
            this.laneB.completeSynchronization(false) // Lane B is monitor
            this.laneC.completeSynchronization(false) // Lane C is monitor
            return true
        }
        return false
    }

    /*
     * Executes one voting minor cycle (10 ms) across incoming CCDL frames.
     */
    public func voteCycle(frameA: CcdlFrame, frameB: CcdlFrame, frameC: CcdlFrame): VotedSurfaceDemands {
        this.totalVotingCycles += 1

        let validA = this.laneA.state.isHealthy() && frameA.verifyChecksum()
        let validB = this.laneB.state.isHealthy() && frameB.verifyChecksum()
        let validC = this.laneC.state.isHealthy() && frameC.verifyChecksum()

        // Vote pitch elevator
        let resElev = this.elevatorVoter.vote(
            frameA.demandedPitchElevDeg, validA,
            frameB.demandedPitchElevDeg, validB,
            frameC.demandedPitchElevDeg, validC
        )

        // Vote lateral aileron
        let resAil = this.aileronVoter.vote(
            frameA.demandedRollAileronDeg, validA,
            frameB.demandedRollAileronDeg, validB,
            frameC.demandedRollAileronDeg, validC
        )

        // Vote directional rudder
        let resRud = this.rudderVoter.vote(
            frameA.demandedYawRudderDeg, validA,
            frameB.demandedYawRudderDeg, validB,
            frameC.demandedYawRudderDeg, validC
        )

        // Vote Trimmable Horizontal Stabilizer (THS)
        let resThs = this.thsVoter.vote(
            frameA.demandedThsDeg, validA,
            frameB.demandedThsDeg, validB,
            frameC.demandedThsDeg, validC
        )

        // Failure detection & isolation: trip lane if persistent divergence
        if (resElev.laneADiverged || resAil.laneADiverged || resRud.laneADiverged) {
            if (this.elevatorVoter.persistentDisagreementCountA >= 5) {
                this.laneA.tripFault()
            }
        }
        if (resElev.laneBDiverged || resAil.laneBDiverged || resRud.laneBDiverged) {
            if (this.elevatorVoter.persistentDisagreementCountB >= 5) {
                this.laneB.tripFault()
            }
        }
        if (resElev.laneCDiverged || resAil.laneCDiverged || resRud.laneCDiverged) {
            if (this.elevatorVoter.persistentDisagreementCountC >= 5) {
                this.laneC.tripFault()
            }
        }

        if (resElev.byzantineDisagreement || resAil.byzantineDisagreement) {
            this.byzantineDisagreementCount += 1
        }

        // Active law selection: if at least 2 lanes healthy -> NormalLaw, else degrade to DirectLaw
        var activeLaw = NormalLaw
        let healthyCount = (validA ? 1 : 0) + (validB ? 1 : 0) + (validC ? 1 : 0)
        if (healthyCount < 2) {
            activeLaw = DirectLaw
        }

        return VotedSurfaceDemands(
            resElev.votedValue,
            resAil.votedValue,
            resRud.votedValue,
            resThs.votedValue,
            activeLaw,
            Int64(healthyCount),
            this.laneA.state.isHealthy(),
            this.laneB.state.isHealthy(),
            this.laneC.state.isHealthy()
        )
    }
}
