package elevon.core

import elevon.adirs.*
import elevon.fcl.*
import elevon.actuator.*
import elevon.afdx.*
import elevon.fdr.*

/*
 * Complete Flight Control Computer (FCC) master system coordinator.
 * Orchestrates TMR voting, ADIRS fusion, flight control laws, and AFDX telemetry.
 */
public class FccSystem {
    public var timeToken: TimeToken
    public let tmrEngine: TmrEngine
    public let adirsManager: AdirsManager
    public let lawManager: FlightLawManager
    public let allocator: ControlAllocator
    public let blackBox: BlackBoxRecorder
    public let exceedanceDetector: ExceedanceDetector

    // Actuator servo models
    public let elevInboardLeftAct: ServoActuator
    public let elevInboardRightAct: ServoActuator
    public let elevOutboardLeftAct: ServoActuator
    public let elevOutboardRightAct: ServoActuator
    public let aileronLeftAct: ServoActuator
    public let aileronRightAct: ServoActuator
    public let rudderAct: ServoActuator

    public var currentSurfaces: SurfaceDeflections
    public var currentNavState: FusedNavState
    public var currentCommands: FlightLawCommands

    public init() {
        this.timeToken = TimeToken.initial()
        this.tmrEngine = TmrEngine()
        this.adirsManager = AdirsManager()
        this.lawManager = FlightLawManager()
        this.allocator = ControlAllocator()
        this.blackBox = BlackBoxRecorder.defaultRecorder()
        this.exceedanceDetector = ExceedanceDetector()

        this.elevInboardLeftAct = ServoActuator("ELEV_IN_L", 40.0, 0.04, -30.0, 25.0)
        this.elevInboardRightAct = ServoActuator("ELEV_IN_R", 40.0, 0.04, -30.0, 25.0)
        this.elevOutboardLeftAct = ServoActuator("ELEV_OUT_L", 40.0, 0.04, -30.0, 25.0)
        this.elevOutboardRightAct = ServoActuator("ELEV_OUT_R", 40.0, 0.04, -30.0, 25.0)
        this.aileronLeftAct = ServoActuator("AIL_L", 40.0, 0.04, -25.0, 25.0)
        this.aileronRightAct = ServoActuator("AIL_R", 40.0, 0.04, -25.0, 25.0)
        this.rudderAct = ServoActuator("RUDDER", 60.0, 0.04, -30.0, 30.0)

        this.currentSurfaces = SurfaceDeflections()

        let defaultRates = Vector3.zero()
        this.currentNavState = FusedNavState(
            250.0, 130.0, 0.42, 3000.0, 2.5, 0.0, 0.0, 1.2, 0.0, 1.0,
            defaultRates, 3, true, true, true, false
        )

        this.currentCommands = FlightLawCommands(
            0.0, 0.0, 0.0, 0.0, NormalLaw, false, false, false, false
        )
    }

    public func initialize(): Bool {
        return this.tmrEngine.initialize()
    }

    /*
     * Executes one 10 ms minor cycle across all avionics subsystems.
     */
    public func stepCycle(
        actualTimeUs: UInt64,
        inceptor: Inceptor,
        pStaticPa: Float64,
        pTotalPa: Float64,
        tatK: Float64,
        gyroRates: Vector3,
        accelMps2: Vector3,
        dtSec: Float64
    ): VotedSurfaceDemands {
        this.timeToken = this.timeToken.advance(actualTimeUs)

        // 1. Air data and inertial navigation computation
        let adcOut = AirDataComputer.compute(pStaticPa, pTotalPa, tatK)
        var irs = InertialReferenceSystem()
        irs.isInitialized = true
        irs.update(gyroRates, accelMps2, dtSec)

        let aeroAngles = VaneCorrector.correctAngles(
            2.5, true, 2.5, true, adcOut.machNumber, gyroRates.y, adcOut.trueAirspeedMps, false
        )

        // 2. ADIRS sensor fusion (3 units cross-checked)
        this.currentNavState = this.adirsManager.fuse(
            adcOut, irs, adcOut, irs, adcOut, irs,
            aeroAngles.alphaTrueDeg, aeroAngles.betaTrueDeg
        )

        // 3. Flight Control Laws
        this.currentCommands = this.lawManager.step(
            inceptor,
            this.currentNavState,
            false,
            false,
            dtSec
        )

        // 4. Generate CCDL frames for Lanes A, B, C
        var frameA = CcdlFrame(
            LaneA, UInt32(this.timeToken.frameIndex), this.timeToken.timestampUs,
            this.currentCommands.elevatorDeg, this.currentCommands.aileronDeg,
            this.currentCommands.rudderDeg, this.currentCommands.thsDeg,
            this.currentCommands.activeLaw, 0
        )
        frameA.finalizeFrame()

        var frameB = CcdlFrame(
            LaneB, UInt32(this.timeToken.frameIndex), this.timeToken.timestampUs,
            this.currentCommands.elevatorDeg, this.currentCommands.aileronDeg,
            this.currentCommands.rudderDeg, this.currentCommands.thsDeg,
            this.currentCommands.activeLaw, 0
        )
        frameB.finalizeFrame()

        var frameC = CcdlFrame(
            LaneC, UInt32(this.timeToken.frameIndex), this.timeToken.timestampUs,
            this.currentCommands.elevatorDeg, this.currentCommands.aileronDeg,
            this.currentCommands.rudderDeg, this.currentCommands.thsDeg,
            this.currentCommands.activeLaw, 0
        )
        frameC.finalizeFrame()

        // 5. TMR voting across lanes
        let voted = this.tmrEngine.voteCycle(frameA, frameB, frameC)

        // 6. Actuator servo loop execution
        this.currentSurfaces = this.allocator.allocate(this.currentCommands, inceptor.speedbrake_lever)
        this.elevInboardLeftAct.step(dtSec, 3000.0)
        this.elevInboardRightAct.step(dtSec, 3000.0)
        this.aileronLeftAct.step(dtSec, 3000.0)
        this.aileronRightAct.step(dtSec, 3000.0)
        this.rudderAct.step(dtSec, 3000.0)

        // 7. Black box flight parameter recording & exceedance detection
        this.blackBox.recordFrame(this.timeToken, this.currentNavState, inceptor, this.currentCommands, voted)
        this.exceedanceDetector.evaluate(this.timeToken, this.currentNavState, this.currentCommands, voted)

        return voted
    }
}
