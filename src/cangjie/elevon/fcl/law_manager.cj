package elevon.fcl

import elevon.core.*
import elevon.adirs.*

/*
 * Consolidated flight control surface commands produced by Flight Control Laws.
 */
public struct FlightLawCommands {
    public var elevatorDeg: Float64
    public var aileronDeg: Float64
    public var rudderDeg: Float64
    public var thsDeg: Float64
    public var activeLaw: FlightLawMode
    public var alphaProtActive: Bool
    public var alphaFloorActive: Bool
    public var highSpeedActive: Bool
    public var gLimitActive: Bool

    public init(
        elevatorDeg: Float64,
        aileronDeg: Float64,
        rudderDeg: Float64,
        thsDeg: Float64,
        activeLaw: FlightLawMode,
        alphaProtActive: Bool,
        alphaFloorActive: Bool,
        highSpeedActive: Bool,
        gLimitActive: Bool
    ) {
        this.elevatorDeg = elevatorDeg
        this.aileronDeg = aileronDeg
        this.rudderDeg = rudderDeg
        this.thsDeg = thsDeg
        this.activeLaw = activeLaw
        this.alphaProtActive = alphaProtActive
        this.alphaFloorActive = alphaFloorActive
        this.highSpeedActive = highSpeedActive
        this.gLimitActive = gLimitActive
    }
}

/*
 * Flight Control Law state machine and master coordinator.
 */
public class FlightLawManager {
    public var currentMode: FlightLawMode
    public let cStar: CStarController
    public let autoTrim: AutoTrimController
    public let lateralLaw: LateralFlightLaw
    public let directionalLaw: DirectionalFlightLaw
    public let alphaGov: AlphaProtectionGovernor
    public let highSpeedGov: HighSpeedProtectionGovernor
    public let attitudeLim: AttitudeLimiter
    public let gGov: GLimiterGovernor

    public init() {
        this.currentMode = NormalLaw
        this.cStar = CStarController.defaultController()
        this.autoTrim = AutoTrimController()
        this.lateralLaw = LateralFlightLaw()
        this.directionalLaw = DirectionalFlightLaw()
        this.alphaGov = AlphaProtectionGovernor()
        this.highSpeedGov = HighSpeedProtectionGovernor()
        this.attitudeLim = AttitudeLimiter()
        this.gGov = GLimiterGovernor()
    }

    /*
     * Evaluates degradation conditions and selects active Flight Control Law mode.
     */
    public func evaluateModeTransitions(
        healthyAdirsUnits: Int64,
        healthyFccLanes: Int64,
        gearExtended: Bool
    ): FlightLawMode {
        if (healthyFccLanes == 0) {
            this.currentMode = MechanicalBackup
            return MechanicalBackup
        }

        if (healthyAdirsUnits >= 2 && healthyFccLanes >= 2) {
            this.currentMode = NormalLaw
            return NormalLaw
        }

        if (healthyAdirsUnits == 1 || healthyFccLanes == 1) {
            if (gearExtended) {
                this.currentMode = DirectLaw
            } else {
                this.currentMode = AlternateLaw
            }
            return this.currentMode
        }

        this.currentMode = DirectLaw
        return DirectLaw
    }

    /*
     * Executes the complete flight control law cycle.
     */
    public func step(
        inceptor: Inceptor,
        nav: FusedNavState,
        gearExtended: Bool,
        flapsExtended: Bool,
        dtSec: Float64
    ): FlightLawCommands {
        this.evaluateModeTransitions(nav.healthyAdirsUnits, 3, gearExtended)

        match (this.currentMode) {
            case NormalLaw =>
                // 1. Longitudinal C* flight law
                var elevCmd = this.cStar.step(
                    inceptor.pitch_stick_norm,
                    nav.loadFactorNz,
                    nav.bodyAngularRates.y,
                    dtSec
                )

                // 2. High-AOA alpha protection
                let (alphaElevOverride, alphaActive, alphaFloor) = this.alphaGov.evaluate(
                    nav.alphaDeg,
                    inceptor.pitch_stick_norm
                )
                if (alphaActive) {
                    elevCmd += alphaElevOverride
                    this.autoTrim.setTrimFrozen(true)
                } else {
                    this.autoTrim.setTrimFrozen(false)
                }

                // 3. High-speed VMO/MMO protection
                let (speedElevOverride, speedActive) = this.highSpeedGov.evaluate(
                    nav.casKnots,
                    nav.mach,
                    inceptor.pitch_stick_norm
                )
                if (speedActive) {
                    elevCmd += speedElevOverride
                    this.autoTrim.setTrimFrozen(true)
                }

                // 4. Attitude pitch limiter
                let (pitchLimited, _) = this.attitudeLim.limitPitchElevator(
                    elevCmd,
                    nav.pitchDeg,
                    nav.bodyAngularRates.y
                )
                elevCmd = pitchLimited

                // 5. Structural g-load limiter
                let (gLimited, gPos, gNeg) = this.gGov.enforce(elevCmd, nav.loadFactorNz, flapsExtended)
                elevCmd = gLimited

                // 6. Auto-trimmer updates THS
                let thsCmd = this.autoTrim.update(elevCmd, dtSec)

                // 7. Lateral roll rate law
                var ailCmd = this.lateralLaw.step(
                    inceptor.roll_stick_norm,
                    nav.rollDeg,
                    nav.bodyAngularRates.x * 57.2957795,
                    dtSec
                )
                let (ailBankLimited, _) = this.attitudeLim.limitBankAileron(ailCmd, nav.rollDeg)
                ailCmd = ailBankLimited

                // 8. Directional yaw damper and turn coordination
                let rudCmd = this.directionalLaw.step(
                    inceptor.rudder_pedal_norm,
                    nav.bodyAngularRates.z,
                    nav.rollDeg,
                    nav.pitchDeg,
                    nav.tasMps,
                    dtSec
                )

                return FlightLawCommands(
                    elevCmd,
                    ailCmd,
                    rudCmd,
                    thsCmd,
                    NormalLaw,
                    alphaActive,
                    alphaFloor,
                    speedActive,
                    gPos || gNeg
                )

            case AlternateLaw =>
                // Reduced protection: C* pitch law maintained, roll rate direct, stall warning active
                let elevCmd = this.cStar.step(inceptor.pitch_stick_norm, nav.loadFactorNz, nav.bodyAngularRates.y, dtSec)
                let thsCmd = this.autoTrim.update(elevCmd, dtSec)
                let ailCmd = inceptor.roll_stick_norm * 25.0
                let rudCmd = inceptor.rudder_pedal_norm * 30.0
                return FlightLawCommands(elevCmd, ailCmd, rudCmd, thsCmd, AlternateLaw, false, false, false, false)

            case DirectLaw =>
                // Direct proportional stick to surface mapping
                let elevCmd = inceptor.pitch_stick_norm * 30.0
                let ailCmd = inceptor.roll_stick_norm * 25.0
                let rudCmd = inceptor.rudder_pedal_norm * 30.0
                return FlightLawCommands(elevCmd, ailCmd, rudCmd, this.autoTrim.currentThsPositionDeg, DirectLaw, false, false, false, false)

            case MechanicalBackup =>
                return FlightLawCommands(0.0, 0.0, inceptor.rudder_pedal_norm * 30.0, this.autoTrim.currentThsPositionDeg, MechanicalBackup, false, false, false, false)
        }
    }
}
