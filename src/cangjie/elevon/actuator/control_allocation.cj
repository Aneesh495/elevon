package elevon.actuator

import std.math.*
import elevon.core.*
import elevon.fcl.*

/*
 * Multi-surface aerodynamic control allocation matrix.
 * Maps generalized pitch, roll, yaw, and speedbrake demands to 12 physical surfaces.
 */
public class ControlAllocator {
    public var inboardElevatorGain: Float64
    public var outboardElevatorGain: Float64
    public var aileronGain: Float64
    public var rollSpoilerGain: Float64
    public var rudderGain: Float64

    public var leftInboardElevatorHealthy: Bool
    public var rightInboardElevatorHealthy: Bool
    public var leftOutboardElevatorHealthy: Bool
    public var rightOutboardElevatorHealthy: Bool

    public init() {
        this.inboardElevatorGain = 1.0
        this.outboardElevatorGain = 1.0
        this.aileronGain = 1.0
        this.rollSpoilerGain = 0.65
        this.rudderGain = 1.0

        this.leftInboardElevatorHealthy = true
        this.rightInboardElevatorHealthy = true
        this.leftOutboardElevatorHealthy = true
        this.rightOutboardElevatorHealthy = true
    }

    /*
     * Dispatches generalized flight law commands across physical surfaces.
     */
    public func allocate(
        commands: FlightLawCommands,
        speedbrakeCmd: Float64
    ): SurfaceDeflections {
        var surf = SurfaceDeflections()

        // 1. Pitch allocation with failure reconfiguration
        var elevDemand = commands.elevatorDeg
        if (!this.leftInboardElevatorHealthy || !this.rightInboardElevatorHealthy) {
            // Reconfigure: boost outboard elevators to compensate for lost inboard authority
            elevDemand *= 1.45
        }

        surf.elevInboardLeftDeg = this.leftInboardElevatorHealthy ? (elevDemand * this.inboardElevatorGain) : 0.0
        surf.elevInboardRightDeg = this.rightInboardElevatorHealthy ? (elevDemand * this.inboardElevatorGain) : 0.0
        surf.elevOutboardLeftDeg = this.leftOutboardElevatorHealthy ? (elevDemand * this.outboardElevatorGain) : 0.0
        surf.elevOutboardRightDeg = this.rightOutboardElevatorHealthy ? (elevDemand * this.outboardElevatorGain) : 0.0
        surf.thsDeg = commands.thsDeg

        // 2. Lateral allocation: differential ailerons + roll-spoiler augmentation
        let ailCmd = commands.aileronDeg
        surf.aileronLeftDeg = ailCmd
        surf.aileronRightDeg = -ailCmd

        // Speedbrake baseline deployment (spoilers 2, 3, 4 deploy symmetrically)
        let baseSpeedbrakeDeg = speedbrakeCmd * 30.0
        for (i in 0..5) {
            surf.spoilersLeftDeg[i] = (i >= 1 && i <= 3) ? baseSpeedbrakeDeg : 0.0
            surf.spoilersRightDeg[i] = (i >= 1 && i <= 3) ? baseSpeedbrakeDeg : 0.0
        }

        // Roll spoiler augmentation: if aileron command > 8 degrees, deploy spoilers on down-going wing
        if (ailCmd > 8.0) {
            let rollSpoilerAdd = (ailCmd - 8.0) * this.rollSpoilerGain
            for (i in 1..=4) {
                surf.spoilersRightDeg[i] += rollSpoilerAdd
                if (surf.spoilersRightDeg[i] > 35.0) {
                    surf.spoilersRightDeg[i] = 35.0
                }
            }
        } else if (ailCmd < -8.0) {
            let rollSpoilerAdd = (-ailCmd - 8.0) * this.rollSpoilerGain
            for (i in 1..=4) {
                surf.spoilersLeftDeg[i] += rollSpoilerAdd
                if (surf.spoilersLeftDeg[i] > 35.0) {
                    surf.spoilersLeftDeg[i] = 35.0
                }
            }
        }

        // 3. Directional allocation: dual rudder segments
        surf.rudderUpperDeg = commands.rudderDeg * this.rudderGain
        surf.rudderLowerDeg = commands.rudderDeg * this.rudderGain

        return surf
    }
}
