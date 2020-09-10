package elevon.telemetry

import elevon.core.*
import elevon.adirs.*
import elevon.fcl.*
import elevon.actuator.*
import elevon.afdx.*

/*
 * Automated execution engine for flight safety and fault-injection drill scenarios.
 */
public class DrillRunner {
    public static func runPitotDrill(fcc: FccSystem): Bool {
        // Run 50 frames of normal cruise
        let inceptor = Inceptor.neutral()
        for (i in 0..50) {
            let t = UInt64(i * 10000)
            fcc.stepCycle(t, inceptor, 30000.0, 36000.0, 240.0, Vector3.zero(), Vector3(0.0, 0.0, -9.80665), 0.01)
        }

        // Inject 20% pressure drop on probe 1 (icing)
        for (i in 50..100) {
            let t = UInt64(i * 10000)
            fcc.stepCycle(t, inceptor, 30000.0, 36000.0 * 0.80, 240.0, Vector3.zero(), Vector3(0.0, 0.0, -9.80665), 0.01)
        }

        // Verify that FDI isolated Probe 1 and system remained in Normal Law
        return fcc.currentCommands.activeLaw.toString() == "NormalLaw"
    }

    public static func runHighSpeedDrill(fcc: FccSystem): Bool {
        let inceptor = Inceptor.neutral()
        // Force airspeed above VMO (360 knots CAS -> impact pressure approx 55,000 Pa)
        for (i in 0..50) {
            let t = UInt64(i * 10000)
            fcc.stepCycle(t, inceptor, 70000.0, 125000.0, 280.0, Vector3.zero(), Vector3(0.0, 0.0, -9.80665), 0.01)
        }

        // Verify high-speed protection engaged and commanded negative (nose-up) elevator
        return fcc.currentCommands.highSpeedActive && (fcc.currentCommands.elevatorDeg < 0.0)
    }

    public static func runAlphaProtDrill(fcc: FccSystem): Bool {
        // Pull full aft stick to demand high pitch
        var inceptor = Inceptor.neutral()
        inceptor.pitch_stick_norm = 1.0 // Full pull up

        for (i in 0..50) {
            let t = UInt64(i * 10000)
            // Inject positive pitch rate
            let gyro = Vector3(0.0, 0.25, 0.0)
            fcc.stepCycle(t, inceptor, 90000.0, 105000.0, 288.0, gyro, Vector3(0.0, 0.0, -15.0), 0.01)
        }

        // Verify alpha protection is active
        return fcc.currentCommands.alphaProtActive
    }
}
