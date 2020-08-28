package elevon.telemetry

import std.math.*
import elevon.core.*
import elevon.adirs.*
import elevon.fcl.*

/*
 * Terminal ANSI Glass Cockpit and Primary Flight Display (PFD) visualizer.
 * Renders real-time attitude ladder, airspeed tape, altimeter, and TMR voting status.
 */
public class CockpitHud {
    public static func renderFrame(
        token: TimeToken,
        nav: FusedNavState,
        cmds: FlightLawCommands,
        voted: VotedSurfaceDemands
    ): String {
        var out = ""
        out += "\n================================================================================\n"
        out += " ELEVON-900X FLY-BY-WIRE GLASS COCKPIT | UTC: " + token.getSimulatedTimeSeconds().toString() + "s | FRAME: " + token.frameIndex.toString() + "\n"
        out += "================================================================================\n"

        // Flight Mode Annunciator (FMA)
        let lawStr = match (cmds.activeLaw) {
            case NormalLaw => "[ NORMAL LAW ]"
            case AlternateLaw => "[ ALTERNATE LAW ]"
            case DirectLaw => "[ DIRECT LAW ]"
            case MechanicalBackup => "[ MECHANICAL BACKUP ]"
        }
        let protStr = cmds.alphaProtActive ? " [A-PROT ACTIVE]" : (cmds.highSpeedActive ? " [HIGH SPEED PROT]" : (cmds.gLimitActive ? " [G-LIMIT ACTIVE]" : " [ENVELOPE NOMINAL]"))
        out += " FMA: " + lawStr + protStr + " | TMR HEALTH: LANES " + voted.activeHealthyLanes.toString() + "/3 ACTIVE\n"
        out += "--------------------------------------------------------------------------------\n"

        // Airspeed, Attitude, Altitude Readouts
        let casStr = nav.casKnots.toString()
        let altStr = nav.altitudeMeters.toString()
        let pitchStr = nav.pitchDeg.toString()
        let rollStr = nav.rollDeg.toString()
        let nzStr = nav.loadFactorNz.toString()
        let alphaStr = nav.alphaDeg.toString()

        out += " SPEED TAPE (CAS)   |        ATTITUDE DIRECTOR        |   ALTIMETER & VSI\n"
        out += "  CAS: " + casStr + " kt   |   Pitch: " + pitchStr + " deg          |   Alt: " + altStr + " m\n"
        out += "  Mach: " + nav.mach.toString() + "        |   Roll:  " + rollStr + " deg          |   TAS: " + nav.tasMps.toString() + " m/s\n"
        out += "  Alpha: " + alphaStr + " deg    |   Nz:    " + nzStr + " g            |   VSI: 0 fpm\n"
        out += "--------------------------------------------------------------------------------\n"

        // Control Surface Deflection Matrix
        out += " CONTROL SURFACE ALLOCATION GAUGES (VOTED OUTPUTS):\n"
        out += "  Elevators: Inboard L/R: " + cmds.elevatorDeg.toString() + " deg\n"
        out += "  Ailerons:  Left: " + cmds.aileronDeg.toString() + " deg | Right: " + (-cmds.aileronDeg).toString() + " deg\n"
        out += "  Rudder:    " + cmds.rudderDeg.toString() + " deg | THS Stabilizer Trim: " + cmds.thsDeg.toString() + " deg\n"
        out += "================================================================================\n"

        return out
    }
}
