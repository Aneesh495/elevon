package elevon.telemetry

import elevon.afdx.*

/*
 * ARINC 664 (AFDX) packet inspector and wire telemetry decoder.
 */
public class PacketInspector {
    public static func inspect(packet: AfdxPacket): String {
        var out = ""
        out += "[AFDX CAPTURE] VL: " + packet.header.virtualLinkId.toString()
        out += " | SEQ: " + packet.header.sequenceNumber.toString()
        out += " | PLANE: " + packet.networkPlane
        out += " | WIRE LEN: " + packet.getWireLengthBytes().toString() + " B"
        out += " | DEST IP: " + packet.header.destIp + ":" + packet.header.destPort.toString()
        out += " | TIME: " + packet.timestampUs.toString() + " us\n"
        return out
    }

    public static func summarizeRedundancy(red: AfdxRedundancyManager): String {
        var out = ""
        out += "AFDX REDUNDANCY STATUS (VL " + red.vlId.toString() + "):\n"
        out += "  Delivered Frames: " + red.totalDeliveredFrames.toString() + "\n"
        out += "  Redundant Discards: " + red.totalRedundantDiscarded.toString() + "\n"
        out += "  Integrity Errors: " + red.integrityChecker.totalIntegrityErrors.toString() + "\n"
        out += "  Max Network A/B Skew: " + red.maxObservedSkewUs.toString() + " us\n"
        out += "  Physical Planes: Network A: " + (red.isNetworkAAlive ? "UP" : "DOWN") + " | Network B: " + (red.isNetworkBAlive ? "UP" : "DOWN") + "\n"
        return out
    }
}
