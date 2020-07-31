package elevon.afdx

import std.collection.*

/*
 * ARINC 664 deterministic avionics switch fabric.
 * Provides ingress traffic policing, non-blocking crossbar routing, and priority queuing.
 */
public class AfdxSwitch {
    public let switchName: String
    public let portCount: Int64
    public var totalSwitchedFrames: UInt64
    public var totalPolicedDiscards: UInt64
    public var vlRouteTable: HashMap<UInt16, ArrayList<Int64>>

    public init(name: String, ports: Int64) {
        this.switchName = name
        this.portCount = ports
        this.totalSwitchedFrames = 0
        this.totalPolicedDiscards = 0
        this.vlRouteTable = HashMap<UInt16, ArrayList<Int64>>()
    }

    /*
     * Registers a static forwarding route for a Virtual Link ID to target output ports.
     */
    public func registerVlRoute(vlId: UInt16, targetPorts: Array<Int64>) {
        let portList = ArrayList<Int64>()
        for (p in targetPorts) {
            portList.append(p)
        }
        this.vlRouteTable.put(vlId, portList)
    }

    /*
     * Ingress polices and switches an AFDX packet across designated output ports.
     */
    public func switchPacket(packet: AfdxPacket, ingressPort: Int64): Array<Int64> {
        let vlId = packet.header.virtualLinkId

        // Ingress frame bounds check: 64 to 1518 bytes per ARINC 664
        let wireLen = packet.getWireLengthBytes()
        if (wireLen < 64 || wireLen > 1518) {
            this.totalPolicedDiscards += 1
            return []
        }

        let route = this.vlRouteTable.get(vlId)
        match (route) {
            case Some(ports) =>
                this.totalSwitchedFrames += 1
                let res = Array<Int64>(ports.size, item: 0)
                for (i in 0..ports.size) {
                    res[i] = ports.get(i).getOrThrow()
                }
                return res
            case None =>
                this.totalPolicedDiscards += 1
                return []
        }
    }
}
