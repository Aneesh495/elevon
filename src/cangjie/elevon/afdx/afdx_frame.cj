package elevon.afdx

/*
 * ARINC 664 (AFDX) Ethernet packet builder and addressing codec.
 */
public struct AfdxFrameHeader {
    public var destMac: Array<UInt8>
    public var srcMac: Array<UInt8>
    public var destIp: String
    public var destPort: UInt16
    public var virtualLinkId: UInt16
    public var sequenceNumber: UInt8

    public init(vlId: UInt16, seq: UInt8, destPort: UInt16) {
        this.virtualLinkId = vlId
        this.sequenceNumber = seq
        this.destPort = destPort

        let vlHigh = UInt8((vlId >> 8) & 0xFF)
        let vlLow = UInt8(vlId & 0xFF)

        // AFDX Destination MAC format: 03:00:00:00:VL_HIGH:VL_LOW
        this.destMac = [0x03, 0x00, 0x00, 0x00, vlHigh, vlLow]
        // AFDX Source MAC: 02:00:00:00:01:01
        this.srcMac = [0x02, 0x00, 0x00, 0x00, 0x01, 0x01]

        // AFDX Multicast IP format: 10.0.VL_HIGH.VL_LOW
        this.destIp = "10.0." + vlHigh.toString() + "." + vlLow.toString()
    }
}

/*
 * Full AFDX packet representation including payload and sequence number.
 */
public class AfdxPacket {
    public var header: AfdxFrameHeader
    public var payload: Array<UInt8>
    public var networkPlane: String // "NETWORK_A" or "NETWORK_B"
    public var timestampUs: UInt64

    public init(
        vlId: UInt16,
        seq: UInt8,
        destPort: UInt16,
        payload: Array<UInt8>,
        networkPlane: String,
        timestampUs: UInt64
    ) {
        this.header = AfdxFrameHeader(vlId, seq, destPort)
        this.payload = payload
        this.networkPlane = networkPlane
        this.timestampUs = timestampUs
    }

    /*
     * Returns total frame length including headers, sequence number, and CRC.
     */
    public func getWireLengthBytes(): Int64 {
        // Ethernet (14) + IPv4 (20) + UDP (8) + Payload + SeqNum (1) + FCS (4)
        return 14 + 20 + 8 + this.payload.size + 1 + 4
    }
}
