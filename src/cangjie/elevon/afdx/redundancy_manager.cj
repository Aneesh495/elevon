package elevon.afdx

/*
 * ARINC 664 Redundancy Manager.
 * Implements First-Valid packet selection across redundant Networks A and B,
 * tracking inter-network transit skew and physical link availability.
 */
public class AfdxRedundancyManager {
    public let vlId: UInt16
    public let integrityChecker: AfdxIntegrityChecker
    public var lastSeenSeqA: Int32
    public var lastSeenSeqB: Int32
    public var lastArrivalTimeUsA: UInt64
    public var lastArrivalTimeUsB: UInt64
    public var maxObservedSkewUs: UInt64
    public var totalDeliveredFrames: UInt64
    public var totalRedundantDiscarded: UInt64
    public var isNetworkAAlive: Bool
    public var isNetworkBAlive: Bool

    public init(vlId: UInt16) {
        this.vlId = vlId
        this.integrityChecker = AfdxIntegrityChecker(vlId)
        this.lastSeenSeqA = -1
        this.lastSeenSeqB = -1
        this.lastArrivalTimeUsA = 0
        this.lastArrivalTimeUsB = 0
        this.maxObservedSkewUs = 0
        this.totalDeliveredFrames = 0
        this.totalRedundantDiscarded = 0
        this.isNetworkAAlive = true
        this.isNetworkBAlive = true
    }

    /*
     * Processes packet arrival from Network A or Network B.
     * Returns true if packet is first-valid and should be forwarded to the avionics application.
     */
    public func processFrame(packet: AfdxPacket): Bool {
        let seq = packet.header.sequenceNumber
        let isNetA = (packet.networkPlane == "NETWORK_A")

        if (isNetA) {
            this.lastSeenSeqA = Int32(seq)
            this.lastArrivalTimeUsA = packet.timestampUs
            this.isNetworkAAlive = true
        } else {
            this.lastSeenSeqB = Int32(seq)
            this.lastArrivalTimeUsB = packet.timestampUs
            this.isNetworkBAlive = true
        }

        // Measure skew if corresponding frame arrived on opposite network
        if (this.lastArrivalTimeUsA > 0 && this.lastArrivalTimeUsB > 0) {
            let skew = (this.lastArrivalTimeUsA >= this.lastArrivalTimeUsB)
                ? (this.lastArrivalTimeUsA - this.lastArrivalTimeUsB)
                : (this.lastArrivalTimeUsB - this.lastArrivalTimeUsA)
            if (skew > this.maxObservedSkewUs) {
                this.maxObservedSkewUs = skew
            }
        }

        // Integrity checking and first-valid sequence acceptance
        let accepted = this.integrityChecker.checkFrame(seq)
        if (accepted) {
            this.totalDeliveredFrames += 1
            return true
        } else {
            this.totalRedundantDiscarded += 1
            return false
        }
    }
}
