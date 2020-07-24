package elevon.afdx

/*
 * ARINC 664 receiver integrity checker for a specific Virtual Link.
 * Enforces strict sequence number progression (0 to 255) and rejects stale or duplicate frames.
 */
public class AfdxIntegrityChecker {
    public let vlId: UInt16
    public var expectedSequenceNumber: UInt8
    public var totalAcceptedFrames: UInt64
    public var totalIntegrityErrors: UInt64
    public var totalDroppedFrames: UInt64
    public var isFirstFrameReceived: Bool

    public init(vlId: UInt16) {
        this.vlId = vlId
        this.expectedSequenceNumber = 0
        this.totalAcceptedFrames = 0
        this.totalIntegrityErrors = 0
        this.totalDroppedFrames = 0
        this.isFirstFrameReceived = false
    }

    /*
     * Validates incoming frame sequence number against ARINC 664 progression rules.
     */
    public func checkFrame(seqNum: UInt8): Bool {
        if (!this.isFirstFrameReceived) {
            // First frame synchronizes the expected sequence counter
            this.expectedSequenceNumber = (seqNum == 255) ? 0 : (seqNum + 1)
            this.isFirstFrameReceived = true
            this.totalAcceptedFrames += 1
            return true
        }

        // Exact match: perfectly in sequence
        if (seqNum == this.expectedSequenceNumber) {
            this.expectedSequenceNumber = (this.expectedSequenceNumber == 255) ? 0 : (this.expectedSequenceNumber + 1)
            this.totalAcceptedFrames += 1
            return true
        }

        // Calculate modular forward distance: (seqNum - expected) mod 256
        let diff = (Int64(seqNum) - Int64(this.expectedSequenceNumber) + 256) % 256

        // Allowed drop window: 1 or 2 missed frames accepted with drop counter increment
        if (diff >= 1 && diff <= 2) {
            this.totalDroppedFrames += UInt64(diff)
            this.expectedSequenceNumber = (seqNum == 255) ? 0 : (seqNum + 1)
            this.totalAcceptedFrames += 1
            return true
        }

        // Out-of-order, stale duplicate, or severe discontinuity
        this.totalIntegrityErrors += 1
        return false
    }
}
