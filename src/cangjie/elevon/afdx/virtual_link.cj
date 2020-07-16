package elevon.afdx

/*
 * ARINC 664 Part 7 (AFDX) Virtual Link (VL) configuration and token bucket shaper.
 */
public class VirtualLinkShaper {
    public let vlId: UInt16
    public let name: String
    public let bagMs: UInt32
    public let lMaxBytes: UInt32
    public let maxJitterUs: UInt32

    public var lastTransmitTimeUs: UInt64
    public var sequenceNumber: UInt8
    public var totalTransmittedFrames: UInt64
    public var totalBagViolations: UInt64

    public init(
        vlId: UInt16,
        name: String,
        bagMs: UInt32,
        lMaxBytes: UInt32,
        maxJitterUs: UInt32
    ) {
        this.vlId = vlId
        this.name = name
        this.bagMs = bagMs
        this.lMaxBytes = lMaxBytes
        this.maxJitterUs = maxJitterUs
        this.lastTransmitTimeUs = 0
        this.sequenceNumber = 0
        this.totalTransmittedFrames = 0
        this.totalBagViolations = 0
    }

    /*
     * Calculates allocated link bandwidth in bits per second:
     * BW = (L_max * 8) / (BAG_ms * 1e-3)
     */
    public func getAllocatedBandwidthBps(): Float64 {
        let frameBits = Float64(this.lMaxBytes * 8)
        let bagSec = Float64(this.bagMs) * 0.001
        return frameBits / bagSec
    }

    /*
     * Checks if transmission at current time conforms to Bandwidth Allocation Gap (BAG).
     */
    public func canTransmit(currentTimeUs: UInt64): Bool {
        if (this.lastTransmitTimeUs == 0) {
            return true
        }
        let bagUs = UInt64(this.bagMs) * 1000
        return (currentTimeUs - this.lastTransmitTimeUs) >= bagUs
    }

    /*
     * Authorizes transmission, updates internal state, and increments the 8-bit AFDX sequence number.
     */
    public func recordTransmission(currentTimeUs: UInt64, frameSizeBytes: UInt32): UInt8 {
        if (!this.canTransmit(currentTimeUs)) {
            this.totalBagViolations += 1
        }

        this.lastTransmitTimeUs = currentTimeUs
        this.totalTransmittedFrames += 1

        let seq = this.sequenceNumber
        // Sequence number wraps modulo 256 (0 to 255) per ARINC 664 specification
        this.sequenceNumber = (this.sequenceNumber == 255) ? 0 : (this.sequenceNumber + 1)
        return seq
    }
}
