package elevon.core

/*
 * Cross-Channel Data Link (CCDL) frame exchanged between Lanes A, B, and C
 * at the start of every 10 ms minor cycle.
 */
public struct CcdlFrame {
    public static let MAGIC_SYNC: UInt32 = 0x55AA4343
    public var transmittingLane: LaneId
    public var sequenceNumber: UInt32
    public var timestampUs: UInt64
    public var demandedPitchElevDeg: Float64
    public var demandedRollAileronDeg: Float64
    public var demandedYawRudderDeg: Float64
    public var demandedThsDeg: Float64
    public var activeLawMode: FlightLawMode
    public var faultBitmask: UInt32
    public var crc32c: UInt32

    public init(
        transmittingLane: LaneId,
        sequenceNumber: UInt32,
        timestampUs: UInt64,
        demandedPitchElevDeg: Float64,
        demandedRollAileronDeg: Float64,
        demandedYawRudderDeg: Float64,
        demandedThsDeg: Float64,
        activeLawMode: FlightLawMode,
        faultBitmask: UInt32
    ) {
        this.transmittingLane = transmittingLane
        this.sequenceNumber = sequenceNumber
        this.timestampUs = timestampUs
        this.demandedPitchElevDeg = demandedPitchElevDeg
        this.demandedRollAileronDeg = demandedRollAileronDeg
        this.demandedYawRudderDeg = demandedYawRudderDeg
        this.demandedThsDeg = demandedThsDeg
        this.activeLawMode = activeLawMode
        this.faultBitmask = faultBitmask
        this.crc32c = 0
    }

    /*
     * Calculates deterministic simulated CRC32C checksum for the CCDL frame payload.
     */
    public func computeChecksum(): UInt32 {
        var h: UInt32 = 0x811C9DC5
        h = (h ^ UInt32(this.transmittingLane.toIndex())) * 0x01000193
        h = (h ^ this.sequenceNumber) * 0x01000193
        h = (h ^ UInt32(this.timestampUs & 0xFFFFFFFF)) * 0x01000193
        h = (h ^ UInt32(this.faultBitmask)) * 0x01000193
        return h
    }

    public mutating func finalizeFrame() {
        this.crc32c = this.computeChecksum()
    }

    public func verifyChecksum(): Bool {
        return this.crc32c == this.computeChecksum()
    }
}
