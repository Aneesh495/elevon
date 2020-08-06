package elevon.fdr

import elevon.core.*
import elevon.adirs.*
import elevon.fcl.*

/*
 * Mandatory FAA Part 121 / EASA flight parameter record.
 */
public struct FdrRecord {
    public var timestampUs: UInt64
    public var frameIndex: UInt64
    public var casKnots: Float32
    public var mach: Float32
    public var altitudeM: Float32
    public var pitchDeg: Float32
    public var rollDeg: Float32
    public var headingDeg: Float32
    public var pitchRateDegS: Float32
    public var rollRateDegS: Float32
    public var yawRateDegS: Float32
    public var loadFactorNz: Float32
    public var elevatorLeftDeg: Float32
    public var elevatorRightDeg: Float32
    public var aileronLeftDeg: Float32
    public var aileronRightDeg: Float32
    public var rudderDeg: Float32
    public var thsDeg: Float32
    public var pitchStickNorm: Float32
    public var rollStickNorm: Float32
    public var rudderPedalNorm: Float32
    public var activeLawMode: UInt8
    public var protFlags: UInt8
    public var laneHealthBits: UInt8
    public var crc32c: UInt32

    public init() {
        this.timestampUs = 0
        this.frameIndex = 0
        this.casKnots = 0.0
        this.mach = 0.0
        this.altitudeM = 0.0
        this.pitchDeg = 0.0
        this.rollDeg = 0.0
        this.headingDeg = 0.0
        this.pitchRateDegS = 0.0
        this.rollRateDegS = 0.0
        this.yawRateDegS = 0.0
        this.loadFactorNz = 1.0
        this.elevatorLeftDeg = 0.0
        this.elevatorRightDeg = 0.0
        this.aileronLeftDeg = 0.0
        this.aileronRightDeg = 0.0
        this.rudderDeg = 0.0
        this.thsDeg = 0.0
        this.pitchStickNorm = 0.0
        this.rollStickNorm = 0.0
        this.rudderPedalNorm = 0.0
        this.activeLawMode = 0
        this.protFlags = 0
        this.laneHealthBits = 0x07 // Lanes A, B, C healthy
        this.crc32c = 0
    }

    public func computeCrc(): UInt32 {
        var h: UInt32 = 0x811C9DC5
        h = (h ^ UInt32(this.timestampUs & 0xFFFFFFFF)) * 0x01000193
        h = (h ^ UInt32(this.frameIndex & 0xFFFFFFFF)) * 0x01000193
        h = (h ^ UInt32(this.activeLawMode)) * 0x01000193
        h = (h ^ UInt32(this.protFlags)) * 0x01000193
        h = (h ^ UInt32(this.laneHealthBits)) * 0x01000193
        return h
    }

    public mutating func finalizeRecord() {
        this.crc32c = this.computeCrc()
    }

    public func verifyCrc(): Bool {
        return this.crc32c == this.computeCrc()
    }
}

/*
 * Pre-allocated circular ring buffer for the Flight Data Recorder.
 * Stores up to 10,000 continuous 100 Hz frames without dynamic heap allocations.
 */
public class BlackBoxRecorder {
    public let capacity: Int64
    public var records: Array<FdrRecord>
    public var writeIndex: Int64
    public var totalRecordedRecords: UInt64
    public var isWrapped: Bool

    public init(capacity: Int64) {
        this.capacity = capacity
        this.records = Array<FdrRecord>(capacity, item: FdrRecord())
        this.writeIndex = 0
        this.totalRecordedRecords = 0
        this.isWrapped = false
    }

    public static func defaultRecorder(): BlackBoxRecorder {
        return BlackBoxRecorder(10000) // 100 seconds at 100 Hz
    }

    /*
     * Records a new minor cycle flight state vector into the circular buffer.
     */
    public func recordFrame(
        token: TimeToken,
        nav: FusedNavState,
        inceptor: Inceptor,
        cmds: FlightLawCommands,
        votedDemands: VotedSurfaceDemands
    ) {
        var r = FdrRecord()
        r.timestampUs = token.timestampUs
        r.frameIndex = token.frameIndex
        r.casKnots = Float32(nav.casKnots)
        r.mach = Float32(nav.mach)
        r.altitudeM = Float32(nav.altitudeMeters)
        r.pitchDeg = Float32(nav.pitchDeg)
        r.rollDeg = Float32(nav.rollDeg)
        r.headingDeg = Float32(nav.headingDeg)
        r.pitchRateDegS = Float32(nav.bodyAngularRates.y * 57.2957795)
        r.rollRateDegS = Float32(nav.bodyAngularRates.x * 57.2957795)
        r.yawRateDegS = Float32(nav.bodyAngularRates.z * 57.2957795)
        r.loadFactorNz = Float32(nav.loadFactorNz)

        r.elevatorLeftDeg = Float32(cmds.elevatorDeg)
        r.elevatorRightDeg = Float32(cmds.elevatorDeg)
        r.aileronLeftDeg = Float32(cmds.aileronDeg)
        r.aileronRightDeg = Float32(-cmds.aileronDeg)
        r.rudderDeg = Float32(cmds.rudderDeg)
        r.thsDeg = Float32(cmds.thsDeg)

        r.pitchStickNorm = Float32(inceptor.pitch_stick_norm)
        r.rollStickNorm = Float32(inceptor.roll_stick_norm)
        r.rudderPedalNorm = Float32(inceptor.rudder_pedal_norm)

        r.activeLawMode = UInt8(match (cmds.activeLaw) {
            case NormalLaw => 0
            case AlternateLaw => 1
            case DirectLaw => 2
            case MechanicalBackup => 3
        })

        var pFlags: UInt8 = 0
        if (cmds.alphaProtActive) pFlags |= (1 << 0)
        if (cmds.alphaFloorActive) pFlags |= (1 << 1)
        if (cmds.highSpeedActive) pFlags |= (1 << 2)
        if (cmds.gLimitActive) pFlags |= (1 << 3)
        r.protFlags = pFlags

        var lHealth: UInt8 = 0
        if (votedDemands.laneAHealth) lHealth |= (1 << 0)
        if (votedDemands.laneBHealth) lHealth |= (1 << 1)
        if (votedDemands.laneCHealth) lHealth |= (1 << 2)
        r.laneHealthBits = lHealth

        r.finalizeRecord()

        this.records[this.writeIndex] = r
        this.writeIndex += 1
        this.totalRecordedRecords += 1

        if (this.writeIndex >= this.capacity) {
            this.writeIndex = 0
            this.isWrapped = true
        }
    }

    public func getLatestRecord(): Option<FdrRecord> {
        if (this.totalRecordedRecords == 0) {
            return None
        }
        let lastIdx = (this.writeIndex == 0) ? (this.capacity - 1) : (this.writeIndex - 1)
        return Some(this.records[lastIdx])
    }
}
