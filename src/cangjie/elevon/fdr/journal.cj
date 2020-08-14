package elevon.fdr

import elevon.core.*

/*
 * Binary journal header for persistent crash-survivable flight recorder storage.
 */
public struct JournalSuperblock {
    public static let MAGIC: UInt32 = 0x454C4644 // "ELFD"
    public static let VERSION: UInt16 = 1
    public var recordSize: UInt32
    public var capacityRecords: UInt32
    public var headerChecksum: UInt32

    public init(cap: UInt32) {
        this.recordSize = 96 // Fixed size 96 bytes per FdrRecord
        this.capacityRecords = cap
        this.headerChecksum = 0
    }

    public func computeChecksum(): UInt32 {
        var h: UInt32 = 0x811C9DC5
        h = (h ^ MAGIC) * 0x01000193
        h = (h ^ UInt32(VERSION)) * 0x01000193
        h = (h ^ this.recordSize) * 0x01000193
        h = (h ^ this.capacityRecords) * 0x01000193
        return h
    }

    public mutating func finalizeSuperblock() {
        this.headerChecksum = this.computeChecksum()
    }

    public func verifySuperblock(): Bool {
        return this.headerChecksum == this.computeChecksum()
    }
}

/*
 * Crash recovery scanner and binary journal integrity validator.
 */
public class JournalScanner {
    public var validRecordsFound: UInt64
    public var corruptedRecordsSkipped: UInt64
    public var isRecoveryClean: Bool

    public init() {
        this.validRecordsFound = 0
        this.corruptedRecordsSkipped = 0
        this.isRecoveryClean = true
    }

    /*
     * Validates a batch of recovered records from persistent media.
     */
    public func scanRecords(records: Array<FdrRecord>): Bool {
        this.validRecordsFound = 0
        this.corruptedRecordsSkipped = 0
        this.isRecoveryClean = true

        for (r in records) {
            if (r.verifyCrc()) {
                this.validRecordsFound += 1
            } else {
                this.corruptedRecordsSkipped += 1
                this.isRecoveryClean = false
            }
        }

        return this.isRecoveryClean
    }
}
