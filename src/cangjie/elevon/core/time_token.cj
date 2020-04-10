package elevon.core

/*
 * Deterministic Time-Triggered execution primitives for Elevon FCC.
 * Models DO-178C Level A synchronous minor and major scheduling frames.
 */
public struct TimeToken {
    public var frameIndex: UInt64
    public var timestampUs: UInt64
    public var minorCycleUs: UInt64
    public var majorCycleUs: UInt64
    public var jitterUs: Int64

    public init(frameIndex: UInt64, timestampUs: UInt64) {
        this.frameIndex = frameIndex
        this.timestampUs = timestampUs
        this.minorCycleUs = 10000 // 10 ms minor frame (100 Hz)
        this.majorCycleUs = 50000 // 50 ms major frame (20 Hz)
        this.jitterUs = 0
    }

    public static func initial(): TimeToken {
        return TimeToken(0, 0)
    }

    /*
     * Advances to the next minor frame and calculates timing jitter.
     */
    public func advance(actualCurrentTimeUs: UInt64): TimeToken {
        var next = TimeToken(this.frameIndex + 1, actualCurrentTimeUs)
        let expectedTimeUs = this.timestampUs + this.minorCycleUs
        if (actualCurrentTimeUs >= expectedTimeUs) {
            next.jitterUs = Int64(actualCurrentTimeUs - expectedTimeUs)
        } else {
            next.jitterUs = -Int64(expectedTimeUs - actualCurrentTimeUs)
        }
        return next
    }

    /*
     * Returns true if the current frame is on a major cycle boundary (every 5th frame).
     */
    public func isMajorCycleBoundary(): Bool {
        return (this.frameIndex % 5) == 0
    }

    /*
     * Checks if jitter exceeds safety tolerance threshold (500 microseconds).
     */
    public func isJitterCompliant(maxJitterAllowedUs: Int64): Bool {
        let absJitter = (this.jitterUs >= 0) ? this.jitterUs : -this.jitterUs
        return absJitter <= maxJitterAllowedUs
    }

    public func getSimulatedTimeSeconds(): Float64 {
        return Float64(this.timestampUs) / 1000000.0
    }
}
