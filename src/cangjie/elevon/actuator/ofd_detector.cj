package elevon.actuator

import std.math.*

/*
 * Oscillatory Failure Detection (OFD) for flight control surface actuators.
 * Detects hazardous limit-cycle oscillations (2.5 Hz to 12 Hz) to prevent flutter.
 */
public class OfdDetector {
    public let surfaceName: String
    public var oscillationCount: UInt32
    public var lastZeroCrossingTimeUs: UInt64
    public var previousPositionDeg: Float64
    public var peakPositiveDeg: Float64
    public var peakNegativeDeg: Float64
    public var isTripped: Bool
    public var tripThresholdAmplitudeDeg: Float64

    public init(surfaceName: String) {
        this.surfaceName = surfaceName
        this.oscillationCount = 0
        this.lastZeroCrossingTimeUs = 0
        this.previousPositionDeg = 0.0
        this.peakPositiveDeg = 0.0
        this.peakNegativeDeg = 0.0
        this.isTripped = false
        this.tripThresholdAmplitudeDeg = 1.75
    }

    /*
     * Analyzes instantaneous surface position for high-frequency oscillation signatures.
     */
    public func update(currentPositionDeg: Float64, timestampUs: UInt64): Bool {
        if (this.isTripped) {
            return true
        }

        // Track positive and negative peaks between zero crossings
        if (currentPositionDeg > this.peakPositiveDeg) {
            this.peakPositiveDeg = currentPositionDeg
        }
        if (currentPositionDeg < this.peakNegativeDeg) {
            this.peakNegativeDeg = currentPositionDeg
        }

        // Detect zero-crossing
        let isZeroCrossing = (this.previousPositionDeg * currentPositionDeg) < 0.0
        if (isZeroCrossing) {
            let halfPeriodUs = timestampUs - this.lastZeroCrossingTimeUs
            this.lastZeroCrossingTimeUs = timestampUs

            // Half-period between 40 ms (12.5 Hz) and 200 ms (2.5 Hz)
            if (halfPeriodUs >= 40000 && halfPeriodUs <= 200000) {
                let peakToPeak = this.peakPositiveDeg - this.peakNegativeDeg
                if (peakToPeak >= this.tripThresholdAmplitudeDeg) {
                    this.oscillationCount += 1
                    // 3 consecutive confirmed cycles triggers protective bypass
                    if (this.oscillationCount >= 3) {
                        this.isTripped = true
                        return true
                    }
                }
            } else {
                // Out of frequency band, decay counter
                if (this.oscillationCount > 0) {
                    this.oscillationCount -= 1
                }
            }

            // Reset peaks for next half-cycle
            this.peakPositiveDeg = 0.0
            this.peakNegativeDeg = 0.0
        }

        this.previousPositionDeg = currentPositionDeg
        return this.isTripped
    }

    public func reset() {
        this.oscillationCount = 0
        this.isTripped = false
        this.peakPositiveDeg = 0.0
        this.peakNegativeDeg = 0.0
    }
}
