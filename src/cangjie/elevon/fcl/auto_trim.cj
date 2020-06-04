package elevon.fcl

/*
 * Automatic Trimmable Horizontal Stabilizer (THS) pitch trimmer.
 * Maintains zero steady-state elevator load to preserve full dynamic elevator authority.
 */
public class AutoTrimController {
    public var currentThsPositionDeg: Float64
    public var trimRateDegPerSec: Float64
    public var deadbandElevatorDeg: Float64
    public var minThsLimitDeg: Float64 // Full nose down (-2.5 deg)
    public var maxThsLimitDeg: Float64 // Full nose up (+13.5 deg)
    public var isTrimFrozen: Bool

    public init() {
        this.currentThsPositionDeg = 0.0
        this.trimRateDegPerSec = 0.5
        this.deadbandElevatorDeg = 0.75
        this.minThsLimitDeg = -3.0
        this.maxThsLimitDeg = 14.0
        this.isTrimFrozen = false
    }

    public func setTrimFrozen(frozen: Bool) {
        this.isTrimFrozen = frozen
    }

    /*
     * Updates THS auto-trim position based on active elevator deflection.
     */
    public func update(elevatorDeflectionDeg: Float64, dtSec: Float64): Float64 {
        if (this.isTrimFrozen) {
            return this.currentThsPositionDeg
        }

        // If elevator is outside deadband, drive THS in direction to relieve elevator load
        if (elevatorDeflectionDeg > this.deadbandElevatorDeg) {
            // Trailing edge down elevator -> nose down trim needed
            let step = this.trimRateDegPerSec * dtSec
            this.currentThsPositionDeg -= step
        } else if (elevatorDeflectionDeg < -this.deadbandElevatorDeg) {
            // Trailing edge up elevator -> nose up trim needed
            let step = this.trimRateDegPerSec * dtSec
            this.currentThsPositionDeg += step
        }

        // Mechanical limit clamps
        if (this.currentThsPositionDeg > this.maxThsLimitDeg) {
            this.currentThsPositionDeg = this.maxThsLimitDeg
        } else if (this.currentThsPositionDeg < this.minThsLimitDeg) {
            this.currentThsPositionDeg = this.minThsLimitDeg
        }

        return this.currentThsPositionDeg
    }
}
