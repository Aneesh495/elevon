package elevon.core

import std.math.*

/*
 * Standard status and error codes for Elevon avionics operations.
 */
public enum FlightStatus {
    | Ok
    | ErrNullPtr
    | ErrOutOfBounds
    | ErrByzantineFault
    | ErrBagViolation
    | ErrFdrCorrupt
    | ErrCrcMismatch
    | ErrHydraulicLoss
    | ErrActuatorStall
    | ErrCcdlTimeout
    | ErrAirDataDisagree
    | ErrInvalidState

    public func isOk(): Bool {
        match (this) {
            case Ok => true
            case _ => false
        }
    }

    public func toString(): String {
        match (this) {
            case Ok => "Ok"
            case ErrNullPtr => "ErrNullPtr"
            case ErrOutOfBounds => "ErrOutOfBounds"
            case ErrByzantineFault => "ErrByzantineFault"
            case ErrBagViolation => "ErrBagViolation"
            case ErrFdrCorrupt => "ErrFdrCorrupt"
            case ErrCrcMismatch => "ErrCrcMismatch"
            case ErrHydraulicLoss => "ErrHydraulicLoss"
            case ErrActuatorStall => "ErrActuatorStall"
            case ErrCcdlTimeout => "ErrCcdlTimeout"
            case ErrAirDataDisagree => "ErrAirDataDisagree"
            case ErrInvalidState => "ErrInvalidState"
        }
    }
}

/*
 * Triplex lane identity for Triple-Modular Redundancy.
 */
public enum LaneId {
    | LaneA
    | LaneB
    | LaneC

    public func toIndex(): Int64 {
        match (this) {
            case LaneA => 0
            case LaneB => 1
            case LaneC => 2
        }
    }

    public static func fromIndex(idx: Int64): Option<LaneId> {
        match (idx) {
            case 0 => Some(LaneA)
            case 1 => Some(LaneB)
            case 2 => Some(LaneC)
            case _ => None
        }
    }
}

/*
 * Operational modes for Flight Control Laws.
 */
public enum FlightLawMode {
    | NormalLaw
    | AlternateLaw
    | DirectLaw
    | MechanicalBackup

    public func toString(): String {
        match (this) {
            case NormalLaw => "NormalLaw"
            case AlternateLaw => "AlternateLaw"
            case DirectLaw => "DirectLaw"
            case MechanicalBackup => "MechanicalBackup"
        }
    }
}

/*
 * 3-Dimensional vector in body or earth reference frames.
 */
public struct Vector3 {
    public var x: Float64
    public var y: Float64
    public var z: Float64

    public init(x: Float64, y: Float64, z: Float64) {
        this.x = x
        this.y = y
        this.z = z
    }

    public static func zero(): Vector3 {
        return Vector3(0.0, 0.0, 0.0)
    }

    public func add(other: Vector3): Vector3 {
        return Vector3(this.x + other.x, this.y + other.y, this.z + other.z)
    }

    public func sub(other: Vector3): Vector3 {
        return Vector3(this.x - other.x, this.y - other.y, this.z - other.z)
    }

    public func scale(factor: Float64): Vector3 {
        return Vector3(this.x * factor, this.y * factor, this.z * factor)
    }

    public func dot(other: Vector3): Float64 {
        return (this.x * other.x) + (this.y * other.y) + (this.z * other.z)
    }

    public func cross(other: Vector3): Vector3 {
        return Vector3(
            (this.y * other.z) - (this.z * other.y),
            (this.z * other.x) - (this.x * other.z),
            (this.x * other.y) - (this.y * other.x)
        )
    }

    public func normSquared(): Float64 {
        return (this.x * this.x) + (this.y * this.y) + (this.z * this.z)
    }

    public func norm(): Float64 {
        return sqrt(this.normSquared())
    }

    public func normalized(): Vector3 {
        let n = this.norm()
        if (n < 1e-12) {
            return Vector3.zero()
        }
        return this.scale(1.0 / n)
    }
}

/*
 * 3x3 Matrix for transformation and inertia tensors.
 */
public struct Matrix3 {
    public var m00: Float64
    public var m01: Float64
    public var m02: Float64
    public var m10: Float64
    public var m11: Float64
    public var m12: Float64
    public var m20: Float64
    public var m21: Float64
    public var m22: Float64

    public init(
        m00: Float64, m01: Float64, m02: Float64,
        m10: Float64, m11: Float64, m12: Float64,
        m20: Float64, m21: Float64, m22: Float64
    ) {
        this.m00 = m00; this.m01 = m01; this.m02 = m02
        this.m10 = m10; this.m11 = m11; this.m12 = m12
        this.m20 = m20; this.m21 = m21; this.m22 = m22
    }

    public static func identity(): Matrix3 {
        return Matrix3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        )
    }

    public func multiplyVector(v: Vector3): Vector3 {
        return Vector3(
            (this.m00 * v.x) + (this.m01 * v.y) + (this.m02 * v.z),
            (this.m10 * v.x) + (this.m11 * v.y) + (this.m12 * v.z),
            (this.m20 * v.x) + (this.m21 * v.y) + (this.m22 * v.z)
        )
    }

    public func transpose(): Matrix3 {
        return Matrix3(
            this.m00, this.m10, this.m20,
            this.m01, this.m11, this.m21,
            this.m02, this.m12, this.m22
        )
    }

    public func determinant(): Float64 {
        return (this.m00 * ((this.m11 * this.m22) - (this.m12 * this.m21))) -
               (this.m01 * ((this.m10 * this.m22) - (this.m12 * this.m20))) +
               (this.m02 * ((this.m10 * this.m21) - (this.m11 * this.m20)))
    }
}

/*
 * Unit quaternion representing spatial attitude rotation.
 */
public struct Quaternion {
    public var w: Float64
    public var x: Float64
    public var y: Float64
    public var z: Float64

    public init(w: Float64, x: Float64, y: Float64, z: Float64) {
        this.w = w
        this.x = x
        this.y = y
        this.z = z
    }

    public static func identity(): Quaternion {
        return Quaternion(1.0, 0.0, 0.0, 0.0)
    }

    public func norm(): Float64 {
        return sqrt((this.w * this.w) + (this.x * this.x) + (this.y * this.y) + (this.z * this.z))
    }

    public func normalized(): Quaternion {
        let n = this.norm()
        if (n < 1e-12) {
            return Quaternion.identity()
        }
        let inv = 1.0 / n
        return Quaternion(this.w * inv, this.x * inv, this.y * inv, this.z * inv)
    }

    public func conjugate(): Quaternion {
        return Quaternion(this.w, -this.x, -this.y, -this.z)
    }

    public func multiply(q: Quaternion): Quaternion {
        return Quaternion(
            (this.w * q.w) - (this.x * q.x) - (this.y * q.y) - (this.z * q.z),
            (this.w * q.x) + (this.x * q.w) + (this.y * q.z) - (this.z * q.y),
            (this.w * q.y) - (this.x * q.z) + (this.y * q.w) + (this.z * q.x),
            (this.w * q.z) + (this.x * q.y) - (this.y * q.x) + (this.z * q.w)
        )
    }

    public func rotateVector(v: Vector3): Vector3 {
        let p = Quaternion(0.0, v.x, v.y, v.z)
        let q_norm = this.normalized()
        let q_conj = q_norm.conjugate()
        let res = q_norm.multiply(p).multiply(q_conj)
        return Vector3(res.x, res.y, res.z)
    }

    /*
     * Converts attitude quaternion to Euler angles:
     * Roll (phi), Pitch (theta), Heading (psi) in radians.
     */
    public func toEulerAngles(): (Float64, Float64, Float64) {
        let sinr_cosp = 2.0 * ((this.w * this.x) + (this.y * this.z))
        let cosr_cosp = 1.0 - (2.0 * ((this.x * this.x) + (this.y * this.y)))
        let roll = atan2(sinr_cosp, cosr_cosp)

        let sinp = 2.0 * ((this.w * this.y) - (this.z * this.x))
        var pitch: Float64 = 0.0
        if (abs(sinp) >= 1.0) {
            pitch = (sinp > 0.0) ? (3.141592653589793 / 2.0) : (-3.141592653589793 / 2.0)
        } else {
            pitch = asin(sinp)
        }

        let siny_cosp = 2.0 * ((this.w * this.z) + (this.x * this.y))
        let cosy_cosp = 1.0 - (2.0 * ((this.y * this.y) + (this.z * this.z)))
        let yaw = atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)
    }

    /*
     * Creates attitude quaternion from Euler angles:
     * Roll (phi), Pitch (theta), Heading (psi) in radians.
     */
    public static func fromEulerAngles(roll: Float64, pitch: Float64, yaw: Float64): Quaternion {
        let cy = cos(yaw * 0.5)
        let sy = sin(yaw * 0.5)
        let cp = cos(pitch * 0.5)
        let sp = sin(pitch * 0.5)
        let cr = cos(roll * 0.5)
        let sr = sin(roll * 0.5)

        return Quaternion(
            (cr * cp * cy) + (sr * sp * sy),
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy)
        )
    }
}

/*
 * Pilot inceptor demands (sidestick, rudder pedals, speedbrake lever).
 */
public struct Inceptor {
    public var pitch_stick_norm: Float64
    public var roll_stick_norm: Float64
    public var rudder_pedal_norm: Float64
    public var speedbrake_lever: Float64
    public var ap_disconnect: Bool
    public var take_over_priority: Bool

    public init(
        pitch_stick_norm: Float64,
        roll_stick_norm: Float64,
        rudder_pedal_norm: Float64,
        speedbrake_lever: Float64,
        ap_disconnect: Bool,
        take_over_priority: Bool
    ) {
        this.pitch_stick_norm = pitch_stick_norm
        this.roll_stick_norm = roll_stick_norm
        this.rudder_pedal_norm = rudder_pedal_norm
        this.speedbrake_lever = speedbrake_lever
        this.ap_disconnect = ap_disconnect
        this.take_over_priority = take_over_priority
    }

    public static func neutral(): Inceptor {
        return Inceptor(0.0, 0.0, 0.0, 0.0, false, false)
    }
}

/*
 * Primary and secondary control surface deflections (degrees).
 */
public struct SurfaceDeflections {
    public var elevInboardLeftDeg: Float64
    public var elevInboardRightDeg: Float64
    public var elevOutboardLeftDeg: Float64
    public var elevOutboardRightDeg: Float64
    public var aileronLeftDeg: Float64
    public var aileronRightDeg: Float64
    public var rudderUpperDeg: Float64
    public var rudderLowerDeg: Float64
    public var thsDeg: Float64
    public var spoilersLeftDeg: Array<Float64>
    public var spoilersRightDeg: Array<Float64>

    public init() {
        this.elevInboardLeftDeg = 0.0
        this.elevInboardRightDeg = 0.0
        this.elevOutboardLeftDeg = 0.0
        this.elevOutboardRightDeg = 0.0
        this.aileronLeftDeg = 0.0
        this.aileronRightDeg = 0.0
        this.rudderUpperDeg = 0.0
        this.rudderLowerDeg = 0.0
        this.thsDeg = 0.0
        this.spoilersLeftDeg = [0.0, 0.0, 0.0, 0.0, 0.0]
        this.spoilersRightDeg = [0.0, 0.0, 0.0, 0.0, 0.0]
    }
}

/*
 * Hydraulic circuit pressure distribution status.
 */
public struct HydraulicStatus {
    public var greenPressPsi: Float64
    public var bluePressPsi: Float64
    public var yellowPressPsi: Float64
    public var greenAvail: Bool
    public var blueAvail: Bool
    public var yellowAvail: Bool

    public init(g: Float64, b: Float64, y: Float64) {
        this.greenPressPsi = g
        this.bluePressPsi = b
        this.yellowPressPsi = y
        this.greenAvail = (g >= 2200.0)
        this.blueAvail = (b >= 2200.0)
        this.yellowAvail = (y >= 2200.0)
    }

    public static func nominal(): HydraulicStatus {
        return HydraulicStatus(3000.0, 3000.0, 3000.0)
    }
}
