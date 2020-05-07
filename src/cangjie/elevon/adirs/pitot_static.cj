package elevon.adirs

import std.math.*

/*
 * Pitot-Static probe transducer modeling and Calibrated Airspeed (CAS) extraction.
 * Implements compressible Saint-Venant aerodynamic formulas.
 */
public class PitotStaticCalculator {
    public static let A0_MPS: Float64 = 340.29437     // Speed of sound at sea level (m/s)
    public static let P0_PA: Float64 = 101325.0       // Sea level standard pressure (Pa)
    public static let RHO0_KG_M3: Float64 = 1.225     // Sea level standard density (kg/m^3)

    /*
     * Calculates Calibrated Airspeed (CAS) in meters/second from impact pressure qc = Pt - Ps.
     */
    public static func computeCas(impactPressurePa: Float64): Float64 {
        if (impactPressurePa <= 0.0) {
            return 0.0
        }

        let ratio = (impactPressurePa / P0_PA) + 1.0
        let term = pow(ratio, 2.0 / 7.0) - 1.0

        if (term <= 0.0) {
            return 0.0
        }

        return A0_MPS * sqrt(5.0 * term)
    }

    /*
     * Converts meters/second to knots (1 m/s = 1.94384 knots).
     */
    public static func mpsToKnots(mps: Float64): Float64 {
        return mps * 1.94384449
    }

    /*
     * Converts knots to meters/second.
     */
    public static func knotsToMps(knots: Float64): Float64 {
        return knots * 0.51444444
    }

    /*
     * Validates that static and total pressures lie within operational flight envelope.
     */
    public static func validatePressures(pStaticPa: Float64, pTotalPa: Float64): Bool {
        // Static pressure: 8,000 Pa (~55,000 ft) to 110,000 Pa (-2,000 ft)
        if (pStaticPa < 8000.0 || pStaticPa > 110000.0) {
            return false
        }
        // Total pressure must be at least static pressure
        if (pTotalPa < (pStaticPa - 100.0)) {
            return false
        }
        // Impact pressure limit: up to 60,000 Pa (~600 knots CAS)
        let qc = pTotalPa - pStaticPa
        if (qc < -50.0 || qc > 60000.0) {
            return false
        }
        return true
    }
}
