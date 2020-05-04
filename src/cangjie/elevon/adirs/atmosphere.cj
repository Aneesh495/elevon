package elevon.adirs

import std.math.*

/*
 * International Standard Atmosphere (ISA) barometric and gas dynamics model.
 * Compliant with ICAO Document 7488/3.
 */
public class IsaAtmosphere {
    public static let P0_PA: Float64 = 101325.0       // Sea level pressure (Pa)
    public static let T0_K: Float64 = 288.15          // Sea level temperature (K)
    public static let RHO0_KG_M3: Float64 = 1.225     // Sea level density (kg/m^3)
    public static let G0_M_S2: Float64 = 9.80665      // Standard gravity (m/s^2)
    public static let R_SPECIFIC: Float64 = 287.05287 // Gas constant for dry air (J/kg*K)
    public static let GAMMA: Float64 = 1.4            // Ratio of specific heats
    public static let LAPSE_RATE: Float64 = -0.0065   // Troposphere lapse rate (K/m)
    public static let TROPOPAUSE_ALT_M: Float64 = 11000.0 // Tropopause altitude (m)
    public static let T_TROPOPAUSE_K: Float64 = 216.65   // Tropopause temperature (K)
    public static let P_TROPOPAUSE_PA: Float64 = 22632.1 // Pressure at 11,000 m (Pa)

    /*
     * Calculates ambient temperature (Kelvin) at geopotential altitude (meters).
     */
    public static func temperatureAtAltitude(altMeters: Float64): Float64 {
        if (altMeters <= TROPOPAUSE_ALT_M) {
            return T0_K + (LAPSE_RATE * altMeters)
        } else {
            return T_TROPOPAUSE_K
        }
    }

    /*
     * Calculates ambient static pressure (Pascals) at geopotential altitude (meters).
     */
    public static func pressureAtAltitude(altMeters: Float64): Float64 {
        if (altMeters <= TROPOPAUSE_ALT_M) {
            let t = temperatureAtAltitude(altMeters)
            let exponent = -G0_M_S2 / (LAPSE_RATE * R_SPECIFIC)
            return P0_PA * pow(t / T0_K, exponent)
        } else {
            let deltaH = altMeters - TROPOPAUSE_ALT_M
            let exponent = -G0_M_S2 * deltaH / (R_SPECIFIC * T_TROPOPAUSE_K)
            return P_TROPOPAUSE_PA * exp(exponent)
        }
    }

    /*
     * Calculates ambient air density (kg/m^3) at geopotential altitude.
     */
    public static func densityAtAltitude(altMeters: Float64): Float64 {
        let p = pressureAtAltitude(altMeters)
        let t = temperatureAtAltitude(altMeters)
        return p / (R_SPECIFIC * t)
    }

    /*
     * Calculates local speed of sound (m/s) at temperature T (Kelvin).
     */
    public static func speedOfSound(tKelvin: Float64): Float64 {
        return sqrt(GAMMA * R_SPECIFIC * tKelvin)
    }

    /*
     * Inverse barometric formula: calculates pressure altitude (meters)
     * given static pressure (Pascals).
     */
    public static func altitudeFromPressure(pStaticPa: Float64): Float64 {
        if (pStaticPa <= 0.0) {
            return 30000.0
        }
        if (pStaticPa >= P_TROPOPAUSE_PA) {
            let exponent = -LAPSE_RATE * R_SPECIFIC / G0_M_S2
            let ratio = pow(pStaticPa / P0_PA, exponent)
            return (T0_K / -LAPSE_RATE) * (1.0 - ratio)
        } else {
            let logRatio = log(pStaticPa / P_TROPOPAUSE_PA)
            let deltaH = -R_SPECIFIC * T_TROPOPAUSE_K * logRatio / G0_M_S2
            return TROPOPAUSE_ALT_M + deltaH
        }
    }
}
