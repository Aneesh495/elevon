#!/usr/bin/env python3
"""
Maneuvering envelope (V-n diagram) ASCII visualization and boundary inspector.
Plots structural limit load factors vs calibrated airspeed.
"""

def generate_vn_diagram():
    out = []
    out.append("================================================================================")
    out.append(" ELEVON-900X MANEUVERING ENVELOPE (V-n FLIGHT DIAGRAM)")
    out.append("================================================================================")
    out.append(" Load Factor (g)")
    out.append("  +3.0 |")
    out.append("  +2.5 |--------*-----------------------------------* (Structural Limit: +2.5g)")
    out.append("  +2.0 |       /                                    | (Flaps Limit: +2.0g)")
    out.append("  +1.5 |      /                                     |")
    out.append("  +1.0 |-----*--------------------------------------| (1g Level Flight)")
    out.append("  +0.5 |    /                                       |")
    out.append("   0.0 |--*-----------------------------------------| (Flaps Negative Limit: 0.0g)")
    out.append("  -0.5 |   \\                                        |")
    out.append("  -1.0 |----*---------------------------------------* (Clean Negative Limit: -1.0g)")
    out.append("  -1.5 |")
    out.append("       +--------------------------------------------+----------> CAS (Knots)")
    out.append("       0   Vs1(135)   Va(220)                     Vmo(340)  Vdf(385)")
    out.append("================================================================================")
    return "\n".join(out)

def main():
    diagram = generate_vn_diagram()
    print(diagram)

if __name__ == "__main__":
    main()
