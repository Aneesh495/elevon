# Elevon Architecture Specification

## 1. System Overview

**Elevon** is a deterministic, safety-critical Fly-by-Wire (FBW) Primary Flight Control Computer (FCC) and ARINC 664 Part 7 (AFDX) avionics network engine. Engineered in **Cangjie** with zero-cost **C17** hardware acceleration, Elevon coordinates triple-modular redundant (TMR) computational lanes, compressible flow air data estimation (ADIRS), and dynamic flight envelope protection laws.

The architecture strictly follows civil transport category requirements (FAA 14 CFR Part 25 / EASA CS-25) and DO-178C Level A / ARP4754A design assurance guidelines.

---

## 2. High-Level Subsystem Topology

```mermaid
flowchart TD
    subgraph Sensors ["Air Data & Inertial Reference System (ADIRS)"]
        P1["Pitot-Static Probe 1"]
        P2["Pitot-Static Probe 2"]
        P3["Pitot-Static Probe 3"]
        IMU1["Triaxial IMU 1"]
        IMU2["Triaxial IMU 2"]
        IMU3["Triaxial IMU 3"]
        Vane["Dual AOA / AOS Vanes"]
    end

    subgraph ADIRS_FDI ["ADIRS Sensor Fusion & FDI"]
        ADC["Compressible Air Data Computer"]
        IRS["Strapdown Quaternion INS"]
        VaneCorr["Fuselage Position Error Correction"]
        SensorVoter["Triplex Sensor MVS Voter"]
    end

    subgraph FCC_TMR ["Triplex Flight Control Computer Core"]
        LaneA["Lane A (Command Lane)"]
        LaneB["Lane B (Monitor Lane)"]
        LaneC["Lane C (Standby Arbitrator)"]
        CCDL["Cross-Channel Data Link (100 Hz Sync)"]
        MVS["Mid-Value Selection Voter Matrix"]
        LawMgr["Flight Law State Machine"]
    end

    subgraph Laws ["Flight Control Laws & Envelope Protection"]
        CStar["C* Longitudinal Blend Law"]
        LatDir["Lateral-Directional & Turn Coord"]
        AutoTrim["THS Automatic Trimmer"]
        ProtGov["Flight Envelope Protections"]
    end

    subgraph Actuation ["Control Surface Actuation (12 Surfaces)"]
        Alloc["Control Allocation Matrix"]
        ServoLoops["EHA Servo Loops & Slew Limiters"]
        OFD["Oscillatory Failure Detection (OFD)"]
        Surfaces["Elevators, Ailerons, Rudder, Spoilers, THS"]
    end

    subgraph Network ["Deterministic Avionics Network"]
        AFDX_Tx["AFDX Virtual Link Shaper (BAG 10ms)"]
        AFDX_Sw["Non-Blocking Crossbar Switch Matrix"]
        AFDX_Rx["First-Valid Redundancy Manager (Net A/B)"]
    end

    subgraph Recording ["Flight Safety Recording"]
        FDR["Crash-Survivable Black Box Ring Buffer"]
        Exceed["Exceedance Snapshot Event Detector"]
        Journal["Circular On-Disk Binary Persistence"]
    end

    Sensors --> ADIRS_FDI
    ADIRS_FDI --> FCC_TMR
    FCC_TMR <--> CCDL
    FCC_TMR --> Laws
    Laws --> MVS
    MVS --> Alloc
    Alloc --> ServoLoops
    ServoLoops --> OFD
    OFD --> Surfaces

    FCC_TMR --> AFDX_Tx
    AFDX_Tx --> AFDX_Sw
    AFDX_Sw --> AFDX_Rx
    AFDX_Rx --> Recording
    FCC_TMR --> FDR
    FDR --> Exceed
    Exceed --> Journal
```

---

## 3. Triple-Modular Redundancy (TMR) Voting Architecture

Elevon employs three independent hardware-isolated execution lanes (Lane A, Lane B, Lane C). Each lane executes identical flight control algorithms on a synchronized 100 Hz (10 ms) minor cycle.

```mermaid
sequenceDiagram
    participant LaneA as Lane A (Command)
    participant LaneB as Lane B (Monitor)
    participant LaneC as Lane C (Standby)
    participant CCDL as Cross-Channel Data Link
    participant Voter as MVS Voter Matrix
    participant Act as Surface Actuators

    Note over LaneA,LaneC: 100 Hz Minor Cycle Tick (0.0 ms)
    LaneA->>CCDL: Broadcast CcdlFrame(A, CRC32C)
    LaneB->>CCDL: Broadcast CcdlFrame(B, CRC32C)
    LaneC->>CCDL: Broadcast CcdlFrame(C, CRC32C)
    
    Note over CCDL: Inter-Lane Exchange (0.5 ms)
    CCDL->>Voter: Deliver Frames A, B, C
    
    Note over Voter: Mid-Value Selection & FDI (1.0 ms)
    Voter->>Voter: Compute Median: MVS(A, B, C)
    Voter->>Voter: Check Outlier Thresholds: |A - B|, |B - C|, |C - A|
    
    alt Outlier Detected (e.g. Lane B Divergence)
        Voter->>LaneB: Assert Fault Latch (Trip Lane B)
        Voter->>Act: Dispatch Duplex Average (Lane A + Lane C)
    else Nominal Agreement
        Voter->>Act: Dispatch Voted Commands
    end
```

---

## 4. Flight Control Law State Transitions

```mermaid
stateDiagram-v2
    [*] --> NormalLaw: Power-On Self Test Passed (3 Lanes, 3 ADIRS)
    
    NormalLaw --> AlternateLaw: Loss of 2 ADIRS OR Loss of 2 FCC Lanes
    AlternateLaw --> DirectLaw: Landing Gear Extended OR Complete Sensor Loss
    DirectLaw --> MechanicalBackup: Total Fly-by-Wire Electrical Bus Loss
    
    NormalLaw: Full Flight Envelope Protection Active
    NormalLaw: C* Pitch Blend Law (Nz + q)
    NormalLaw: Roll Rate Demand & Bank Angle Hold (33 deg)
    NormalLaw: Automatic Yaw Damper & Turn Coordination
    NormalLaw: Active High-AOA Alpha Protection & Alpha Floor
    NormalLaw: High-Speed VMO / MMO Nose-Up Override
    NormalLaw: Structural Load Factor Limiters (+2.5g / -1.0g)

    AlternateLaw: Stability Augmentation Maintained
    AlternateLaw: Roll Law Direct
    AlternateLaw: Stall Warning Horn & Overspeed Warning
    AlternateLaw: Auto-Trim Available

    DirectLaw: Direct Sidestick to Control Surface Mapping
    DirectLaw: Manual Trim Wheel Operation Required
    DirectLaw: All Protections Lost

    MechanicalBackup: Mechanical Cable Linkage to THS and Rudder Pedals
```

---

## 5. ARINC 664 (AFDX) Redundancy Management

Every avionics Ethernet frame is duplicated across two completely independent physical network planes (Network A and Network B). Receivers employ First-Valid sequence number acceptance to ensure zero packet loss during physical cable cuts or switch reboots.

```mermaid
flowchart LR
    subgraph Transmitter ["End System Transmitter"]
        App["Avionics Application"]
        Shaper["Token Bucket Shaper (BAG Regulation)"]
        Dup["Dual-PHY Frame Duplicator"]
    end

    subgraph NetworkA ["AFDX Network Plane A"]
        SwA["Switch Matrix A"]
    end

    subgraph NetworkB ["AFDX Network Plane B"]
        SwB["Switch Matrix B"]
    end

    subgraph Receiver ["End System Receiver"]
        Integ["Integrity Checker (Seq Num mod 256)"]
        FirstValid["First-Valid Redundancy Selector"]
        ActApp["Actuator Servo Controller"]
    end

    App --> Shaper
    Shaper --> Dup
    Dup -->|Packet Copy A| SwA
    Dup -->|Packet Copy B| SwB
    SwA --> Integ
    SwB --> Integ
    Integ --> FirstValid
    FirstValid -->|First Arrival Delivered| ActApp
    FirstValid -->|Redundant Duplicate Discarded| Discard["Drop Buffer"]
```
