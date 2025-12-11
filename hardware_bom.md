# Hardware Bill of Materials & Physical Setup
## Autonomous Mobile Navigator - ROSbot XL with Ouster OS1

---

## Required Hardware Components

### Core Components
1. **Husarion ROSbot XL** - Mobile robot platform
   - Includes built-in battery for base operation
   
2. **Ouster OS1 LiDAR** - 3D laser scanner
   - Requires: 15V power supply
   
3. **NVIDIA Jetson Nano/Orin** - Sensor processing computer
   - Requires: 20V power supply
   
4. **WiFi Router/Module** - Network hub for distributed system
   - Requires: USB-C 5V power supply

---

## Power System

### External Power Banks (2 Required)

**Power Bank #1 (Large):**
- Capacity: High-capacity (~20,000+ mAh recommended)
- Outputs: 15V and 20V
- Powers: Ouster LiDAR + Jetson

**Power Bank #2 (Small):**
- Capacity: Standard (~5,000 mAh)
- Output: USB-C 5V
- Powers: WiFi module

### Power Cables Required
- **15V cable:** Power Bank #1 → Ouster LiDAR
- **20V cable:** Power Bank #1 → Jetson
- **USB-C cable:** Power Bank #2 → WiFi Module

**Note:** ROSbot XL has its own internal battery - no external power needed.

---

## Network Cables

### Ethernet Cables (3 Required)

1. **Jetson ↔ WiFi Module**
   - Length: ≤ 1 foot (ideally 6 inches)
   
2. **Ouster LiDAR ↔ WiFi Module**
   - Length: ≤ 1 foot (ideally 6 inches)
   
3. **ROSbot XL ↔ WiFi Module**
   - Length: ≤ 1 foot (ideally 6 inches)

**Critical:** Short cable lengths required due to limited mounting space on ROSbot XL chassis.

---

## Physical Mounting Considerations

### Space Constraints
- ROSbot XL has **minimal mounting space**
- All components must be mounted compactly on top plate
- Cable management is critical - use velcro straps/zip ties

### Recommended Layout
```
     [Ouster LiDAR]
           |
    [WiFi Module (center)]
      /    |    \
  Jetson  ROSbot  Power Banks
```

### Weight Distribution
- Mount heavy components (power banks) low and centered
- LiDAR should be mounted highest for best field of view
- Ensure balanced weight to avoid tipping

---

## Assembly Checklist

- [ ] Mount all components on ROSbot chassis
- [ ] Connect Jetson to WiFi via Ethernet (≤1 ft cable)
- [ ] Connect Ouster to WiFi via Ethernet (≤1 ft cable)
- [ ] Connect ROSbot to WiFi via Ethernet (≤1 ft cable)
- [ ] Connect 15V cable: Power Bank → Ouster
- [ ] Connect 20V cable: Power Bank → Jetson
- [ ] Connect USB-C cable: Power Bank → WiFi Module
- [ ] Verify ROSbot internal battery is charged
- [ ] Secure all cables with cable management
- [ ] Test component power-on sequence
- [ ] Verify network connectivity (all on same WiFi)

---

## Power-On Sequence

1. **Power on ROSbot XL** (internal battery)
2. **Power on WiFi Module** (small power bank)
3. **Power on Jetson** (large power bank, 20V)
4. **Power on Ouster LiDAR** (large power bank, 15V)
5. **Wait 30 seconds** for all components to boot
6. **Verify network connectivity** on laptop

---

## Estimated Runtime

- **ROSbot Battery:** ~2-3 hours (motor operation)
- **Large Power Bank:** ~1.5-2 hours (LiDAR + Jetson)
- **Small Power Bank:** ~4-5 hours (WiFi module)

**Bottleneck:** Large power bank limits mission duration to ~1.5-2 hours.

---

## Notes & Warnings

⚠️ **Do NOT power Jetson or LiDAR from ROSbot's power supply** - insufficient capacity and may cause system instability.

⚠️ **Ethernet cable length is critical** - longer cables won't fit in compact mounting space.

---
