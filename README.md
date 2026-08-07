# Motcore

**Multi-Axis Actuator Tree — one motor, many axes**

![License Hardware](https://img.shields.io/badge/license-CERN--OHL--P--2.0-blue)
![License Software](https://img.shields.io/badge/license-MIT-green)
![Status](https://img.shields.io/badge/status-early%20prototype-yellow)
![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen)

> A novel approach to multi-axis actuation: one motor, multiple independently controlled axes through selective friction engagement.

<!-- TODO: demo video — once it's recorded and hosted, restore as the first item:
     [🎥 Demo Video](URL) | -->
[📖 Documentation](docs/) | [🧬 Design Evolution](docs/design-evolution.md) | [💬 Discussions](../../discussions) | [🐛 Issues](../../issues)

---

## 🚧 Project Status

**Current version: v0.1 - Proof of Concept**

This is an **early-stage open hardware project** developed in spare time. The current prototype uses LEGO Technic components to validate the core mechanical principle. It works, it demonstrates the concept, but it's not production-ready.

**What exists today:**
- ✅ Functional proof of concept
- ✅ Arduino-based control system
- ✅ Validated friction-based power transmission
- ✅ Independent control of 4 output axes

**What's being developed:**
- 🔄 v5 vertical cone clutch — geometry settled, CAD pending
- 🔄 3D-printable components (FreeCAD parametric models)
- 🔄 Comprehensive documentation
- 🔄 BOM with standard components

The mechanism has been through **five generations**. Four are superseded and
should not be re-proposed — see [Design Evolution](docs/design-evolution.md) for
what each one was and what killed it.

**Timeline:** When it's ready. No pressure, no deadlines. Life happens. 🙂

---

## 💡 The Problem

Traditional multi-axis systems face a fundamental trade-off:

| Approach | Cost | Complexity | Independent Control |
|----------|------|------------|---------------------|
| **One motor per axis** | High | Low | ✅ Yes |
| **Mechanical coupling** | Medium | High | ❌ No (coupled motion) |
| **Motcore** | **Low** | **Medium** | **✅ Yes** |

**Motcore fills the gap:** affordable multi-axis control without sacrificing independence.

---

## 🎯 The Motcore Solution

### Core Innovation

A single continuously rotating motor drives multiple output axes through
**vertical conical friction clutches**. Each axis has its own clutch module on a
carriage that translates vertically; engaging a clutch connects that axis to the
motor cone. Each output feeds the **next hub of the tree**, so every level is a
reduction — torque is regenerated at each stage, and speed is the resource there
is plenty of.

```
        side view (one axis, Y-Z cross-section)

           motor axis (Z)
                │
                │        ┌───┐  ← output cone on carriage
                │       ╱     ╲       (translates vertically ↕)
                │      │  ○ ○  │ ── output shaft (fixed)
              ╱ │ ╲     ╲     ╱       + internal corona
             ╱  │  ╲     └───┘
            ╱ ○ │ ○ ╲   ← O-rings interleaved, flank to flank
           ╱────┴────╲
             motor cone
```

**Key principle:** selective friction engagement as a power transmission method —
with **free rotation obtained geometrically**, not by loosening a clutch.

### Features

✅ **Single inexpensive motor** — 28BYJ-48 stepper (€3)  
✅ **Independent axis control** — 4+ axes, each controllable separately  
✅ **Real free rotation** — a disengaged axis carries *nothing*, so an arm can fall under its own weight  
✅ **Reduction at every level** — 1.4× torque per stage, not 1:1  
✅ **Per-axis torque limiter** — slip threshold set in software by carriage position  
✅ **Scalable** — add more axes without changing the central motor  
✅ **Low cost** — complete system ~€50 in components  
✅ **Educational platform** — demonstrates mechanical principles elegantly  

---

## 🧠 Design Philosophy

Motcore is inspired by principles of elegant mechanical design:

**Passive Dynamics** (McGeer)  
Like passive walkers that minimize active control, Motcore leverages fundamental physics - friction as a natural power transmission medium.

**Underactuation** (MIT Underactuated Robotics)  
One motor powers multiple independent degrees of freedom through selective engagement. Fewer actuators, smarter design.

**Mechanical Intelligence** (Jansen's Strandbeests)  
The mechanism's geometry and material properties do the "thinking", reducing software complexity.

**Let the parts you already have do the work**  
The engagement stroke needs compliance to preload the friction surfaces. Rather
than add a spring, a slot or a flexure, v5 absorbs that travel in the **root
clearance of the gear it already needed**. No extra part, nothing to tune.

### Why This Matters

Traditional multi-axis systems fight complexity with more complexity (more motors, more controllers, more synchronization). Motcore embraces the opposite: **simplicity through smart exploitation of physical phenomena**.

This isn't just about cost - it's about design elegance.

---

## 🔧 Current Prototype: Arduino Cube

<!-- TODO: photo of the v0.1 LEGO prototype — save as docs/images/prototype-v01.jpg and restore:
     ![Motcore v0.1 Prototype](docs/images/prototype-v01.jpg) -->
*v0.1 proof of concept using LEGO Technic components*

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Central Motor** | 28BYJ-48 stepper (5V, ~3.5Ω coil resistance) |
| **Motor Driver** | ULN2003 Darlington array |
| **Output Axes** | 4 perpendicular axes (5mm diameter) |
| **Engagement** | 4x SG90 servo motors with friction wheels |
| **Controller** | Arduino Uno (master) |
| **Driver Board** | Arduino Uno (receiver) |
| **Interface** | MCUFRIEND 2.4" TFT touchscreen (240x320) |
| **Communication** | Serial (9600 baud) |
| **Power** | 5V USB |

### How It Works (v0.1 prototype)

1. **Central motor rotates continuously** at constant speed
2. **Servos position friction wheels** to engage/disengage with output axes
3. **Output axes move laterally** to contact different sides of the central shaft
4. **Direction control** through selective engagement (clockwise/counterclockwise)
5. **Independent operation** - each axis controlled separately via touchscreen

*This describes the LEGO proof of concept, which validated the core principle.
The engagement mechanism has been redesigned four times since — see
[Design Evolution](docs/design-evolution.md).*

---

## 🎓 Applications

### Immediate Use Cases
- **Educational robotics** - teach mechanical power transmission
- **Low-cost automation** - simple pick-and-place systems
- **Rapid prototyping** - test multi-DOF concepts quickly
- **Art installations** - kinetic sculptures with multiple motions
- **Research platform** - explore friction-based actuation

### Future Potential
- Multi-axis CNC plotters
- Compliant grippers with variable stiffness
- Bio-inspired locomotion (quadruped with 1 motor?)
- Haptic feedback devices
- Prosthetic control systems

---

## 📦 Bill of Materials (v0.1)

| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|-------|
| 28BYJ-48 Stepper Motor + ULN2003 | 1 | €3 | €3 |
| SG90 Micro Servo | 4 | €2 | €8 |
| Arduino Uno R3 | 2 | €10 | €20 |
| MCUFRIEND TFT Touchscreen 2.4" | 1 | €15 | €15 |
| Jumper wires, breadboard | - | €5 | €5 |
| **TOTAL** | | | **€51** |

*LEGO components not included - used for proof of concept only*

**v0.2 BOM (in development)** will replace LEGO with 3D printed + standard mechanical components.

---

## 🚀 Getting Started

### Prerequisites

**Software:**
- [PlatformIO](https://platformio.org/) (recommended) or [Arduino IDE](https://www.arduino.cc/en/software)
- [FreeCAD](https://www.freecad.org/) (for 3D models, optional)

**Hardware:**
- Components from BOM above
- 3D printer (optional - for v0.2 components)

### Installation

```bash
# Clone the repository
git clone https://github.com/Motcore/poc.git
cd poc

# Open in PlatformIO
# OR open src/controller/ and src/driver/ in Arduino IDE
```

### Quick Start

1. **Upload firmware**
   - Upload `src/controller/` to master Arduino
   - Upload `src/driver/` to receiver Arduino

2. **Wire according to schematic**
   - See [docs/wiring.md](docs/wiring.md) for pinout

3. **Assemble mechanical components**
   - See [docs/assembly.md](docs/assembly.md) for step-by-step

4. **Power on and test**
   - Use touchscreen interface to control axes

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [Design Evolution](docs/design-evolution.md) | The five generations, what killed each one, what survived |
| [Clutch Geometry](docs/clutch-geometry.md) | v5 derivation — cone angles, ratios, ring interleaving, constraints |
| [Assembly Guide](docs/assembly.md) | Step-by-step build instructions |
| [Wiring Diagram](docs/wiring.md) | Electrical connections and pinout |
| [Firmware Guide](docs/firmware.md) | Code architecture and customization |
| [3D Models](docs/3d-models.md) | FreeCAD parametric design (v0.2) |
| [Build Log](docs/build-log.md) | Prototype journal — BOM, prints, calibrations, tests |
| [Troubleshooting](docs/troubleshooting.md) | Common issues and solutions |

### 🛠️ Design Tools

| Tool | Description |
|------|-------------|
| **[v5 Vertical Cone Clutch Visualiser](https://htmlpreview.github.io/?https://github.com/motcore/poc/blob/main/cad/clutch_geometry_v5.html)** | **Active.** Interactive Y-Z cross-section. Live sliders for α, m, Zp/Zc, L, s₀, d, n, g, δ; the four carriage stops; an inset showing the pinion inside the corona; and live constraint checking. |
| [FreeCAD Macro — Calibration coupon](cad/calibration.py) | FDM tolerance calibration coupon. Print before anything else. |

<details>
<summary>Superseded design tools (kept for the record — do not build from these)</summary>

| Tool | Generation |
|------|------------|
| [Bevel Clutch Geometry Visualizer](https://htmlpreview.github.io/?https://github.com/motcore/poc/blob/main/cad/clutch_geometry.html) | v2 — bevel cone |
| [O-ring Clutch Geometry Visualizer](https://htmlpreview.github.io/?https://github.com/motcore/poc/blob/main/cad/clutch_geometry_v3.html) | v3/v4 — tilting flat disc |
| [FreeCAD Macro — Plates](cad/motcore_plates.py) | v2 — plates, motor cone, clutch cone |
| [FreeCAD Macro — Animation](cad/motcore_animate.py) | v2 — tilt animation around the UJ pivot |
| [FreeCAD Macro — v1 O-ring clutch](cad/motcore_v1.py) | v3 — printed compliant neck |
| [FreeCAD Macro — Compliant lever](cad/motcore_compliant_lever.py) | v4 — metal UJ + engagement blade + over-centre latch |

</details>

---

## 🔩 v5 Design Notes — Vertical Cone Clutch

Full derivation in [docs/clutch-geometry.md](docs/clutch-geometry.md); the
interactive visualiser is `cad/clutch_geometry_v5.html`.

### Mechanism

The motor cone sits on the central vertical shaft. The output cone rides a
**carriage that translates vertically** on two 3 mm guide rods — no tilt, no
pivot, no universal joint, one degree of freedom. Three ideas carry it:

**1. Rubber meets rubber.** The motor cone carries `n` O-rings on its
generatrix, the output cone `n − 1` offset by half a pitch, so the two sets
**interdigitate** and touch flank to flank. That gives μ ≈ 1.2–1.5 instead of
the 0.6–0.9 of rubber on plastic, and the rings never touch the plastic at all.
Preload is **radial between flanks**, so it is decoupled from apex displacement.

**2. Free rotation is geometric.** A pinion rigid to the output cone lives
permanently **inside** an internal corona fixed to the output shaft — captive by
construction, it can never fall out. Free = pinion **concentric** with the
corona, centre distance 0, so the output shaft carries *nothing*. No friction
threshold, which matters because lifting an arm and letting it fall demand the
same torque; no threshold can separate them.

**3. It reduces.** Friction 1.428 × gear 0.500 = **0.714**, i.e. 1.4× torque per
stage. The output feeds the next hub of the tree, so torque has to be
regenerated at every level.

### Four carriage positions, descending

| # | Position | State |
|---|----------|-------|
| 1 | **Free** | pinion concentric, rings separated, output shaft drives nothing |
| 2 | **Mesh** | centre distance `e` reached, rings **still separated**, relative velocity zero → teeth engage without shock |
| 3 | **Contact** | rubber flanks touch, normal force zero |
| 4 | **Preload** | flanks compressed, torque transmitted |

Preload travel is absorbed by the gear's own **root clearance** (0.25 · m) — no
slot, no floating ring, no compliant blade.

### Key Design Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motor cone half-angle α | 55° | from the vertical axis; output cone = 90 − α |
| Gear module m | 1.0 | |
| Pinion / corona teeth Zp / Zc | 14 / 28 | internal mesh, reducing |
| Contact line length L | 18 mm | along the generatrix |
| Apex → contact line s₀ | 10 mm | |
| O-ring wire d | 2.5 mm | rubber on rubber |
| Rings on motor cone n | 5 | output cone carries n − 1 |
| Margin g | 0.20 mm | full mesh → rubber contact |
| Preload δ | 0.25 mm | past contact |

### Derived

| Metric | Value |
|--------|-------|
| Friction ratio | 1.428 |
| Gear ratio | 0.500 |
| **Total ω_out / ω_motor** | **0.714 → 1.4× torque** |
| Centre distance `e` | 7.00 mm |
| Free float / mesh travel | 5.00 / 2.00 mm |
| **Total carriage stroke** | **7.45 mm** |
| Ring pitch q | 2.25 mm (< d ✓) |
| Apex separation | 1.33 mm |
| Micro-slip | ≈ 7.4 % |

### Open questions — not settled

- [ ] **Ring density vs micro-slip.** More rings interdigitate better but push
      the cones apart. n=5 → 7.4 %, n=7 → 13.6 %, n=3 → flanks never touch.
      Narrow window; needs a sweep over `L` and `d`.
- [ ] **`g + δ` exceeds root clearance at m = 1** (0.45 vs 0.25 mm). Unresolved.
- [ ] **Re-meshing while the output shaft still turns** clashes teeth. Firmware
      must gate engagement on an AS5600 velocity check.
- [ ] **Ring manufacture.** Catalogue O-rings fix the diameters; the pitches then
      have to land on the generatrix.
- [ ] **CAD.** Geometry is settled in the visualiser; the FreeCAD macro is not
      written yet.

### Position feedback is structural

Friction slip accumulates non-repeatably in series along a tree, so position is
only known by **measuring** it. An **AS5600** on every output shaft is part of
the mechanism, not an optional extra. Its I2C address is fixed at 0x36, so
multiple encoders need multiplexing (analog output, PWM, or a TCA9548A).

---

## 🗺️ Roadmap

### v0.2 - 3D Printed Prototype (In Progress)
- [ ] Parametric FreeCAD model of the v5 vertical cone clutch
- [ ] Carriage + 3 mm guide rods, single-DOF actuation
- [ ] Resolve `g + δ` vs root clearance
- [ ] Sweep `L` and `d` for the ring density / micro-slip window
- [ ] Source O-rings and land the ring pitches on the generatrix
- [ ] Complete assembly documentation

### v0.3 - Refinement
- [ ] AS5600 on every output shaft, with multiplexing
- [ ] Velocity-gated engagement (no re-mesh while the shaft turns)
- [ ] Torque measurement and control
- [ ] Multiple friction material testing
- [ ] Calibration procedures

### v1.0 - Production Ready
- [ ] Optimized BOM with sourcing links
- [ ] PCB design for integrated controller
- [ ] Comprehensive documentation
- [ ] Video tutorials
- [ ] Academic paper publication

### v2.0 - Advanced Features
- [ ] Scalable to 6+ axes
- [ ] Encoder feedback for precise positioning
- [ ] ROS integration
- [ ] Alternative engagement mechanisms

**Note:** These are aspirational goals, not commitments. Progress happens when contributors have time.

---

## 🤝 Contributing

**Contributions are welcome!** This project is developed by volunteers in their spare time.

### How to Contribute

We appreciate help in any form:

**🔧 Engineering & Design**
- FreeCAD models of components
- FEM analysis of compliant mechanisms
- Alternative engagement system designs
- Friction material testing

**📝 Documentation**
- Improve assembly instructions
- Create diagrams and illustrations
- Write troubleshooting guides
- Translate documentation

**💻 Software**
- Improve firmware efficiency
- Add new control modes
- Create calibration routines
- Develop GUI improvements

**🧪 Testing & Feedback**
- Build the prototype and report results
- Test different materials
- Measure performance metrics
- Share your modifications

### Contribution Guidelines

1. **No pressure, no deadlines** - contribute at your own pace
2. **Open from day one** - all work is immediately open source
3. **Credit always given** - contributors are acknowledged
4. **Discuss first** - open an issue before major work
5. **Be kind** - respectful collaboration always

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

---

## 📄 Licenses

This project uses multiple licenses for different components:

### Hardware & Mechanical Design
**CERN Open Hardware Licence Version 2 - Permissive (CERN-OHL-P-2.0)**

You are free to use, modify, and distribute the hardware designs, even commercially, as long as you provide attribution and include the license text.

[LICENSE-HARDWARE.md](LICENSE-HARDWARE.md)

### Software & Firmware
**MIT License**

You are free to use, modify, and distribute the software, even commercially, with minimal restrictions.

[LICENSE-SOFTWARE.md](LICENSE-SOFTWARE.md)

### Documentation
**Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)**

You are free to share and adapt the documentation, even commercially, as long as you provide attribution and distribute your contributions under the same license.

[LICENSE-DOCS.md](LICENSE-DOCS.md)

---

## 🙏 Acknowledgments

**Original concept and design:** Javier Asensio Montiel (2025)

**Inspired by:**
- Steve McGeer's work on passive dynamic walking
- Theo Jansen's Strandbeest kinetic sculptures
- Research on compliant mechanisms
- Underactuated robotics principles
- Piezoelectric actuator design philosophy

**Special thanks to:**
<!-- TODO: list contributors here as they join -->
- Contributors will be listed here as they join

---

## 📞 Contact & Community

- **GitHub Issues:** [Bug reports and feature requests](../../issues)
- **GitHub Discussions:** [Questions, ideas, and general chat](../../discussions)

<!-- TODO: add contact channels once they exist —
     - **Email:** <address>
     - **Hackaday.io:** <project URL>
     - **LinkedIn:** <profile URL>
-->


---

## 💬 FAQ

### Is this production-ready?
No. This is an early prototype demonstrating the concept. v0.1 uses LEGO components. v0.2 is being designed with proper mechanical parts.

### Can I build this myself?
Yes! The v0.1 BOM and basic instructions are available. v0.2 documentation is being developed.

### Will this be commercialized?
This is an open hardware project. Anyone can manufacture and sell kits, as long as they comply with the CERN-OHL-P license. The design will always remain open.

### How precise is the positioning?
Current prototype: low precision (proof of concept). v0.2+ will explore encoder feedback for improved accuracy.

### Can it handle heavy loads?
The current design is optimized for light loads (educational/prototyping). Load capacity depends on friction wheel material and contact pressure.

### Why friction drive instead of gears?
Friction allows variable torque transmission, compliance, and simple
engagement/disengagement — and it doubles as a per-axis torque limiter. The
trade-off is efficiency and precision: slip accumulates non-repeatably, which is
why v5 puts an encoder on every output shaft. v5 actually uses **both** — a
friction stage for engagement and torque limiting, and a gear stage for
reduction and for free rotation.

### How can a friction clutch give true free rotation?
It can't, and that's what killed four generations. Lifting an arm and letting it
fall demand the same torque at the shaft, so no slip threshold separates them.
v5 sidesteps it: free rotation comes from the **gear** stage, where the pinion
sits concentric inside the corona at centre distance 0 and the output shaft
carries nothing at all.

### Why does the design reduce instead of running 1:1?
Because each output feeds the next hub of a tree. Torque is consumed going down
and must be regenerated at every level; speed is the resource there is plenty of.

---

## ⭐ Support the Project

If Motcore is useful or interesting to you:

- ⭐ **Star this repository** - helps others discover the project
- 🐛 **Report bugs** - help improve reliability
- 💡 **Suggest ideas** - shape the project's direction  
- 🔧 **Contribute code/designs** - make it better
- 📖 **Improve documentation** - help others understand
- 📢 **Share the project** - spread the word
- 💰 **Sponsor development** (future: Open Collective or similar)

---

## 📊 Project Stats

![GitHub stars](https://img.shields.io/github/stars/Motcore/poc?style=social)
![GitHub forks](https://img.shields.io/github/forks/Motcore/poc?style=social)
![GitHub watchers](https://img.shields.io/github/watchers/Motcore/poc?style=social)
![GitHub issues](https://img.shields.io/github/issues/Motcore/poc)
![GitHub pull requests](https://img.shields.io/github/issues-pr/Motcore/poc)

---

**Motcore** - Making multi-axis actuation accessible through elegant design  
© 2025 Javier Asensio Montiel | Open Source Hardware | CERN-OHL-P-2.0

---

*"Perfection is achieved not when there is nothing more to add, but when there is nothing left to take away."*  
*- Antoine de Saint-Exupéry*
