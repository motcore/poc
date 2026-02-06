# Motcore

**Multi-Axis Friction Drive Actuator**

![License Hardware](https://img.shields.io/badge/license-CERN--OHL--P--2.0-blue)
![License Software](https://img.shields.io/badge/license-MIT-green)
![Status](https://img.shields.io/badge/status-early%20prototype-yellow)
![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen)

> A novel approach to multi-axis actuation: one motor, multiple independently controlled axes through selective friction engagement.

[🎥 Demo Video](#) | [📖 Documentation](docs/) | [💬 Discussions](../../discussions) | [🐛 Issues](../../issues)

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
- 🔄 3D-printable components (FreeCAD parametric models)
- 🔄 Compliant mechanism for engagement system
- 🔄 Comprehensive documentation
- 🔄 BOM with standard components

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

A single continuously rotating motor transmits power to multiple output axes through **servo-controlled friction wheels**. By selectively engaging/disengaging friction contact, each axis can be independently controlled without requiring its own motor.

```
     Friction Wheel A ← Central Motor (rotating) → Friction Wheel B
                                  ↑
                                  |
                           Output Axis
                      (moves laterally to engage A or B)
```

**Key principle:** Selective friction engagement as a power transmission method.

### Features

✅ **Single inexpensive motor** - 28BYJ-48 stepper (€3)  
✅ **Independent axis control** - 4+ axes, each controllable separately  
✅ **Simple mechanics** - no complex gearboxes or clutches  
✅ **Scalable** - add more axes without changing the central motor  
✅ **Low cost** - complete system ~€50 in components  
✅ **Educational platform** - demonstrates mechanical principles elegantly  

---

## 🧠 Design Philosophy

Motcore is inspired by principles of elegant mechanical design:

**Passive Dynamics** (McGeer)  
Like passive walkers that minimize active control, Motcore leverages fundamental physics - friction as a natural power transmission medium.

**Underactuation** (MIT Underactuated Robotics)  
One motor powers multiple independent degrees of freedom through selective engagement. Fewer actuators, smarter design.

**Mechanical Intelligence** (Jansen's Strandbeests)  
The mechanism's geometry and material properties do the "thinking", reducing software complexity.

**Compliant Mechanisms**  
The engagement system uses flexure-based compliance for smooth, reliable contact without complex joints.

### Why This Matters

Traditional multi-axis systems fight complexity with more complexity (more motors, more controllers, more synchronization). Motcore embraces the opposite: **simplicity through smart exploitation of physical phenomena**.

This isn't just about cost - it's about design elegance.

---

## 🔧 Current Prototype: Arduino Cube

![Motcore v0.1 Prototype](docs/images/prototype-v01.jpg)
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

### How It Works

1. **Central motor rotates continuously** at constant speed
2. **Servos position friction wheels** to engage/disengage with output axes
3. **Output axes move laterally** to contact different sides of the central shaft
4. **Direction control** through selective engagement (clockwise/counterclockwise)
5. **Independent operation** - each axis controlled separately via touchscreen

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
git clone https://github.com/motcore/arduino-cube.git
cd arduino-cube

# Open in PlatformIO
# OR open software/controller/ and software/driver/ in Arduino IDE
```

### Quick Start

1. **Upload firmware**
   - Upload `software/controller/` to master Arduino
   - Upload `software/driver/` to receiver Arduino

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
| [Assembly Guide](docs/assembly.md) | Step-by-step build instructions |
| [Wiring Diagram](docs/wiring.md) | Electrical connections and pinout |
| [Firmware Guide](docs/firmware.md) | Code architecture and customization |
| [3D Models](docs/3d-models.md) | FreeCAD parametric design (v0.2) |
| [Troubleshooting](docs/troubleshooting.md) | Common issues and solutions |

---

## 🗺️ Roadmap

### v0.2 - 3D Printed Prototype (In Progress)
- [ ] Parametric FreeCAD models of all components
- [ ] Compliant mechanism for engagement system
- [ ] Standard mechanical components (bearings, rods)
- [ ] Improved friction wheel design (TPU or O-rings)
- [ ] Complete assembly documentation

### v0.3 - Refinement
- [ ] Closed-loop position feedback
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
- [Contributors will be listed here as they join]

---

## 📞 Contact & Community

- **GitHub Issues:** [Bug reports and feature requests](../../issues)
- **GitHub Discussions:** [Questions, ideas, and general chat](../../discussions)
- **Email:** [opcional - tu email]
- **Hackaday.io:** [cuando lo crees]
- **LinkedIn:** [opcional - tu perfil]

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
Friction allows for variable torque transmission, compliance, and simpler engagement/disengagement. It's mechanically elegant but has trade-offs in efficiency and precision.

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

![GitHub stars](https://img.shields.io/github/stars/motcore/arduino-cube?style=social)
![GitHub forks](https://img.shields.io/github/forks/motcore/arduino-cube?style=social)
![GitHub watchers](https://img.shields.io/github/watchers/motcore/arduino-cube?style=social)
![GitHub issues](https://img.shields.io/github/issues/motcore/arduino-cube)
![GitHub pull requests](https://img.shields.io/github/issues-pr/motcore/arduino-cube)

---

**Motcore** - Making multi-axis actuation accessible through elegant design  
© 2025 Javier Asensio Montiel | Open Source Hardware | CERN-OHL-P-2.0

---

*"Perfection is achieved not when there is nothing more to add, but when there is nothing left to take away."*  
*- Antoine de Saint-Exupéry*
