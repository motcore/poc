# Contributing to Motcore

First of all — thank you for your interest! Motcore is a volunteer-driven open hardware project. Every contribution, big or small, is appreciated.

---

## Ground Rules

1. **No pressure, no deadlines** — contribute at your own pace
2. **Open from day one** — all contributions are immediately open source
3. **Credit always given** — contributors are acknowledged in the README
4. **Discuss first** — open an issue before starting major work to avoid duplication
5. **Be kind** — respectful and constructive collaboration only

---

## How to Contribute

### 🐛 Reporting Bugs

Open a [GitHub Issue](../../issues) with:
- A clear description of the problem
- Steps to reproduce it
- Your hardware/software setup
- Photos or Serial Monitor output if relevant

### 💡 Suggesting Ideas

Open a [GitHub Discussion](../../discussions) or an Issue tagged `enhancement`. Describe:
- The problem you're trying to solve
- Your proposed solution
- Any trade-offs you've considered

### 🔧 Engineering & CAD

- Work with FreeCAD (parametric models preferred)
- Follow the naming convention: `ComponentName_vX.X.FCStd`
- Export a STEP file alongside the native FreeCAD file
- Add a screenshot or render to `docs/images/`
- See [3D Models](docs/3d-models.md) for the current design status

### 💻 Firmware & Software

- Follow the existing code style
- Comment non-obvious logic
- Test on real hardware before submitting
- Keep commits focused and atomic

### 📝 Documentation

- Fix typos, improve clarity, add examples
- Translate documentation to other languages
- Create diagrams, illustrations, or videos

### 🧪 Testing & Feedback

- Build the prototype and share your results
- Test different friction materials and report findings
- Measure and share performance data (speed, torque, repeatability)

---

## Workflow

1. **Fork** the repository
2. **Create a branch** with a descriptive name (`feature/compliant-mechanism`, `fix/servo-calibration`)
3. **Make your changes**
4. **Test** your changes
5. **Open a Pull Request** with a clear description of what you've done and why

---

## Commit Style

Use clear, descriptive commit messages:

```
feat: add closed-loop position feedback for axis 1
fix: correct servo angle calculation for CCW direction
docs: add wiring diagram for ULN2003 connection
cad: add parametric FreeCAD model for friction wheel
```

---

## Questions?

Open a [Discussion](../../discussions) — no question is too basic.
