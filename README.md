# Lizard Robot V1 — MuJoCo Locomotion

A MuJoCo simulation of the **V1 tail-assembly lizard robot** (a SolidWorks /
Dynamixel design). The robot has a 3-joint articulated spine plus four
single-DOF legs, and is driven by an **open-loop central-pattern-generator
(CPG) gait controller** configured entirely from one YAML file. Two control
laws are provided and selectable at runtime:

| Law | What it is | Best result (this repo) |
|-----|-----------|-------------------------|
| `sine` | Traveling-wave spine + antisymmetric sine legs (our optimized gait) | 0.62 m / 10 s, near-straight |
| `hardware` | Port of the real robot's MATLAB code: standing **C-wave** spine + duty-cycle **diagonal-trot** legs | 4.6 m / 10 s (free freq), 0.95 m / 16 s (locked 0.2 Hz) |

The full mathematical description of both control laws is in the companion
textbook (`docs/Lizard_Controller_Textbook.docx` / `.pdf`).

---

## 1. Robot at a glance

* **7 actuated joints**, all modeled as Dynamixel **XL430-W250-T** servos
  (position-controlled, torque clipped at the ±1.4 N·m stall torque).
  * Spine (lateral yaw): `Front`, `Back`, `Tail`
  * Legs (1-DOF, transverse-plane rotation): `FR`, `FL` (front girdle),
    `HR`, `HL` (rear girdle)
* **Total mass** ≈ 0.577 kg, body length ≈ 0.45 m.
* Sensors: IMU (accelerometer / gyro / velocimeter), base pose, per-joint
  position & velocity, per-actuator torque.
* World frame: **+X = forward**, **+Z = up**, **+Y = left**.

---

## 2. Installation (Anaconda)

The project is pure Python and runs on Windows, Linux, or macOS.

### 2.1 Install Anaconda / Miniconda
Download from <https://www.anaconda.com/download> (or Miniconda) and install.

### 2.2 Create and activate the environment

```bash
conda create -n lizard python=3.10 -y
conda activate lizard
```

### 2.3 Install the Python dependencies

```bash
pip install mujoco==3.9.0 numpy pyyaml "imageio[ffmpeg]" matplotlib optuna
```

| Package | Version used | Purpose |
|---------|--------------|---------|
| `mujoco` | **3.9.0** | physics engine + renderer |
| `numpy` | 2.2.x | math |
| `pyyaml` | latest | read `gait.yaml` |
| `imageio[ffmpeg]` | latest | write `.mp4` videos |
| `matplotlib` | latest | plot sensor logs |
| `optuna` | 4.9.x | gait parameter optimization |

> **MuJoCo ≥ 3.0 ships its own renderer and binaries** — there is no separate
> `mujoco-py` install and no `MUJOCO_KEY` license file needed.

### 2.4 Verify

```bash
python -c "import mujoco; print(mujoco.__version__)"   # -> 3.9.0
```

### 2.5 Headless rendering note (servers / WSL / CI)
On a normal desktop with a GPU the viewer and video rendering work out of the
box. On a headless Linux machine, render through a virtual display:

```bash
sudo apt-get install -y xvfb
MUJOCO_GL=glx xvfb-run -a python run.py --headless --duration 10 --video out.mp4
```

---

## 3. Repository layout

```
Lizard_Robot_MuJoCo/
├── README.md                     # this guide
├── run.py                        # main entry point (viewer / headless / video / logging)
├── configs/
│   └── gait.yaml                 # ALL gait parameters (single source of truth)
├── controllers/
│   ├── __init__.py
│   └── gait_controller.py        # the two control laws (sine + hardware)
├── models/
│   ├── lizard.xml                # MJCF scene: robot + floor + actuators + sensors + keyframe
│   └── meshes/*.STL              # CAD meshes
├── scripts/
│   ├── convert_urdf.py           # reproduce URDF -> MJCF conversion
│   ├── plot_results.py           # plot a results.npz
│   ├── render_topk.py            # render top-K optimizer trials
│   ├── optimize_search.py        # sine-mode: straight-line optimizer
│   ├── optimize_gait.py          # sine-mode: frequency × wave-speed sweep
│   ├── optimize_phase.py         # sine-mode: leg-body phase sweep
│   ├── optimize_hw.py            # hardware-mode: free-frequency optimizer
│   └── optimize_hw2.py           # hardware-mode: fixed-frequency optimizer
├── reference/
│   └── lizardCode_1DOF.m         # the real-robot MATLAB driver (read-only reference)
├── docs/
│   ├── Lizard_Controller_Textbook.docx   # full controller math (Word equations)
│   └── Lizard_Controller_Textbook.pdf
└── outputs/                      # generated: videos, run data, optimizer studies
    ├── videos/*.mp4
    ├── data/run_<timestamp>/{results.npz, summary.json, config.yaml}
    └── *.db                      # Optuna studies (resumable)
```

---

## 4. Running the simulation (`run.py`)

```bash
# 1) Interactive viewer (uses whatever is in configs/gait.yaml)
python run.py

# 2) Headless, 12 s, record a video + log data
python run.py --headless --duration 12 --video outputs/videos/test.mp4

# 3) Pick the camera
python run.py --headless --video side.mp4 --camera track_side   # chase cam
python run.py --headless --video top.mp4  --camera track_top    # top-down

# 4) Override ANY config value on the command line (dot-notation), no file edit
python run.py --set controller=hardware --set hardware.body_amp=0.8
python run.py --set controller=sine --set gait.frequency=1.0
```

**CLI flags**

| Flag | Default | Meaning |
|------|---------|---------|
| `--config PATH` | `configs/gait.yaml` | config file |
| `--duration S` | from config | sim length (s) |
| `--headless` | off | no viewer window |
| `--video PATH` | none | write an mp4 (implies offscreen render) |
| `--camera NAME` | `track_side` | `track_side` or `track_top` |
| `--fps N` | 30 | video framerate |
| `--set KEY=VALUE` | — | override a config value (repeatable) |
| `--no-save` | off | skip the `outputs/data` log |

**Headless outputs** land in `outputs/data/run_<timestamp>/`:
`results.npz` (full time series), `summary.json` (distance, speed, heading
drift, roll/pitch RMS, energy, cost of transport), and the exact `config.yaml`
used for that run.

Plot a run:

```bash
python scripts/plot_results.py outputs/data/run_<timestamp>/results.npz --save
```

---

## 5. The controller (`controllers/gait_controller.py`)

Both laws are **open-loop**: at every physics step the controller maps the
current time `t` to a position target for each of the 7 servos, then MuJoCo's
position actuators track those targets. A cosine **start-up ramp**
(`settle_time` → `ramp_time`) brings the motion in smoothly from the keyframe
pose. Choose the law with `controller: sine | hardware` in the YAML.

### 5.1 `sine` law — traveling wave
* **Spine** joint `i` (0 = head): a traveling sine whose phase advances by
  `2π·wavenumber·wave_speed/(N−1)` per segment, plus a constant `turn_trim`
  steering bias.
* **Legs**: each leg oscillates as a sine locked to its girdle's body phase,
  with **left = −right** (antisymmetric) and one global `phase_offset`.

### 5.2 `hardware` law — standing C-wave + duty-cycle trot
A port of the real robot's MATLAB driver, with the body changed to a
**standing wave**:
* **Spine**: every joint shares **one** sinusoid (no per-segment phase lag),
  so the whole body curls into a **C** and flips side each half-cycle. The
  C-shape requires `Front` to have the **opposite sign** of `Back`/`Tail`
  (same sign produces an S — see the textbook for the geometric reason).
* **Legs**: a **duty-cycle** function presses each foot down for a fraction
  `duty` of the cycle; the four legs run a **diagonal trot** set by two phase
  offsets, `phi_leg` (left/right) and `phi_lat` (front/back). With both = π
  the diagonal pairs are `FL+HR` and `FR+HL`.
* `body_lock` times the body bend against the foot contacts. At the tuned
  value, **bending left lands the left-front leg (FL+HR)** and bending right
  lands the right-front leg (FR+HL).

> The exact equations (with symbols, units, and derivations) are in
> `docs/Lizard_Controller_Textbook.docx` / `.pdf`.

---

## 6. Configuration reference (`configs/gait.yaml`)

```yaml
controller: hardware        # sine | hardware

gait:
  frequency: 0.2            # [Hz] sine-mode temporal frequency
  wave_speed: 1.0           # sine-mode spatial-propagation rate
  settle_time: 1.0          # [s] hold the start pose before motion
  ramp_time: 1.5            # [s] cosine amplitude ramp-in

spine:                      # SINE law
  amplitude: 0.94           # [rad]
  wavenumber: 0.83          # [waves along the body]
  turn_trim: 0.09           # [rad] constant yaw bias (drive straight)
  joints: [Front, Back, Tail]

legs:                       # SINE law
  amplitude: 0.95           # [rad]
  phase_offset: 0.67        # [cycles]
  right: [FR, HR]
  left:  [FL, HL]
  girdle: {FR: Front, FL: Front, HR: Back, HL: Back}

hardware:                   # HARDWARE (C-wave) law
  body_amp: 0.8             # [rad] C-bend amplitude
  body_lock: 2.234          # [rad] body-vs-leg phase
  body_shape: {Front: 1.0, Back: -1.0, Tail: -1.0}   # Front opposite => C (same sign => S)
  leg_amp: 0.564            # [rad] foot-down amplitude
  leg_freq: 0.2             # [Hz] body+leg frequency
  duty: 0.491               # stance fraction
  phi_leg: 3.14159          # [rad] left/right leg phase
  phi_lat: 3.14159          # [rad] front/back leg phase
  down_sign: {FR: -1, FL: -1, HR: 1, HL: -1}

offset:                     # per-joint initial angle [rad] (shared by both laws)
  Front: 0.0
  FR: -0.165
  FL: -0.165
  Back: 0.0
  HR: 0.165
  HL: -0.165
  Tail: 0.0

simulation:
  model: models/lizard.xml
  duration: 10.0
  keyframe: stand
```

> **Editor tip:** if `run.py` ever crashes with
> `yaml.reader.ReaderError: unacceptable character #x0000`, your editor saved
> trailing **null bytes** into `gait.yaml`. Strip them:
> `python -c "p='configs/gait.yaml';open(p,'w').write(open(p,'rb').read().replace(b'\x00',b'').decode())"`

---

## 7. Optimizing the gait

All optimizers load the MuJoCo model once, run many short **headless** physics
rollouts (no rendering), and use **Optuna** (TPE) with a **persistent SQLite
study** so they can be stopped and resumed. Each run accumulates trials into
its `outputs/*.db`; rerun the same command to continue.

### 7.1 Hardware-mode (current focus)

```bash
# Free frequency — maximize forward x (legs paddle, body C-bends)
python scripts/optimize_hw.py  --duration 8  --budget 38      # run ~one minute, resumable
python scripts/optimize_hw.py  --best                         # print best so far

# Frequency FIXED at gait.yaml's leg_freq — optimize everything else
python scripts/optimize_hw2.py --duration 16 --budget 38
python scripts/optimize_hw2.py --best
```

These lock the gait *identity* (C-shape, diagonal trot, `body_lock` near π so
the bend-side front leg lands) and tune `body_amp`, `leg_amp`, `duty`,
`body_lock`, and the initial leg posture.

### 7.2 Sine-mode optimizers

```bash
python scripts/optimize_search.py --duration 8 --budget 38    # straight-line: maximize fwd_x − |y| − |yaw|
python scripts/optimize_gait.py   --duration 6                # 2-D frequency × wave_speed grid
python scripts/optimize_phase.py  --duration 8                # leg-body phase sweep
```

### 7.3 Render the best trials

```bash
python scripts/render_topk.py --rank 0     # 0 = best; writes outputs/videos/top1_*.mp4
```

After optimizing, copy the printed best parameters into `configs/gait.yaml`
(and keep `models/lizard.xml`'s `stand` keyframe leg angles consistent with
the `offset` block to avoid a start-up jump).

---

## 8. Rebuilding the model from CAD

The checked-in `models/lizard.xml` was converted from the SolidWorks URDF
export. To regenerate it after a CAD change:

```bash
python scripts/convert_urdf.py        # writes models/lizard_body_dump.xml
```

then port the body-tree changes into `models/lizard.xml` (which adds the
floor, actuators, sensors, cameras, keyframe, and materials by hand).

---

## 9. Quick troubleshooting

| Symptom | Fix |
|---------|-----|
| `ReaderError: #x0000` on launch | strip null bytes from `gait.yaml` (see §6) |
| `unacceptable ... EGL` / black video on a server | use `MUJOCO_GL=glx xvfb-run -a ...` (§2.5) |
| robot curves instead of going straight (sine law) | tune `spine.turn_trim`, or run `optimize_search.py` |
| robot barely moves (hardware law) | increase `hardware.leg_freq` / `body_amp`, or run `optimize_hw.py` |
| start-up jump | make `models/lizard.xml` keyframe leg angles match `offset` |

---

## 10. Credits / references

* Real-robot gait reference: `reference/lizardCode_1DOF.m` (Dynamixel SDK,
  MATLAB).
* Physics: [MuJoCo](https://mujoco.org) 3.9.0.
* Optimization: [Optuna](https://optuna.org).
