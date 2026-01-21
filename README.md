# Flexiv Interface

A lightweight Python interface and examples for Flexiv Rizon series robots. This repository provides a simple wrapper around the Flexiv SDK ([flexivrdk](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)) for reading robot states, joint and Cartesian motion control, admittance/force-control experiments, IK utilities, and demo scripts for quick verification and development.

## Quick Highlights

* High-level wrapper: [flexiv\_interface.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) — joint & Cartesian control threads, state access, force-torque sensor handling.
* Admittance & force-keeping examples: [admittance](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).
* Multiple IK solver implementations in [IK](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).
* Demo scripts and small examples under [demo](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) and [example\_py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).
* URDF and mesh resources under [resources](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).

## Requirements

* Python 3.8+ (recommended)
* Conda (Miniconda or Anaconda)
* Packages (examples):
  * numpy, scipy, matplotlib
  * pinocchio, example-robot-data (only for admittance simulation)
  * flexivrdk (Flexiv official SDK — install per vendor instructions)
  * Optional: casadi (if using [casadi\_ik.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)), redis, ros2 (if using visualization)

Note: [flexivrdk](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) is vendor-provided and may require separate installation (wheel or vendor SDK). Admittance simulation requires [pinocchio](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) and [example-robot-data](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) which are optional for hardware usage.

## Conda Environment (recommended)

Create and activate a conda environment for this project:

```

# create env with specific python version
conda create -n flexiv python=3.10 -y
conda activate flexiv
pip install flexivrdk==1.7.0 pin==2.6.20 pin-pink numpy scipy pyrobotiqgripper
pip install -e .

# if you want to run admittance control simulation
pip install example_robot_data==4.3.0

#if you want to debug with visualization, install ros2 humble, and follow the redis install instruction
```

## Quick Start — Examples

Run a simple read/command example (real robot required for hardware calls):

```
python test/test_teml.py
```

## Files & Folders (overview)

* [flexiv\_interface.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : Flexiv SDK wrapper. Methods include:
  * [get\_joint\_position()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [get\_tcp\_pose()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [get\_external\_wrench\_world()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), etc.
  * [start\_joint\_position\_control()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [start\_cartesian\_control()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [set\_joint\_target\_positions()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [set\_cartesian\_target\_pose()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).
  * [zero\_ft\_sensor()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), [move\_to\_home()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) utility methods.
* [admittance](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) :
  * [admittance.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : an admittance controller and simulation example (uses Pinocchio/example-robot-data).
  * [force\_keeping.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html), `x_admittance.py` : related controllers/helpers.
* [IK](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : inverse kinematics solvers (`casadi_ik.py`, `mink_ik.py`, [pink\_ik.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)).
* [demo](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) :
  * [press\_desk.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : force-keeping demo that uses admittance & IK.
* [example\_py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : small runnable examples:
  * [test\_interface.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : read sensor, Cartesian/joint control demos.
  * other test scripts for quick verification.
* [utils.py](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : utilities (LPF, safety barrier, pose helpers).
* [resources](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) : URDF and meshes for Rizon variants.

## Safety & Usage Notes

* Always verify software with the robot powered and in a safe environment (no people in the danger zone).
* Zero the force/torque sensor before running force-control demos: call [zero\_ft\_sensor()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) in [FlexivInterface](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html).
* Start with very small motions and low velocities when testing hardware control.
* [flexivrdk](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) requires proper network and controller-side setup — follow Flexiv documentation.

## Contributing

* Pull requests and issues are welcome. Provide minimal reproducible examples for bugs or behavior changes.
* Keep code style consistent and add small tests or example scripts when adding features.
