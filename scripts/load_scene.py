#!/usr/bin/env python3
"""Load a track scene into a running CoppeliaSim and select the ODE engine.

Why this exists
---------------
Headless CoppeliaSim (``coppeliaSim.sh -h``) ignores the scene given on its
command line. It starts, opens its remote API port, and sits there with an
empty scene -- and the simulator API then reports
``5:KO:SimComponent Loading Failure``, which reads like a broken car rather
than a missing track. The only way to load a scene headlessly is over the
remote API, which is what this does.

It also checks the scene afterwards. Two of the scenes shipped with the N4S
package are unusable -- one refuses to load at all, another loads without a
working lidar -- and finding that out from a silent failure costs an evening.

Usage
-----
    scripts/load_scene.py path/to/track.ttt
    COPPELIASIM_ROOT=~/opt/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04 \\
        scripts/load_scene.py ~/n4s-scenes/bootstrap_track.ttt

CoppeliaSim must already be running. Point --coppeliasim (or
COPPELIASIM_ROOT) at the install directory; the Python bindings ship inside
it and are not redistributed here.
"""

import argparse
import os
import shutil
import sys
import tempfile
import time

# Engine ids used by sim_intparam_dynamic_engine.
ENGINES = {"bullet": 0, "ode": 1, "vortex": 2, "newton": 3}

# The objects the simulator API looks up by name. A scene missing them will
# load without complaint and then fail every command.
REQUIRED_PREFIXES = ("motor_joint", "steer_joint", "checkpoint_sensor")


def find_bindings(root):
    """Assemble the remote API Python bindings into an importable directory.

    sim.py loads remoteApi.so from its *own* directory, but the install ships
    the two in different trees, so they have to be brought together. A
    throwaway directory keeps the CoppeliaSim install untouched.
    """
    py_dir = os.path.join(root, "programming", "remoteApiBindings", "python",
                          "python")
    lib_root = os.path.join(root, "programming", "remoteApiBindings", "lib",
                            "lib")
    if not os.path.isdir(py_dir):
        sys.exit("error: no Python bindings under %s\n"
                 "       is --coppeliasim pointing at the install root?"
                 % py_dir)

    lib = None
    for candidate in sorted(os.listdir(lib_root), reverse=True):
        path = os.path.join(lib_root, candidate, "remoteApi.so")
        if os.path.isfile(path):
            lib = path
            break
    if lib is None:
        sys.exit("error: no remoteApi.so under %s" % lib_root)

    staging = tempfile.mkdtemp(prefix="lidar-racer-rapi.")
    for name in ("sim.py", "simConst.py"):
        shutil.copy(os.path.join(py_dir, name), staging)
    shutil.copy(lib, staging)
    return staging


def scene_objects(sim, client):
    ret, _, _, _, names = sim.simxGetObjectGroupData(
        client, sim.sim_appobj_object_type, 0, sim.simx_opmode_blocking)
    return names if ret == 0 else []


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", nargs="?",
                        help="path to a .ttt scene file (omit with --check)")
    parser.add_argument("--check", action="store_true",
                        help="inspect the scene already open, load nothing. "
                             "Use this with the windowed CoppeliaSim, which "
                             "refuses to load a scene over the remote API")
    parser.add_argument("--coppeliasim",
                        default=os.environ.get("COPPELIASIM_ROOT"),
                        help="CoppeliaSim install root "
                             "(or set COPPELIASIM_ROOT)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=19997)
    parser.add_argument("--engine", default="ode", choices=sorted(ENGINES),
                        help="physics engine to select (default: ode)")
    args = parser.parse_args()

    if not args.coppeliasim:
        sys.exit("error: pass --coppeliasim or set COPPELIASIM_ROOT")
    if not args.check and not args.scene:
        sys.exit("error: give a scene path, or pass --check")
    scene = None
    if args.scene:
        scene = os.path.abspath(os.path.expanduser(args.scene))
        if not os.path.isfile(scene):
            sys.exit("error: no such scene: %s" % scene)
        # CoppeliaSim resolves this path itself, and its own launcher is known
        # to mishandle spaces. Refusing early beats a silent empty scene.
        if " " in scene:
            sys.exit("error: the scene path contains a space: %s\n"
                     "       CoppeliaSim mishandles those; move it elsewhere."
                     % scene)

    staging = find_bindings(os.path.expanduser(args.coppeliasim))
    sys.path.insert(0, staging)
    import sim

    sim.simxFinish(-1)
    client = sim.simxStart(args.host, args.port, True, True, 5000, 5)
    if client == -1:
        sys.exit("error: cannot reach CoppeliaSim on %s:%d -- is it running?"
                 % (args.host, args.port))

    if not args.check:
        if sim.simxLoadScene(client, scene, 0,
                             sim.simx_opmode_blocking) != sim.simx_return_ok:
            sim.simxFinish(client)
            sys.exit("error: CoppeliaSim refused to load %s\n"
                     "       Two causes: the scene file is unusable (try\n"
                     "       another), or this is the windowed CoppeliaSim,\n"
                     "       which refuses remote scene loads. In the window,\n"
                     "       use File > Open scene, then re-run with --check."
                     % os.path.basename(scene))
        time.sleep(3)

    names = scene_objects(sim, client)
    if not names:
        sim.simxFinish(client)
        sys.exit("error: no objects in the current scene -- nothing is loaded.")

    missing = [p for p in REQUIRED_PREFIXES
               if not any(n.startswith(p) for n in names)]
    checkpoints = sum(1 for n in names if n.startswith("checkpoint_sensor"))

    sim.simxSetIntegerParameter(client, sim.sim_intparam_dynamic_engine,
                                ENGINES[args.engine], sim.simx_opmode_blocking)
    engine = sim.simxGetIntegerParameter(client,
                                         sim.sim_intparam_dynamic_engine,
                                         sim.simx_opmode_blocking)[1]
    sim.simxFinish(client)
    shutil.rmtree(staging, ignore_errors=True)

    print("scene       %s" % (os.path.basename(scene) if scene
                                else "(already open)"))
    print("objects     %d" % len(names))
    print("checkpoints %d" % checkpoints)
    print("engine      %s" % args.engine
          if engine == ENGINES[args.engine] else "engine      REFUSED")
    if missing:
        print("WARNING: the scene is missing %s -- the API will not be able "
              "to drive it." % ", ".join(missing))
        return 1
    print("\nReady. Leave the simulation stopped; the AI starts it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
