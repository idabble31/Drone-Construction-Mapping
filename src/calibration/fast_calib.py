#!/usr/bin/env python3
"""
fast_calib_pipeline.py

Prototype script that drives the full fast-calib pipeline via subprocess.
This is the backend logic that the GUI will eventually wrap.

Usage:
    # Start a fresh container automatically:
    python3 fast_calib_pipeline.py

    # Attach to an already-running container by name:
    python3 fast_calib_pipeline.py --container fast_calib_session

    # With explicit bag/snap paths:
    python3 fast_calib_pipeline.py --bag /path/to/calib_target.bag --snap /path/to/snap_target.jpg

    # Skip straight to a specific step:
    python3 fast_calib_pipeline.py --container fast_calib_session --from-step filter
    python3 fast_calib_pipeline.py --container fast_calib_session --from-step calibrate

    # Multi-scene mode:
    python3 fast_calib_pipeline.py --container fast_calib_session --multi-scene
"""

import subprocess
import argparse
import shutil
import time
import re
import sys
import os
from dataclasses import dataclass, field
from pathlib import Path


# ──────────────────────────────────────────────
# CONFIG — edit these to match your environment
# ──────────────────────────────────────────────

class Config:
    DOCKER_IMAGE = "fast_calib:noetic"

    # Host paths (static config files)
    HOST_CONFIG_DIR     = Path("/home/scanar/scan_ar/src/calibration/fast_calib_docker")
    QR_PARAMS_PATH      = HOST_CONFIG_DIR / "config/test_qr_params.yaml"
    LAUNCH_FILE_PATH    = HOST_CONFIG_DIR / "launch/calib_robosense_single.launch"
    FILTER_SCRIPT_PATH  = HOST_CONFIG_DIR / "scripts/distance_filter_tool_rs.py"
    COMMON_LIB_PATH     = HOST_CONFIG_DIR / "include/common_lib.h"

    # Default input paths
    DEFAULT_BAG_PATH    = Path("/home/scanar/calibration_bags/calib_target.bag")
    DEFAULT_SNAP_PATH   = Path("/home/scanar/calibration_bags/snap_target.jpg")
    DEFAULT_OUTPUT_DIR  = Path("/home/scanar/calibration_bags/output")

    # Container-side paths
    CONTAINER_BAG            = "/root/data/calib_target.bag"
    CONTAINER_SNAP           = "/root/data/snap_target.jpg"
    CONTAINER_QR_PARAMS      = "/catkin_ws/src/FAST-Calib/config/test_qr_params.yaml"
    CONTAINER_LAUNCH         = "/catkin_ws/src/FAST-Calib/launch/calib_robosense_single.launch"
    CONTAINER_FILTER         = "/catkin_ws/src/FAST-Calib/scripts/distance_filter_tool_rs.py"
    CONTAINER_COMMON         = "/catkin_ws/src/FAST-Calib/include/common_lib.h"
    CONTAINER_FILTER_RESULT  = "/catkin_ws/src/FAST-Calib/target_lock_config.txt"
    CONTAINER_CALIB_RESULT   = "/catkin_ws/src/FAST-Calib/output/single_calib_result.txt"
    CONTAINER_OUTPUT_DIR     = "/catkin_ws/src/FAST-Calib/output"

    # Reloaded data paths — separate from volume-mounted paths to avoid
    # "device or resource busy" error when docker cp onto a bind mount
    CONTAINER_BAG_RELOADED   = "/root/data/calib_new.bag"
    CONTAINER_SNAP_RELOADED  = "/root/data/calib_new.jpg"

    # Recording
    RECORD_SCRIPT  = Path.home() / "scan_ar/src/calibration/record_calib_extrinsic.sh"
    RECORDED_BAG   = Path.home() / "calibration_bags/calib_lidar.bag"
    RECORDED_SNAP  = Path.home() / "calibration_bags/calib_image.jpg"


# ──────────────────────────────────────────────
# DATA MODELS
# ──────────────────────────────────────────────

@dataclass
class FilterResult:
    x_min: float = 0.0; x_max: float = 0.0
    y_min: float = 0.0; y_max: float = 0.0
    z_min: float = 0.0; z_max: float = 0.0

    def __str__(self):
        return (
            f"  x: [{self.x_min}, {self.x_max}]\n"
            f"  y: [{self.y_min}, {self.y_max}]\n"
            f"  z: [{self.z_min}, {self.z_max}]"
        )


@dataclass
class CalibResult:
    cam_model: str = ""
    cam_width: int = 0;  cam_height: int = 0
    fx: float = 0.0; fy: float = 0.0
    cx: float = 0.0; cy: float = 0.0
    d0: float = 0.0; d1: float = 0.0
    d2: float = 0.0; d3: float = 0.0
    Rcl: list = field(default_factory=list)   # 9 floats, row-major 3x3
    Pcl: list = field(default_factory=list)   # 3 floats
    rmse: float = -1.0
    scene_name: str = ""

    def is_valid(self):
        if len(self.Rcl) != 9 or len(self.Pcl) != 3:
            return False
        if len(set(round(v, 3) for v in self.Rcl)) <= 2:
            log("[WARN] Rcl matrix has near-identical values — calibration likely failed.")
            return False
        return True

    def __str__(self):
        rcl_fmt = ", ".join(f"{v:.6f}" for v in self.Rcl)
        pcl_fmt = ", ".join(f"{v:.6f}" for v in self.Pcl)
        rmse_str = f"{self.rmse:.4f} m" if self.rmse >= 0 else "N/A"
        return (
            f"  Scene:  {self.scene_name or 'unnamed'}\n"
            f"  Camera: {self.cam_model} {self.cam_width}x{self.cam_height}\n"
            f"  fx={self.fx} fy={self.fy} cx={self.cx} cy={self.cy}\n"
            f"  Rcl: [{rcl_fmt}]\n"
            f"  Pcl: [{pcl_fmt}]\n"
            f"  RMSE: {rmse_str}"
        )


# ──────────────────────────────────────────────
# LOGGING & PROMPTS
# ──────────────────────────────────────────────

def log(msg: str):
    print(f"[fast_calib] {msg}", flush=True)


def log_section(title: str):
    print(f"\n{'='*55}", flush=True)
    print(f"  {title}", flush=True)
    print(f"{'='*55}", flush=True)


def prompt_choice(prompt: str, choices: list) -> str:
    labels = {
        "save":     "Save this result and continue",
        "refilter": "Re-run distance filter (re-tune bounding box)",
        "recalib":  "Re-run calibration only (keep current filter)",
        "newdata":  "Record new calibration data and restart from filter",
        "quit":     "Quit without saving",
    }
    print("\n" + "-"*40)
    print(prompt)
    for i, key in enumerate(choices, 1):
        print(f"  [{i}] {labels.get(key, key)}")
    print("-"*40)

    while True:
        raw = input("Enter choice number: ").strip()
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(choices):
                return choices[idx]
        print(f"  Please enter a number between 1 and {len(choices)}.")


# ──────────────────────────────────────────────
# DOCKER HELPERS
# ──────────────────────────────────────────────

def find_running_container(name_or_id: str) -> str:
    result = subprocess.run(
        ["docker", "inspect", "--format={{.Id}} {{.State.Status}}", name_or_id],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        return ""
    parts = result.stdout.strip().split()
    if len(parts) == 2 and parts[1] == "running":
        return parts[0]
    log(f"[WARN] Container '{name_or_id}' exists but status is "
        f"'{parts[1] if len(parts) > 1 else 'unknown'}'")
    return ""


def resolve_container(name_or_id: str) -> str:
    log(f"Looking for running container: '{name_or_id}'...")
    container_id = find_running_container(name_or_id)
    if not container_id:
        log(f"[ERROR] No running container found matching '{name_or_id}'.")
        log("        Run 'docker ps' to see running containers.")
        return ""
    log(f"Found container: {container_id[:12]}")
    return container_id




# ──────────────────────────────────────────────
# PIPELINE STEPS
# ──────────────────────────────────────────────

def validate_inputs() -> bool:
    """Check that all host-side config files exist before running."""
    ok = True
    for path, label in [
        (Config.QR_PARAMS_PATH,     "qr_params.yaml"),
        (Config.LAUNCH_FILE_PATH,   "launch file"),
        (Config.FILTER_SCRIPT_PATH, "filter script"),
        (Config.COMMON_LIB_PATH,    "common_lib.h"),
    ]:
        if not path.exists():
            log(f"[ERROR] Missing {label}: {path}")
            ok = False
    return ok


def run_distance_filter(container_id: str,
                         bag_override: str = "") -> "FilterResult | None":
    """
    Runs distance_filter_tool_rs.py interactively inside the container.
    Opens an Open3D window — user picks 4 corners then presses Q.
    bag_override: use a different container-side bag path (e.g. after reload).
    """
    log("Running distance filter (interactive)...")
    log(">>> An Open3D window will open. Pick the 4 target corners, then press Q.")

    short_id = container_id[:12]
    bag_path = bag_override if bag_override else Config.CONTAINER_BAG
    cmd = [
        "docker", "exec",
        "--env", "DISPLAY",
        short_id,
        "bash", "-c",
        "source /opt/ros/noetic/setup.bash && "
        "source /catkin_ws/devel/setup.bash && "
        "cd /catkin_ws/src/FAST-Calib && "
        f"python3 {Config.CONTAINER_FILTER} {bag_path}"
    ]

    result = subprocess.run(cmd, timeout=None)
    if result.returncode != 0:
        log(f"[ERROR] Distance filter exited with code {result.returncode}")
        return None

    log("Distance filter complete. Parsing results...")

    read_result = subprocess.run(
        ["docker", "exec", short_id, "cat", Config.CONTAINER_FILTER_RESULT],
        capture_output=True, text=True
    )

    if read_result.returncode != 0:
        log(f"[ERROR] Could not read filter result: {read_result.stderr.strip()}")
        return None

    if not read_result.stdout.strip():
        log(f"[ERROR] Filter result file is empty.")
        return None

    log(f"Raw filter result:\n{read_result.stdout.strip()}")
    return parse_filter_result(read_result.stdout)


def parse_filter_result(text: str) -> "FilterResult | None":
    fields = {}
    pattern = re.compile(r"^(x_min|x_max|y_min|y_max|z_min|z_max):\s*([-\d.]+)", re.MULTILINE)
    for match in pattern.finditer(text):
        fields[match.group(1)] = float(match.group(2))

    expected = {"x_min", "x_max", "y_min", "y_max", "z_min", "z_max"}
    if not expected.issubset(fields.keys()):
        log(f"[ERROR] Filter result missing fields. Got: {list(fields.keys())}")
        return None

    return FilterResult(**fields)


def patch_qr_params(filter_result: FilterResult) -> bool:
    log("Patching qr_params.yaml with filter results...")
    try:
        content = Config.QR_PARAMS_PATH.read_text()
    except Exception as e:
        log(f"[ERROR] Could not read qr_params.yaml: {e}")
        return False

    for key, value in vars(filter_result).items():
        content, n = re.subn(
            rf"^({key}:\s*)[-\d.]+",
            rf"\g<1>{value}",
            content,
            flags=re.MULTILINE
        )
        if n == 0:
            log(f"[WARN] Key '{key}' not found in qr_params.yaml — appending.")
            content += f"\n{key}: {value}"

    try:
        Config.QR_PARAMS_PATH.write_text(content)
        log("qr_params.yaml updated successfully.")
        return True
    except Exception as e:
        log(f"[ERROR] Could not write qr_params.yaml: {e}")
        return False


def run_calibration(container_id: str) -> bool:
    """
    Launches fast-calib inside the container and streams logs.
    Kills the process as soon as a success or failure marker is found,
    avoiding the infinite ros::spin() loop.
    """
    log("Launching fast-calib (streaming logs)...")
    log("-" * 50)

    short_id = container_id[:12]
    cmd = [
        "docker", "exec", short_id,
        "bash", "-c",
        "source /opt/ros/noetic/setup.bash && "
        "source /catkin_ws/devel/setup.bash && "
        "roslaunch fast_calib calib_robosense_single.launch"
    ]

    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )

    success_marker = "calibration results saved to"
    failure_marker = "unable to find a candidate"
    found_result  = False
    found_failure = False

    for line in process.stdout:
        line = line.rstrip()
        print(line, flush=True)
        if success_marker.lower() in line.lower():
            found_result = True
        if failure_marker.lower() in line.lower():
            found_failure = True
        if found_result or found_failure:
            time.sleep(1)   # flush remaining log lines
            process.kill()
            break

    process.wait()
    log("-" * 50)

    if not found_result:
        if found_failure:
            log("[ERROR] Calibration failed — could not find 4 circle centers.")
            log("        → Try re-running distance filter or re-recording bags.")
        else:
            log("[ERROR] roslaunch exited unexpectedly.")
        return False

    return True


def collect_output(container_id: str, output_dir: Path,
                   scene_name: str = "") -> "CalibResult | None":
    """
    Reads single_calib_result.txt from host (via volume mount),
    or falls back to docker cp if no volume mount was set up.
    """
    result_path = output_dir / "single_calib_result.txt"

    if not result_path.exists():
        log("Result file not found on host. Trying docker cp...")
        output_dir.mkdir(parents=True, exist_ok=True)
        cp_result = subprocess.run(
            ["docker", "cp",
             f"{container_id[:12]}:{Config.CONTAINER_CALIB_RESULT}",
             str(result_path)],
            capture_output=True, text=True
        )
        if cp_result.returncode != 0:
            log(f"[ERROR] docker cp failed:\n{cp_result.stderr.strip()}")
            return None
        log("Copied result file via docker cp.")

    log(f"Parsing calibration result from {result_path}")
    content = result_path.read_text()
    calib = parse_calib_result(content)
    if calib is None:
        return None

    calib.scene_name = scene_name
    rmse_match = re.search(r"RMSE[:\s]+([\d.]+)", content)
    if rmse_match:
        calib.rmse = float(rmse_match.group(1))

    return calib


def save_scene_result(calib: CalibResult, output_dir: Path, scene_name: str):
    raw_src = output_dir / "single_calib_result.txt"
    raw_dst = output_dir / f"{scene_name}_result.txt"
    if raw_src.exists():
        shutil.copy(raw_src, raw_dst)
        log(f"Raw result saved to: {raw_dst}")

    yaml_path = output_dir / f"{scene_name}_extrinsic.yaml"
    save_calib_yaml(calib, yaml_path)
    log(f"Clean YAML saved to: {yaml_path}")


def parse_calib_result(text: str) -> "CalibResult | None":
    c = CalibResult()

    def find(pattern, cast=str, default=None):
        m = re.search(pattern, text)
        return cast(m.group(1)) if m else default

    c.cam_model  = find(r"cam_model:\s*(\w+)")
    c.cam_width  = find(r"cam_width:\s*(\d+)", int, 0)
    c.cam_height = find(r"cam_height:\s*(\d+)", int, 0)
    c.fx = find(r"cam_fx:\s*([-\d.]+)", float, 0.0)
    c.fy = find(r"cam_fy:\s*([-\d.]+)", float, 0.0)
    c.cx = find(r"cam_cx:\s*([-\d.]+)", float, 0.0)
    c.cy = find(r"cam_cy:\s*([-\d.]+)", float, 0.0)
    c.d0 = find(r"cam_d0:\s*([-\d.]+)", float, 0.0)
    c.d1 = find(r"cam_d1:\s*([-\d.e]+)", float, 0.0)
    c.d2 = find(r"cam_d2:\s*([-\d.e]+)", float, 0.0)
    c.d3 = find(r"cam_d3:\s*([-\d.e]+)", float, 0.0)

    rcl_match = re.search(r"Rcl:\s*\[(.+?)\]", text, re.DOTALL)
    if rcl_match:
        c.Rcl = [float(n) for n in re.findall(r"[-\d.]+", rcl_match.group(1))]

    pcl_match = re.search(r"Pcl:\s*\[(.+?)\]", text, re.DOTALL)
    if pcl_match:
        c.Pcl = [float(n) for n in re.findall(r"[-\d.]+", pcl_match.group(1))]

    if not c.cam_model:
        log("[ERROR] Could not parse calibration result.")
        return None

    return c


def save_calib_yaml(calib: CalibResult, path: Path):
    def fmt_matrix(vals, cols):
        rows = [vals[i:i+cols] for i in range(0, len(vals), cols)]
        return "\n" + "\n".join(
            "    - [" + ", ".join(f"{v:.6f}" for v in row) + "]"
            for row in rows
        )

    rmse_str = f"{calib.rmse:.4f}" if calib.rmse >= 0 else "N/A"
    path.write_text(f"""# fast-calib extrinsic calibration result
# Scene: {calib.scene_name or 'unnamed'}
# RMSE:  {rmse_str} m
# Generated by fast_calib_pipeline.py

camera:
  model: {calib.cam_model}
  width: {calib.cam_width}
  height: {calib.cam_height}
  intrinsics:
    fx: {calib.fx}
    fy: {calib.fy}
    cx: {calib.cx}
    cy: {calib.cy}
  distortion:
    d0: {calib.d0}
    d1: {calib.d1}
    d2: {calib.d2}
    d3: {calib.d3}

extrinsics:
  # Rotation matrix (LiDAR to Camera), row-major 3x3
  Rcl:{fmt_matrix(calib.Rcl, 3)}
  # Translation vector (LiDAR to Camera)
  Pcl: [{", ".join(f"{v:.6f}" for v in calib.Pcl)}]
""")


# ──────────────────────────────────────────────
# NEW DATA RECORDING
# ──────────────────────────────────────────────

def record_new_data() -> bool:
    """Runs record_calib_extrinsic.sh and waits for completion."""
    if not Config.RECORD_SCRIPT.exists():
        log(f"[ERROR] Record script not found at: {Config.RECORD_SCRIPT}")
        return False

    log_section("Recording New Calibration Data")
    log(f"Running: {Config.RECORD_SCRIPT}")
    log("This will take ~10 seconds + conversion time...")

    result = subprocess.run(
        ["bash", str(Config.RECORD_SCRIPT)],
        timeout=120
    )

    if result.returncode != 0:
        log(f"[ERROR] Record script exited with code {result.returncode}")
        return False

    ok = True
    for path, label in [(Config.RECORDED_BAG, "bag"), (Config.RECORDED_SNAP, "snap image")]:
        if path.exists():
            log(f"  ✓ {label}: {path}")
        else:
            log(f"  ✗ [ERROR] Missing {label}: {path}")
            ok = False

    return ok


def reload_data_into_container(container_id: str) -> bool:
    """Copies newly recorded bag and snap into the running container."""
    short_id = container_id[:12]
    log_section("Reloading New Data into Container")

    files = [
        (Config.RECORDED_BAG,  Config.CONTAINER_BAG_RELOADED),
        (Config.RECORDED_SNAP, Config.CONTAINER_SNAP_RELOADED),
    ]

    for host_path, container_path in files:
        if not host_path.exists():
            log(f"[ERROR] File not found on host: {host_path}")
            return False

        container_dir = str(Path(container_path).parent)
        subprocess.run(
            ["docker", "exec", short_id, "mkdir", "-p", container_dir],
            capture_output=True
        )

        log(f"Copying {host_path.name} → container:{container_path}")
        cp_result = subprocess.run(
            ["docker", "cp", str(host_path), f"{short_id}:{container_path}"],
            capture_output=True, text=True
        )
        if cp_result.returncode != 0:
            log(f"[ERROR] docker cp failed: {cp_result.stderr.strip()}")
            return False

    log("New data loaded into container successfully.")
    return True


def update_qr_params_paths():
    """Update bag_path and image_path in qr_params.yaml to container-side paths."""
    try:
        content = Config.QR_PARAMS_PATH.read_text()
        content = re.sub(r"^(bag_path:\s*).*",
                         rf"\g<1>{Config.CONTAINER_BAG_RELOADED}",
                         content, flags=re.MULTILINE)
        content = re.sub(r"^(image_path:\s*).*",
                         rf"\g<1>{Config.CONTAINER_SNAP_RELOADED}",
                         content, flags=re.MULTILINE)
        Config.QR_PARAMS_PATH.write_text(content)
        log("qr_params.yaml paths updated.")
    except Exception as e:
        log(f"[WARN] Could not update qr_params.yaml paths: {e}")


# ──────────────────────────────────────────────
# POST-CALIBRATION DECISION LOOP
# ──────────────────────────────────────────────

def post_calibration_loop(
    container_id: str,
    output_dir: Path,
    scene_name: str,
    calib_result: "CalibResult | None",
    bag_path: Path,
    snap_path: Path,
) -> tuple:
    """
    Interactive loop shown after every calibration attempt (success or failure).
    Returns (action, final_calib_result):
        "saved"   → result accepted and saved to disk
        "quit"    → user exited without saving
    """
    while True:
        log_section(f"Calibration Result — {scene_name}")

        if calib_result is not None:
            print(str(calib_result))
            if not calib_result.is_valid():
                log("[WARN] Result failed sanity check (suspicious Rcl values).")
            choices = ["save", "refilter", "recalib", "newdata", "quit"]
        else:
            log("[WARN] Calibration did not find 4 circle centers.")
            choices = ["refilter", "recalib", "newdata", "quit"]

        action = prompt_choice("What would you like to do?", choices)

        # ── Save ──
        if action == "save":
            save_scene_result(calib_result, output_dir, scene_name)
            log(f"Scene '{scene_name}' saved successfully.")
            return "saved", calib_result

        # ── Re-run distance filter then calibrate ──
        elif action == "refilter":
            log_section("Re-running Distance Filter")
            filter_result = run_distance_filter(container_id)
            if filter_result is None:
                log("[ERROR] Distance filter failed. Try again.")
                continue
            log(f"New filter result:\n{filter_result}")
            if not patch_qr_params(filter_result):
                continue

            log_section("Re-running Calibration")
            success = run_calibration(container_id)
            calib_result = collect_output(container_id, output_dir, scene_name) if success else None

        # ── Re-run calibration only (keep current filter) ──
        elif action == "recalib":
            log_section("Re-running Calibration (same filter)")
            success = run_calibration(container_id)
            calib_result = collect_output(container_id, output_dir, scene_name) if success else None

        # ── Record new data, reload, re-filter, re-calibrate ──
        elif action == "newdata":
            if not record_new_data():
                log("[ERROR] Recording failed. Check ROS2 is running and topics are available.")
                continue

            if not reload_data_into_container(container_id):
                log("[ERROR] Failed to load new data into container.")
                continue

            update_qr_params_paths()

            log_section("Re-running Distance Filter on New Data")
            filter_result = run_distance_filter(
                container_id, bag_override=Config.CONTAINER_BAG_RELOADED
            )
            if filter_result is None:
                log("[ERROR] Distance filter failed on new data.")
                calib_result = None
                continue
            log(f"New filter result:\n{filter_result}")
            if not patch_qr_params(filter_result):
                calib_result = None
                continue

            log_section("Re-running Calibration on New Data")
            success = run_calibration(container_id)
            calib_result = collect_output(container_id, output_dir, scene_name) if success else None

        # ── Quit ──
        elif action == "quit":
            log("Quitting without saving.")
            return "quit", None


# ──────────────────────────────────────────────
# STARTUP MENU
# ──────────────────────────────────────────────

def startup_menu(args) -> tuple:
    """
    Interactive startup menu shown at launch.
    Returns (action, args) where action is one of:
        "full"       — use existing bag/snap, run filter + calibrate
        "record"     — record new data first, then filter + calibrate
        "filter"     — skip to filter (data already in container)
        "calibrate"  — skip to calibrate (filter already done)
        "multiscene" — multi-scene mode
        "quit"       — exit
    """
    log_section("fast-calib Pipeline — Startup")

    print("  Container : " + (args.container or "(none — will start new)"))
    print("  Output    : " + str(args.output))
    print("  Scene     : " + args.scene_name)
    print()

    options = [
        ("filter",      "Filter + calibrate — run distance filter then calibrate"),
        ("calibrate",   "Calibrate only     — skip filter, re-run calibration with current params"),
        ("record",      "Record first       — grab new bag/snap, then filter + calibrate"),
        ("multiscene",  "Multi-scene mode   — record + calibrate 3 scenes for joint optimisation"),
        ("quit",        "Quit"),
    ]

    print("-"*60)
    print("  What would you like to do?")
    print("-"*60)
    for i, (key, desc) in enumerate(options, 1):
        print(f"  [{i}] {desc}")
    print("-"*60)

    while True:
        raw = input("Enter choice number: ").strip()
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(options):
                action = options[idx][0]
                log(f"Selected: {options[idx][1].split('—')[0].strip()}")
                return action, args
        print(f"  Please enter a number between 1 and {len(options)}.")


# ──────────────────────────────────────────────
# MAIN PIPELINE
# ──────────────────────────────────────────────

STEPS = ["filter", "calibrate"]


def run_pipeline(
    container: str,
    output_dir: Path,
    from_step: str = "filter",
    scene_name: str = "scene",
    interactive: bool = True,
) -> tuple:
    """
    Run the calibration pipeline against an already-running container.
    The container must be started separately before calling this.
    Returns (success, calib_result).
    """
    try:
        container_id = resolve_container(container)
        if not container_id:
            return False, None

        step_idx = STEPS.index(from_step) if from_step in STEPS else 0

        # ── Validate host-side config files ──
        log_section("Validating Config Files")
        if not validate_inputs():
            return False, None

        # ── STEP 1: Distance filter + patch yaml ──
        if step_idx <= STEPS.index("filter"):
            log_section("STEP 1: Running Distance Filter")
            filter_result = run_distance_filter(container_id)
            if filter_result is None:
                return False, None
            log(f"Filter result:\n{filter_result}")

            log_section("STEP 2: Patching qr_params.yaml")
            if not patch_qr_params(filter_result):
                return False, None
        else:
            log("=== Skipping distance filter (from_step=calibrate) ===")

        # ── STEP 2: Calibration ──
        if step_idx <= STEPS.index("calibrate"):
            log_section("STEP 3: Running fast-calib")
            success = run_calibration(container_id)
        else:
            log("=== Skipping calibration ===")
            success = True

        # ── STEP 3: Collect output ──
        log_section("STEP 4: Collecting Output")
        calib_result = collect_output(container_id, output_dir, scene_name) if success else None

        # ── POST-CALIBRATION DECISION LOOP ──
        if interactive:
            action, final_result = post_calibration_loop(
                container_id=container_id,
                output_dir=output_dir,
                scene_name=scene_name,
                calib_result=calib_result,
                bag_path=Config.RECORDED_BAG,
                snap_path=Config.RECORDED_SNAP,
            )
            if action == "saved":
                log_section("CALIBRATION COMPLETE")
                return True, final_result
            else:
                return False, None
        else:
            if calib_result is not None:
                save_scene_result(calib_result, output_dir, scene_name)
                if not calib_result.is_valid():
                    log("[WARN] Result saved but failed sanity check — review manually.")
                return calib_result.is_valid(), calib_result
            else:
                log("[ERROR] Calibration failed in non-interactive mode.")
                return False, None

    except KeyboardInterrupt:
        log("\nInterrupted by user.")
        return False, None

# ──────────────────────────────────────────────
# MULTI-SCENE MODE
# ──────────────────────────────────────────────

def run_multi_scene(container: str, output_dir: Path, num_scenes: int = 3):
    """
    Guide the user through N scenes for multi-scene calibration.
    Each scene uses record_new_data() to capture fresh data automatically.
    """
    scene_names = {1: "scene_forward", 2: "scene_right", 3: "scene_left"}
    scene_hints = {
        1: "Place target FACING the sensor directly (~1–1.5m away)",
        2: "Rotate target ~30–45° to the RIGHT",
        3: "Rotate target ~30–45° to the LEFT",
    }

    saved_results = []

    for i in range(1, num_scenes + 1):
        scene_name = scene_names.get(i, f"scene_{i}")
        scene_dir  = output_dir / scene_name
        scene_dir.mkdir(parents=True, exist_ok=True)

        log_section(f"SCENE {i} of {num_scenes}: {scene_name.upper()}")
        print(f"  Hint: {scene_hints.get(i, 'New scene placement')}")
        print(f"  Output will be saved to: {scene_dir}")
        input("\n  Press Enter when you are ready to record...")

        if not record_new_data():
            log("[ERROR] Recording failed.")
            retry = input("  Retry this scene? [Y/n] ").strip().lower()
            if retry != "n":
                i -= 1  # retry same scene
                continue
            else:
                break

        container_id = resolve_container(container)
        if not container_id:
            log("[ERROR] Container not found.")
            break

        if not reload_data_into_container(container_id):
            log("[ERROR] Failed to reload data into container.")
            break

        update_qr_params_paths()

        ok, result = run_pipeline(
            container=container,
            output_dir=scene_dir,
            from_step="filter",
            scene_name=scene_name,
            interactive=True,
        )

        if ok and result is not None:
            saved_results.append((scene_name, result))
            log(f"Scene {i} ({scene_name}) saved. ({len(saved_results)}/{num_scenes} complete)")
        else:
            skip = input(f"\n  Scene {i} not saved. Skip and continue? [y/N] ").strip().lower()
            if skip != "y":
                log("Aborting multi-scene run.")
                break

    # Summary
    log_section("MULTI-SCENE SUMMARY")
    if saved_results:
        print(f"  {len(saved_results)} scene(s) saved:")
        for name, result in saved_results:
            rmse_str = f"{result.rmse:.4f} m" if result.rmse >= 0 else "N/A"
            print(f"    • {name}: RMSE={rmse_str}")
        print()
        if len(saved_results) >= 3:
            log("You have 3+ scenes. Run multi_calib.launch for joint optimization:")
            print("    roslaunch fast_calib multi_calib.launch")
        else:
            log(f"You need {3 - len(saved_results)} more scene(s) for multi-scene calibration.")
    else:
        log("No scenes were saved.")


# ──────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="fast-calib pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single scene, fresh container:
  python3 fast_calib_pipeline.py

  # Single scene, attach to existing container:
  python3 fast_calib_pipeline.py --container heuristic_lewin

  # Skip straight to recalibration:
  python3 fast_calib_pipeline.py --container heuristic_lewin --from-step calibrate

  # Multi-scene mode (records + calibrates 3 scenes interactively):
  python3 fast_calib_pipeline.py --container heuristic_lewin --multi-scene

  # Non-interactive (auto-save, no menus):
  python3 fast_calib_pipeline.py --container heuristic_lewin --no-interactive
        """
    )
    parser.add_argument("--output",         type=Path, default=Config.DEFAULT_OUTPUT_DIR)
    parser.add_argument("--container",      type=str,  default="extrinsic_calibration")
    parser.add_argument("--from-step",      type=str,  default="filter", choices=STEPS)
    parser.add_argument("--scene-name",     type=str,  default="scene")
    parser.add_argument("--multi-scene",    action="store_true")
    parser.add_argument("--no-interactive", action="store_true")
    args = parser.parse_args()

    if not args.container:
        print("[ERROR] --container is required. Start the container first, then run this script.")
        print("        Example: docker run -d --rm --net=host --name fast_calib_session fast_calib:noetic tail -f /dev/null")
        sys.exit(1)

    # Show startup menu unless bypassed by CLI flags
    use_menu = (
        not args.no_interactive
        and args.from_step == "filter"
        and not args.multi_scene
    )

    if use_menu:
        action, args = startup_menu(args)
    elif args.multi_scene:
        action = "multiscene"
    else:
        action = args.from_step

    if action == "quit":
        log("Goodbye.")
        sys.exit(0)

    elif action == "multiscene":
        run_multi_scene(container=args.container, output_dir=args.output)
        sys.exit(0)

    elif action == "record":
        log_section("Recording New Data Before Pipeline")
        if not record_new_data():
            log("[ERROR] Recording failed. Exiting.")
            sys.exit(1)
        container_id = resolve_container(args.container)
        if not container_id:
            sys.exit(1)
        if not reload_data_into_container(container_id):
            log("[ERROR] Failed to reload data into container.")
            sys.exit(1)
        update_qr_params_paths()
        ok, _ = run_pipeline(
            container=args.container,
            output_dir=args.output,
            from_step="filter",
            scene_name=args.scene_name,
            interactive=not args.no_interactive,
        )
        sys.exit(0 if ok else 1)

    else:
        from_step = action if action in STEPS else "filter"
        ok, _ = run_pipeline(
            container=args.container,
            output_dir=args.output,
            from_step=from_step,
            scene_name=args.scene_name,
            interactive=not args.no_interactive,
        )
        sys.exit(0 if ok else 1)