#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from pathlib import Path
# from configs.config import *
import numpy as np


DEFAULT_BETA = 2.0
NORMAL_AXES = {
    "x": np.array([1.0, 0.0, 0.0], dtype=float),
    "y": np.array([0.0, 1.0, 0.0], dtype=float),
    "z": np.array([0.0, 0.0, 1.0], dtype=float),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=float),
    "-y": np.array([0.0, -1.0, 0.0], dtype=float),
    "-z": np.array([0.0, 0.0, -1.0], dtype=float),
}
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}
EPS = 1e-12


def wrap_angle_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def euler_to_rot(roll, pitch, yaw, angle_unit="rad"):
    """Match the existing project convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    if angle_unit == "deg":
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    return rz(yaw) @ ry(pitch) @ rx(roll)


def rot_to_euler(R, out_unit="deg"):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    if out_unit == "deg":
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    return roll, pitch, yaw


def normalize_quaternion(q):
    q = np.array(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm <= EPS:
        return np.full(4, float("nan"), dtype=float)
    return q / norm


def rot_to_quat(R):
    """Return a normalized quaternion in [w, x, y, z] order."""
    R = np.array(R, dtype=float)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ], dtype=float)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 0.0)) * 2.0
        q = np.array([
            (R[2, 1] - R[1, 2]) / s,
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
        ], dtype=float)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 0.0)) * 2.0
        q = np.array([
            (R[0, 2] - R[2, 0]) / s,
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
        ], dtype=float)
    else:
        s = math.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 0.0)) * 2.0
        q = np.array([
            (R[1, 0] - R[0, 1]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
        ], dtype=float)

    return normalize_quaternion(q)


def quaternion_geodesic_error_deg(q_gt, q_est):
    q_gt = normalize_quaternion(q_gt)
    q_est = normalize_quaternion(q_est)
    dot = float(abs(np.dot(q_gt, q_est)))
    if not np.isfinite(dot):
        return float("nan")
    dot = float(np.clip(dot, 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def rotation_loss(q_gt, q_est):
    q_gt = normalize_quaternion(q_gt)
    q_est = normalize_quaternion(q_est)
    if not np.all(np.isfinite(q_gt)) or not np.all(np.isfinite(q_est)):
        return float("nan")
    return float(min(
        np.linalg.norm(q_gt - q_est) ** 2,
        np.linalg.norm(q_gt + q_est) ** 2,
    ))


def make_transform(x, y, z, roll, pitch, yaw, angle_unit="rad"):
    T = np.eye(4, dtype=float)
    T[:3, :3] = euler_to_rot(roll, pitch, yaw, angle_unit=angle_unit)
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T


def invert_transform(T):
    Tinv = np.eye(4, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def convert_to_cvframe(T_frame):
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])
    return T_frame @ T


def pose_from_transform(T):
    x, y, z = T[:3, 3].tolist()
    roll, pitch, yaw = rot_to_euler(T[:3, :3], out_unit="deg")
    return x, y, z, roll, pitch, yaw


def compute_relative_transform(gt_row, angle_unit="deg"):
    T_world_robot = make_transform(
        gt_row["x_robot"], gt_row["y_robot"], gt_row["z_robot"],
        gt_row["roll_robot"], gt_row["pitch_robot"], gt_row["yaw_robot"],
        angle_unit=angle_unit,
    )
    T_world_rgbd_blender = make_transform(
        gt_row["x_rgbd"], gt_row["y_rgbd"], gt_row["z_rgbd"],
        gt_row["roll_rgbd"], gt_row["pitch_rgbd"], gt_row["yaw_rgbd"],
        angle_unit=angle_unit,
    )
    T_world_rgbd = convert_to_cvframe(T_world_rgbd_blender)
    return invert_transform(T_world_rgbd) @ T_world_robot


def normalize_frame_id(token):
    token = str(token).strip()
    path = Path(token)
    if path.suffix.lower() in IMAGE_EXTS:
        return path.stem
    return token


def parse_ground_truth_poses(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f, delimiter=" ", skipinitialspace=True)
        for row in reader:
            clean = {k: v for k, v in row.items() if k is not None}
            rows.append({
                "timestamp": normalize_frame_id(clean["timestamp"]),
                "x_robot": float(clean["x_robot"]),
                "y_robot": float(clean["y_robot"]),
                "z_robot": float(clean["z_robot"]),
                "roll_robot": float(clean["roll_robot"]),
                "pitch_robot": float(clean["pitch_robot"]),
                "yaw_robot": float(clean["yaw_robot"]),
                "x_rgbd": float(clean["x_rgbd"]),
                "y_rgbd": float(clean["y_rgbd"]),
                "z_rgbd": float(clean["z_rgbd"]),
                "roll_rgbd": float(clean["roll_rgbd"]),
                "pitch_rgbd": float(clean["pitch_rgbd"]),
                "yaw_rgbd": float(clean["yaw_rgbd"]),
            })
    return rows


def parse_estimated_poses(path):
    rows_by_frame = {}
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip().split()
        runtime_idx = None
        for runtime_name in ("inference_runtime_seconds", "runtime_seconds", "runtime"):
            if runtime_name in header:
                runtime_idx = header.index(runtime_name)
                break
        # Some pose files append runtime as an unlabeled final column after yaw.
        if runtime_idx is None and len(header) > 7:
            runtime_idx = len(header) - 1
        for line in f:
            parts = line.strip().split()
            if len(parts) < 2:
                continue

            frame_id = normalize_frame_id(parts[0])
            if parts[1] == "ERROR":
                runtime = float("nan")
                if runtime_idx is not None and len(parts) > runtime_idx:
                    try:
                        runtime = float(parts[runtime_idx])
                    except ValueError:
                        pass
                rows_by_frame[frame_id] = {
                    "timestamp": frame_id,
                    "ok": False,
                    "error_msg": " ".join(parts[1:]),
                    "runtime": runtime,
                }
                continue

            try:
                rows_by_frame[frame_id] = {
                    "timestamp": frame_id,
                    "ok": True,
                    "error_msg": "",
                    "x_robot": float(parts[1]),
                    "y_robot": float(parts[2]),
                    "z_robot": float(parts[3]),
                    "roll_robot": float(parts[4]),   # MegaPose output is already degrees.
                    "pitch_robot": float(parts[5]),
                    "yaw_robot": float(parts[6]),
                    "runtime": float(parts[runtime_idx]) if runtime_idx is not None else float("nan"),
                }
            except (IndexError, ValueError):
                # Old failure rows may include traceback continuation lines. They are not frames.
                continue
    return rows_by_frame


def angle_to_deg(value, angle_unit):
    return math.degrees(value) if angle_unit == "rad" else value


def rotation_is_valid(roll_deg, pitch_deg, yaw_deg):
    vals = np.array([roll_deg, pitch_deg, yaw_deg], dtype=float)
    if not np.all(np.isfinite(vals)):
        return False
    R = euler_to_rot(roll_deg, pitch_deg, yaw_deg, angle_unit="deg")
    return (
        np.all(np.isfinite(R))
        and np.allclose(R.T @ R, np.eye(3), atol=1e-6)
        and np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    )


def prediction_validity(est_row):
    if est_row is None:
        return False, "missing_prediction"
    if not est_row.get("ok", False):
        return False, est_row.get("error_msg", "megapose_returned_no_pose")

    required = ["x_robot", "y_robot", "z_robot", "roll_robot", "pitch_robot", "yaw_robot"]
    try:
        pose_values = np.array([est_row[k] for k in required], dtype=float)
    except KeyError as exc:
        return False, f"missing_pose_value:{exc}"

    if not np.all(np.isfinite(pose_values)):
        return False, "non_finite_pose_value"
    if not rotation_is_valid(est_row["roll_robot"], est_row["pitch_robot"], est_row["yaw_robot"]):
        return False, "invalid_rotation"
    return True, ""


def viewing_angle_deg(T_camera_marker, marker_normal_local):
    t_gt = T_camera_marker[:3, 3].astype(float)
    t_norm = float(np.linalg.norm(t_gt))
    if not np.isfinite(t_norm) or t_norm <= EPS:
        return float("nan")

    normal = np.array(marker_normal_local, dtype=float)
    normal_norm = float(np.linalg.norm(normal))
    if not np.isfinite(normal_norm) or normal_norm <= EPS:
        return float("nan")
    normal = normal / normal_norm

    R_gt = T_camera_marker[:3, :3].astype(float)
    normal_camera = R_gt @ normal
    direction_marker_to_camera = -t_gt / t_norm
    cos_angle = float(np.clip(np.dot(normal_camera, direction_marker_to_camera), -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))


def nan_pose_fields(prefix):
    return {
        f"{prefix}_x": float("nan"),
        f"{prefix}_y": float("nan"),
        f"{prefix}_z": float("nan"),
        f"{prefix}_roll_deg": float("nan"),
        f"{prefix}_pitch_deg": float("nan"),
        f"{prefix}_yaw_deg": float("nan"),
    }


def pose_fields(prefix, pose):
    x, y, z, roll, pitch, yaw = pose
    return {
        f"{prefix}_x": x,
        f"{prefix}_y": y,
        f"{prefix}_z": z,
        f"{prefix}_roll_deg": roll,
        f"{prefix}_pitch_deg": pitch,
        f"{prefix}_yaw_deg": yaw,
    }


def compute_frame_metrics(gt_rows, est_rows_by_frame, beta, normal_axis, angle_unit):
    rows = []
    marker_normal_local = NORMAL_AXES[normal_axis]

    for gt in gt_rows:
        frame_id = gt["timestamp"]
        T_gt_camera_marker = compute_relative_transform(gt, angle_unit=angle_unit)
        gt_t = T_gt_camera_marker[:3, 3].astype(float)
        gt_R = T_gt_camera_marker[:3, :3].astype(float)
        gt_q = rot_to_quat(gt_R)
        gt_pose = pose_from_transform(T_gt_camera_marker)
        _, _, _, gt_roll, gt_pitch, gt_yaw = gt_pose

        est = est_rows_by_frame.get(frame_id)
        valid_prediction, invalid_reason = prediction_validity(est)
        runtime = float("nan")
        if est is not None:
            runtime = est.get("runtime", float("nan"))

        row = {
            "frame_id": frame_id,
            "valid_prediction": valid_prediction,
            "error_msg": invalid_reason,
            "beta": beta,
            "inference_runtime_seconds": runtime,
            "viewing_angle_deg": viewing_angle_deg(T_gt_camera_marker, marker_normal_local),
            "world_marker_x": gt["x_robot"],
            "world_marker_y": gt["y_robot"],
            "world_marker_z": gt["z_robot"],
            "world_marker_roll_deg": angle_to_deg(gt["roll_robot"], angle_unit),
            "world_marker_pitch_deg": angle_to_deg(gt["pitch_robot"], angle_unit),
            "world_marker_yaw_deg": angle_to_deg(gt["yaw_robot"], angle_unit),
            "world_camera_x": gt["x_rgbd"],
            "world_camera_y": gt["y_rgbd"],
            "world_camera_z": gt["z_rgbd"],
            "world_camera_roll_deg": angle_to_deg(gt["roll_rgbd"], angle_unit),
            "world_camera_pitch_deg": angle_to_deg(gt["pitch_rgbd"], angle_unit),
            "world_camera_yaw_deg": angle_to_deg(gt["yaw_rgbd"], angle_unit),
            **pose_fields("gt", gt_pose),
            **nan_pose_fields("est"),
            "dx_m": float("nan"),
            "dy_m": float("nan"),
            "dz_m": float("nan"),
            "translation_error_m": float("nan"),
            "geodesic_rotation_error_deg": float("nan"),
            "roll_error_deg": float("nan"),
            "pitch_error_deg": float("nan"),
            "yaw_error_deg": float("nan"),
            "translation_loss": float("nan"),
            "rotation_loss": float("nan"),
            "weighted_pose_loss": float("nan"),
        }

        if est is not None and est.get("ok", False):
            est_pose = (
                est.get("x_robot", float("nan")),
                est.get("y_robot", float("nan")),
                est.get("z_robot", float("nan")),
                est.get("roll_robot", float("nan")),
                est.get("pitch_robot", float("nan")),
                est.get("yaw_robot", float("nan")),
            )
            row.update(pose_fields("est", est_pose))

        if valid_prediction:
            est_t = np.array([est["x_robot"], est["y_robot"], est["z_robot"]], dtype=float) #type: ignore
            est_R = euler_to_rot(
                est["roll_robot"],  #type: ignore
                est["pitch_robot"], #type: ignore
                est["yaw_robot"],   #type: ignore
                angle_unit="deg",   
            )
            est_q = rot_to_quat(est_R)

            dt = est_t - gt_t
            dx, dy, dz = dt.tolist()
            translation_loss = float(dt @ dt)
            r_loss = rotation_loss(gt_q, est_q)
            geodesic_error_deg = quaternion_geodesic_error_deg(gt_q, est_q)
            roll_error_deg = abs(wrap_angle_deg(est["roll_robot"] - gt_roll)) #type: ignore
            pitch_error_deg = abs(wrap_angle_deg(est["pitch_robot"] - gt_pitch))    #type: ignore
            yaw_error_deg = abs(wrap_angle_deg(est["yaw_robot"] - gt_yaw))  #type: ignore

            row.update({
                "dx_m": dx,
                "dy_m": dy,
                "dz_m": dz,
                "translation_error_m": math.sqrt(translation_loss),
                "geodesic_rotation_error_deg": geodesic_error_deg,
                "roll_error_deg": roll_error_deg,
                "pitch_error_deg": pitch_error_deg,
                "yaw_error_deg": yaw_error_deg,
                "translation_loss": translation_loss,
                "rotation_loss": r_loss,
                "weighted_pose_loss": translation_loss + beta * r_loss,
            })

        rows.append(row)

    return rows


PER_FRAME_FIELDS = [
    "frame_id",
    "valid_prediction", "error_msg",
    "translation_error_m", "geodesic_rotation_error_deg",
    "roll_error_deg", "pitch_error_deg", "yaw_error_deg",
    "translation_loss", "rotation_loss", "beta", "weighted_pose_loss",
    "inference_runtime_seconds", "viewing_angle_deg",
    "dx_m", "dy_m", "dz_m",
    "gt_x", "gt_y", "gt_z", "gt_roll_deg", "gt_pitch_deg", "gt_yaw_deg",
    "est_x", "est_y", "est_z", "est_roll_deg", "est_pitch_deg", "est_yaw_deg",
    "world_marker_x", "world_marker_y", "world_marker_z",
    "world_marker_roll_deg", "world_marker_pitch_deg", "world_marker_yaw_deg",
    "world_camera_x", "world_camera_y", "world_camera_z",
    "world_camera_roll_deg", "world_camera_pitch_deg", "world_camera_yaw_deg",
]


def finite_stats(values):
    vals = np.array(values, dtype=float)
    vals = vals[np.isfinite(vals)]

    if vals.size == 0:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "variance": None,
            "std_dev": None,
            "min": None,
            "max": None,
            "p90": None,
        }

    return {
        "count": int(vals.size),
        "mean": float(np.mean(vals)),
        "median": float(np.median(vals)),
        "variance": float(np.var(vals)),
        "std_dev": float(np.std(vals)),
        "min": float(np.min(vals)),
        "max": float(np.max(vals)),
        "p90": float(np.percentile(vals, 90)),
    }


def threshold_key(threshold, unit):
    value = f"{threshold:g}".replace(".", "_").replace("-", "minus_")
    return f"le_{value}_{unit}"


def threshold_success_rates(values, thresholds, unit):
    vals = np.array(values, dtype=float)
    vals = vals[np.isfinite(vals)]
    results = {}

    for threshold in thresholds:
        key = threshold_key(threshold, unit)
        threshold_count = int(np.count_nonzero(vals <= threshold)) if vals.size else 0
        results[key] = {
            f"threshold_{unit}": float(threshold),
            "count": threshold_count,
            "percent": float(threshold_count / vals.size * 100.0) if vals.size else None,
        }

    return results


def summarize_rows(rows):
    valid = [r for r in rows if r["valid_prediction"]]
    total = len(rows)
    valid_count = len(valid)
    failed_count = total - valid_count
    return {
        "total_expected_frames": total,
        "valid_predictions": valid_count,
        "failed_predictions": failed_count,
        "pose_estimation_rate_percent": (valid_count / total * 100.0) if total else 0.0,
        "metrics": {
            # Error/loss summaries use valid predictions only.
            "translation_error_m": finite_stats([r["translation_error_m"] for r in valid]),
            "geodesic_rotation_error_deg": finite_stats([r["geodesic_rotation_error_deg"] for r in valid]),
            "roll_error_deg": finite_stats([r["roll_error_deg"] for r in valid]),
            "pitch_error_deg": finite_stats([r["pitch_error_deg"] for r in valid]),
            "yaw_error_deg": finite_stats([r["yaw_error_deg"] for r in valid]),
            "translation_loss": finite_stats([r["translation_loss"] for r in valid]),
            "rotation_loss": finite_stats([r["rotation_loss"] for r in valid]),
            "weighted_pose_loss": finite_stats([r["weighted_pose_loss"] for r in valid]),
            # Runtime and viewing angle are not estimation errors; keep all finite attempted/expected frames.
            "inference_runtime_seconds": finite_stats([r["inference_runtime_seconds"] for r in rows]),
            "viewing_angle_deg": finite_stats([r["viewing_angle_deg"] for r in rows]),
        },
        "threshold_success_rates": {
            "translation_error_m": threshold_success_rates(
                [r["translation_error_m"] for r in valid],
                thresholds=[0.01, 0.10, 0.25, 0.5],
                unit="m",
            ),
            "geodesic_rotation_error_deg": threshold_success_rates(
                [r["geodesic_rotation_error_deg"] for r in valid],
                thresholds=[1.0, 10.0, 30.0, 60.0],
                unit="deg",
            ),
        },
    }


def safe_name(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name)).strip("_") or "run"


def write_per_frame_csv(rows, output_dir, run_name):
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"per_frame_metrics_{safe_name(run_name)}.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=PER_FRAME_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_summary_json(summary, output_dir, run_name):
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"summary_metrics_{safe_name(run_name)}.json"
    path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return path


def resolve_inputs(args):
    gt_path = Path(args.gt_file)
    est_path = Path(args.est_file)
    output_dir = Path(args.output_dir) if args.output_dir else Path("error_outputs")
    return gt_path, est_path, output_dir


def run_analysis(args):
    gt_path, est_path, output_dir = resolve_inputs(args)
    gt_rows = parse_ground_truth_poses(gt_path)
    est_rows = parse_estimated_poses(est_path)

    rows = compute_frame_metrics(
        gt_rows,
        est_rows,
        beta=args.beta,
        normal_axis=args.normal_axis,
        angle_unit=args.angle_unit,
    )

    run_name = f'{args.tag}'
    summary = summarize_rows(rows)
    summary.update({
        "run_name": run_name,
        "beta": args.beta,
        "normal_axis": args.normal_axis,
        "marker_normal_local": NORMAL_AXES[args.normal_axis].tolist(),
        "angle_unit": args.angle_unit,
        "gt_file": str(gt_path),
        "est_file": str(est_path),
    })

    per_frame_path = write_per_frame_csv(rows, output_dir, run_name)
    summary_path = write_summary_json(summary, output_dir, run_name)

    print(f"Per-frame metrics CSV: {per_frame_path}")
    print(f"Summary JSON: {summary_path}")
    print(
        "Pose-estimation rate: "
        f"{summary['valid_predictions']}/{summary['total_expected_frames']} "
        f"({summary['pose_estimation_rate_percent']:.2f}%)"
    )
    return rows, summary, {"output_dir": output_dir, "per_frame_path": per_frame_path, "summary_path": summary_path}


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt-file", "--gt_file", dest="gt_file", required=True, help="Path to gt_poses_*.txt.")
    parser.add_argument("--est-file", "--est_file", dest="est_file", required=True, help="Path to est_poses_*.txt.")
    parser.add_argument("--beta", type=float, default=DEFAULT_BETA)
    parser.add_argument("--tag", type=str, required=True, help="Please describe entire experiment here")
    parser.add_argument("--output-dir", "--output_dir", dest="output_dir", default="error_outputs", help="Output directory.")
    parser.add_argument("--normal-axis", "--normal_axis", dest="normal_axis", choices=sorted(NORMAL_AXES.keys()), default="z")
    parser.add_argument(
        "--angle-unit",
        choices=["rad", "deg"],
        default="deg",
        help="Angle unit used in the ground-truth pose file.",
    )
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    run_analysis(args)


if __name__ == "__main__":
    main()
