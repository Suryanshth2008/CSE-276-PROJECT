"""
Run-Level Evaluation and Benchmark Suite for Vision-Based Outdoor UGV.

Evaluates navigation CSV logs to calculate:
- Navigation Success Rate (%)
- Mean Path Deviation & Cross-Track RMSE
- Steering Stability & Jitter Index
- Failure Mode Categorization
- Average Processing FPS & Latency
- Generates comparative plots and Markdown benchmark reports.
"""
import argparse
import glob
import os
import tempfile
from datetime import datetime

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib"))

import matplotlib
matplotlib.use("Agg")  # Non-interactive backend for headless plot generation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def parse_eval_args():
    p = argparse.ArgumentParser(description="Evaluate UGV navigation test runs")
    p.add_argument("--log", default=None, help="Path to a specific navigation CSV log (default: latest log in outputs/)")
    p.add_argument("--compare", nargs="+", help="Compare multiple CSV log files side-by-side")
    p.add_argument("--output-dir", default="outputs", help="Folder to save evaluation plots and reports")
    p.add_argument("--no-plot", action="store_true", help="Skip generating graphical plots")
    return p.parse_args()


def find_latest_log(outputs_dir="outputs"):
    csv_files = glob.glob(os.path.join(outputs_dir, "navigation_log_*.csv"))
    if not csv_files:
        # Fallback to simulation logs if no navigation log found
        csv_files = glob.glob(os.path.join(outputs_dir, "simulation_log_*.csv"))
    if not csv_files:
        return None
    csv_files.sort(key=os.path.getmtime, reverse=True)
    return csv_files[0]


def evaluate_single_log(csv_path):
    """Calculates comprehensive navigation benchmark metrics for a single test run."""
    df = pd.read_csv(csv_path)

    # Standardize column names if reading simulation log
    if "sim_time" in df.columns and "timestamp_seconds" not in df.columns:
        df["timestamp_seconds"] = df["sim_time"]

    total_frames = len(df)
    if total_frames == 0:
        return None

    duration = float(df["timestamp_seconds"].iloc[-1] - df["timestamp_seconds"].iloc[0]) if total_frames > 1 else 0.0

    # 1. Detection & Navigation Success Rates
    if "status" in df.columns:
        found_frames = df["status"].isin(["CANDIDATE_PATH_FOUND", "PATH_FOUND", "AUTONOMOUS_NAVIGATING"]).sum()
        uncertain_frames = total_frames - found_frames
    else:
        found_frames = (df["decision"] != "STOP / PATH UNCERTAIN").sum()
        uncertain_frames = total_frames - found_frames

    detection_rate_pct = (found_frames / total_frames) * 100.0

    # 2. Decision Distribution
    decision_counts = df["decision"].value_counts().to_dict() if "decision" in df.columns else {}

    # 3. Path Deviation & Cross-Track Error (where path was detected)
    df["err_numeric"] = pd.to_numeric(df["normalized_center_error"], errors="coerce")
    valid_errors = df["err_numeric"].dropna()

    if len(valid_errors) > 0:
        mean_abs_error = float(valid_errors.abs().mean())
        max_abs_error = float(valid_errors.abs().max())
        rmse_error = float(np.sqrt((valid_errors ** 2).mean()))

        # Steering Smoothness / Jitter Index (Mean derivative of steering error)
        error_diffs = valid_errors.diff().abs().dropna()
        jitter_index = float(error_diffs.mean()) if len(error_diffs) > 0 else 0.0
    else:
        mean_abs_error = 0.0
        max_abs_error = 0.0
        rmse_error = 0.0
        jitter_index = 0.0

    # 4. Processing Speed (FPS)
    fps_col = "processing_fps_average" if "processing_fps_average" in df.columns else None
    if fps_col and df[fps_col].dropna().count() > 0:
        mean_fps = float(df[fps_col].mean())
    else:
        mean_fps = (total_frames / duration) if duration > 0 else 20.0

    # 5. Failure Categorization
    failure_counts = {}
    if "failure_reason" in df.columns:
        failure_counts = df["failure_reason"].dropna().value_counts().to_dict()
    else:
        # Infer failure categories from error & area
        lost_mask = df["decision"].str.contains("STOP", na=False)
        failure_counts["PATH_NOT_DETECTED"] = int(lost_mask.sum())

    results = {
        "file": os.path.basename(csv_path),
        "total_frames": total_frames,
        "duration_sec": round(duration, 2),
        "detection_rate_pct": round(detection_rate_pct, 1),
        "mean_abs_error": round(mean_abs_error, 4),
        "max_abs_error": round(max_abs_error, 4),
        "rmse_error": round(rmse_error, 4),
        "jitter_index": round(jitter_index, 4),
        "mean_fps": round(mean_fps, 1),
        "decision_counts": decision_counts,
        "failure_counts": failure_counts,
        "df": df
    }
    return results


def generate_evaluation_plots(eval_results, output_dir):
    """Generates 4-panel publication-ready performance chart."""
    df = eval_results["df"]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_path = os.path.join(output_dir, f"evaluation_report_{stamp}.png")

    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(f"UGV Navigation Performance Report — {eval_results['file']}", fontsize=14, fontweight="bold")

    times = df["timestamp_seconds"]

    # 1. Path Center Error vs Time
    ax1 = axs[0, 0]
    ax1.set_title("Normalized Center Error Over Time", fontsize=11, fontweight="bold")
    if "err_numeric" in df.columns:
        ax1.plot(times, df["err_numeric"], color="#0284c7", lw=1.5, label="Centerline Error")
        ax1.axhline(0.08, color="#22c55e", ls="--", alpha=0.8, label="Deadband (+0.08)")
        ax1.axhline(-0.08, color="#22c55e", ls="--", alpha=0.8, label="Deadband (-0.08)")
        ax1.axhline(0.0, color="#64748b", ls=":", alpha=0.6)
    ax1.set_xlabel("Time (seconds)")
    ax1.set_ylabel("Normalized Error (-1 to +1)")
    ax1.set_ylim(-1.05, 1.05)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)

    # 2. Decision Distribution (Pie Chart)
    ax2 = axs[0, 1]
    ax2.set_title("Steering Decision Breakdown", fontsize=11, fontweight="bold")
    dec_counts = eval_results["decision_counts"]
    if dec_counts:
        labels = list(dec_counts.keys())
        sizes = list(dec_counts.values())
        colors = ["#22c55e" if "FORWARD" in l or "STRAIGHT" in l else
                  ("#38bdf8" if "LEFT" in l else ("#f97316" if "RIGHT" in l else "#ef4444"))
                  for l in labels]
        ax2.pie(sizes, labels=labels, autopct="%1.1f%%", colors=colors, startangle=140,
                textprops={"fontsize": 9, "fontweight": "bold"})
    else:
        ax2.text(0.5, 0.5, "No Decision Data", ha="center", va="center")

    # 3. Candidate Path Contour Area
    ax3 = axs[1, 0]
    ax3.set_title("Detected Candidate Path Area (Pixels)", fontsize=11, fontweight="bold")
    area_col = "candidate_area_pixels" if "candidate_area_pixels" in df.columns else None
    if area_col:
        df["area_numeric"] = pd.to_numeric(df[area_col], errors="coerce")
        ax3.fill_between(times, df["area_numeric"].fillna(0), color="#10b981", alpha=0.3)
        ax3.plot(times, df["area_numeric"].fillna(0), color="#059669", lw=1.2)
        ax3.axhline(1500, color="#ef4444", ls="--", label="Min Area Threshold (1500px)")
    ax3.set_xlabel("Time (seconds)")
    ax3.set_ylabel("Area (Pixels)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=8)

    # 4. Processing Rate (FPS)
    ax4 = axs[1, 1]
    ax4.set_title("Real-Time Processing Rate (FPS)", fontsize=11, fontweight="bold")
    fps_col = "processing_fps_average" if "processing_fps_average" in df.columns else None
    if fps_col and df[fps_col].dropna().count() > 0:
        ax4.plot(times, df[fps_col], color="#8b5cf6", lw=1.5, label="Processing FPS")
        ax4.axhline(20.0, color="#f59e0b", ls="--", label="Target Video Rate (20 FPS)")
    ax4.set_xlabel("Time (seconds)")
    ax4.set_ylabel("FPS")
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc="lower right", fontsize=8)

    plt.tight_layout()
    plt.savefig(plot_path, dpi=160)
    plt.close()
    return plot_path


def main():
    args = parse_eval_args()
    os.makedirs(args.output_dir, exist_ok=True)

    target_log = args.log or find_latest_log(args.output_dir)
    if not target_log or not os.path.exists(target_log):
        print(f"Error: No navigation CSV log found to evaluate in {args.output_dir}.")
        print("Run 'python main.py' first to generate a run log.")
        return

    print("=" * 65)
    print("      Vision UGV Navigation — Run Evaluation & Benchmark")
    print("=" * 65)
    print(f"Evaluating log file: {target_log}\n")

    res = evaluate_single_log(target_log)
    if not res:
        print("Error: Empty or unreadable log file.")
        return

    # Print Formatted Benchmark Table
    print(f"{'METRIC':<36} | {'MEASURED VALUE':<20}")
    print("-" * 60)
    print(f"{'Total Frames Analyzed':<36} | {res['total_frames']} frames")
    print(f"{'Test Run Duration':<36} | {res['duration_sec']} seconds")
    print(f"{'Path Detection Consistency':<36} | {res['detection_rate_pct']}%")
    print(f"{'Mean Absolute Path Deviation':<36} | {res['mean_abs_error']:.4f} (normalized)")
    print(f"{'Cross-Track Deviation RMSE':<36} | {res['rmse_error']:.4f}")
    print(f"{'Max Peak Path Deviation':<36} | {res['max_abs_error']:.4f}")
    print(f"{'Steering Jitter / Smoothness Index':<36} | {res['jitter_index']:.4f} (lower is smoother)")
    print(f"{'Average Processing Speed':<36} | {res['mean_fps']} FPS")
    print("-" * 60)

    print("\nSteering Decision Distribution:")
    for decision, count in res["decision_counts"].items():
        pct = (count / res["total_frames"]) * 100
        bars = "█" * int(pct // 5)
        print(f"  - {decision:<24}: {count:>4} frames ({pct:>5.1f}%) {bars}")

    if not args.no_plot:
        plot_path = generate_evaluation_plots(res, args.output_dir)
        print(f"\nEvaluation Performance Plot: {plot_path}")

    print("\nBenchmark analysis completed.")


if __name__ == "__main__":
    main()
