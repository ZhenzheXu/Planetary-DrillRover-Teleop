#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Integrated rover experiment logger for STM32 USART6 LOG output.

Place this file in:
    /home/pi/Desktop/rover_experiment/rover_scene_logger_common.py

Data are saved in:
    /home/pi/rover_logs/<scene_name>/

Important:
    This version does NOT rely on newline characters from the STM32.
    It can correctly split glued serial data like:
        LOG,...,0LOG,...,0LOG,...
"""
import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

import serial
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"
DEFAULT_BAUD = 115200

COLUMNS = [
    "tag", "time_ms", "control_source",
    "lf_t", "lf_r", "rf_t", "rf_r", "lr_t", "lr_r", "rr_t", "rr_r",
    "dm3_online", "dm3_err", "dm3_rx_count", "dm3_pos_deg_x100", "dm3_vel_mrad_s",
    "dm3_torque_x100", "dm3_tmos", "dm3_trotor", "dm3_target_deg_x100", "stick_x",
    "zdt42_cmd_x10", "disc_cmd", "remote_ch0", "remote_ch1", "remote_ch3",
]

SCENE_CONFIG = {
    "flat_sand_demo": {
        "title": "Flat sand mobility and soil collection demonstration",
        "desc": "Forward/backward/turning, drill loosening, and collecting disc operation on flat sand.",
    },
    "slope_sand_demo": {
        "title": "Slope sand traversal demonstration",
        "desc": "Slope ascent/descent with arm posture and collecting disc assistance.",
    },
    "pit_escape_demo": {
        "title": "Pit escape demonstration",
        "desc": "Self-rescue using arm motion, drill/working mechanism, and wheel recovery.",
    },
}


class LogPacketExtractor:
    """
    Robust LOG packet extractor.

    The STM32 should ideally output one record per line:
        LOG,...\\r\\n

    But in real serial reading, records may be glued together:
        LOG,...,0LOG,...,0LOG,...

    This extractor uses the 'LOG,' marker itself as the packet boundary.
    """

    def __init__(self, expected_fields: int):
        self.expected_fields = expected_fields
        self.buffer = ""

    def feed(self, chunk: str):
        records = []

        if not chunk:
            return records

        # Normalize and remove control characters that may appear in serial streams.
        chunk = chunk.replace("\r", "").replace("\n", "")
        chunk = chunk.replace("\x00", "")

        self.buffer += chunk

        # Drop any leading garbage before the first LOG marker.
        first = self.buffer.find("LOG,")
        if first < 0:
            # Keep only a small tail in case "LOG," is split across chunks.
            self.buffer = self.buffer[-8:]
            return records
        if first > 0:
            self.buffer = self.buffer[first:]

        # Split before every LOG marker.
        parts = re.split(r"(?=LOG,)", self.buffer)

        # parts[0] may be empty. Keep the last segment in buffer because it may be incomplete.
        complete_parts = parts[:-1]
        self.buffer = parts[-1] if parts else ""

        for seg in complete_parts:
            rec = self.parse_segment(seg)
            if rec is not None:
                records.append(rec)

        return records

    def flush(self):
        rec = self.parse_segment(self.buffer)
        self.buffer = ""
        return [rec] if rec is not None else []

    def parse_segment(self, seg: str):
        seg = seg.strip()
        if not seg.startswith("LOG,"):
            return None

        parts = seg.split(",")

        # If a trailing glued record still leaked in, cut at the next LOG marker.
        # Usually this is already handled by split, but this keeps the parser safe.
        cleaned = []
        for p in parts:
            if "LOG" in p and p != "LOG":
                before, _, after = p.partition("LOG")
                if before:
                    cleaned.append(before)
                break
            cleaned.append(p)
        parts = cleaned

        if len(parts) < self.expected_fields:
            return None

        # Use exactly the expected number of fields. Extra bytes are ignored.
        parts = parts[:self.expected_fields]

        if parts[0] != "LOG":
            return None

        # Basic validation: time_ms should be numeric.
        try:
            int(float(parts[1]))
        except Exception:
            return None

        return parts


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def load_and_process(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError("CSV is empty: no LOG rows were recorded.")

    for col in ["scene"] + COLUMNS:
        if col not in df.columns:
            raise RuntimeError(f"Missing column: {col}")

    for col in COLUMNS:
        if col != "tag":
            df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df[df["tag"] == "LOG"].copy()
    df = df.dropna(subset=["time_ms"])

    if len(df) < 2:
        raise RuntimeError("Not enough valid LOG rows for plotting.")

    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
    df["left_target_rpm"] = (df["lf_t"] + df["lr_t"]) / 2.0
    df["left_actual_rpm"] = (df["lf_r"] + df["lr_r"]) / 2.0
    df["right_target_rpm"] = (df["rf_t"] + df["rr_t"]) / 2.0
    df["right_actual_rpm"] = (df["rf_r"] + df["rr_r"]) / 2.0
    df["avg_abs_wheel_rpm"] = (df[["lf_r", "rf_r", "lr_r", "rr_r"]].abs().sum(axis=1)) / 4.0

    df["dm3_pos_deg"] = df["dm3_pos_deg_x100"] / 100.0
    df["dm3_target_deg"] = df["dm3_target_deg_x100"] / 100.0
    df["dm3_vel_rad_s"] = df["dm3_vel_mrad_s"] / 1000.0
    df["dm3_torque"] = df["dm3_torque_x100"] / 100.0

    df["zdt42_cmd_rpm"] = df["zdt42_cmd_x10"] / 10.0
    return df


def save_plot(fig, out_dir: Path, stem: str):
    fig.tight_layout()
    fig.savefig(out_dir / f"{stem}.png", dpi=300)
    fig.savefig(out_dir / f"{stem}.pdf")
    plt.close(fig)


def plot_common(df: pd.DataFrame, out_dir: Path, scene: str, prefix: str):
    t = df["time_s"]

    fig = plt.figure(figsize=(10, 5.5))
    plt.plot(t, df["lf_t"], label="LF target")
    plt.plot(t, df["lf_r"], label="LF feedback")
    plt.plot(t, df["rf_t"], label="RF target")
    plt.plot(t, df["rf_r"], label="RF feedback")
    plt.xlabel("Time (s)")
    plt.ylabel("Wheel speed (rpm)")
    plt.title(f"{scene}: wheel speed tracking")
    plt.grid(True)
    plt.legend(ncol=2)
    save_plot(fig, out_dir, f"{prefix}_wheel_speed_tracking")

    fig = plt.figure(figsize=(10, 5.5))
    plt.plot(t, df["left_target_rpm"], label="Left target")
    plt.plot(t, df["left_actual_rpm"], label="Left feedback")
    plt.plot(t, df["right_target_rpm"], label="Right target")
    plt.plot(t, df["right_actual_rpm"], label="Right feedback")
    plt.xlabel("Time (s)")
    plt.ylabel("Side speed (rpm)")
    plt.title(f"{scene}: differential-drive side speed response")
    plt.grid(True)
    plt.legend(ncol=2)
    save_plot(fig, out_dir, f"{prefix}_side_speed_response")

    fig = plt.figure(figsize=(10, 5.5))
    plt.plot(t, df["dm3_target_deg"], label="Arm target/hold angle")
    plt.plot(t, df["dm3_pos_deg"], label="Arm feedback angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(f"{scene}: self-rescue arm angle response")
    plt.grid(True)
    plt.legend()
    save_plot(fig, out_dir, f"{prefix}_dm3_angle_response")

    fig = plt.figure(figsize=(10, 5.5))
    plt.plot(t, df["dm3_vel_rad_s"], label="Arm feedback velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title(f"{scene}: self-rescue arm velocity response")
    plt.grid(True)
    plt.legend()
    save_plot(fig, out_dir, f"{prefix}_dm3_velocity_response")

    fig = plt.figure(figsize=(10, 5.5))
    plt.plot(t, df["dm3_torque"], label="Arm feedback torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque / estimated torque")
    plt.title(f"{scene}: self-rescue arm torque response")
    plt.grid(True)
    plt.legend()
    save_plot(fig, out_dir, f"{prefix}_dm3_torque_response")

    fig = plt.figure(figsize=(10, 5.5))
    drill_scale = max(1.0, float(df["zdt42_cmd_rpm"].max()))
    plt.plot(t, df["zdt42_cmd_rpm"], label="Drill command speed")
    plt.plot(t, df["disc_cmd"] * drill_scale, label="Collecting disc ON/OFF, scaled")
    plt.xlabel("Time (s)")
    plt.ylabel("Command / state")
    plt.title(f"{scene}: drilling and collecting commands")
    plt.grid(True)
    plt.legend()
    save_plot(fig, out_dir, f"{prefix}_drill_disc_commands")


def plot_scene_specific(df: pd.DataFrame, out_dir: Path, scene: str, prefix: str):
    t = df["time_s"]

    if scene == "flat_sand_demo":
        fig = plt.figure(figsize=(10, 5.5))
        scale = max(1.0, float(df["avg_abs_wheel_rpm"].max()))
        plt.plot(t, df["avg_abs_wheel_rpm"], label="Average absolute wheel speed")
        plt.plot(t, df["zdt42_cmd_rpm"], label="Drill command speed")
        plt.plot(t, df["disc_cmd"] * scale, label="Collecting disc ON/OFF, scaled")
        plt.xlabel("Time (s)")
        plt.ylabel("Mobility / operation state")
        plt.title("Flat sand: mobility, drilling, and collection timeline")
        plt.grid(True)
        plt.legend()
        save_plot(fig, out_dir, f"{prefix}_flat_operation_timeline")

    elif scene == "slope_sand_demo":
        fig = plt.figure(figsize=(10, 5.5))
        scale = max(1.0, float(df["dm3_pos_deg"].abs().max()))
        plt.plot(t, df["avg_abs_wheel_rpm"], label="Average absolute wheel speed")
        plt.plot(t, df["dm3_pos_deg"], label="Arm feedback angle")
        plt.plot(t, df["disc_cmd"] * scale, label="Collecting disc ON/OFF, scaled")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed / arm posture / disc state")
        plt.title("Slope sand: wheel motion and assisting mechanism timeline")
        plt.grid(True)
        plt.legend()
        save_plot(fig, out_dir, f"{prefix}_slope_assist_timeline")

    elif scene == "pit_escape_demo":
        fig = plt.figure(figsize=(10, 5.5))
        plt.plot(t, df["avg_abs_wheel_rpm"], label="Average absolute wheel speed")
        plt.plot(t, df["dm3_torque"], label="Arm feedback torque")
        plt.plot(t, df["zdt42_cmd_rpm"], label="Drill command speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Recovery-related response")
        plt.title("Pit escape: wheel recovery, arm load, and drill command")
        plt.grid(True)
        plt.legend()
        save_plot(fig, out_dir, f"{prefix}_pit_escape_recovery")


def record(scene: str, port: str, baud: int, duration: float | None):
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = Path.home() / "rover_logs" / scene
    ensure_dir(base_dir)

    csv_path = base_dir / f"{scene}_{now_str}.csv"
    meta_path = base_dir / f"{scene}_{now_str}_meta.txt"
    raw_path = base_dir / f"{scene}_{now_str}_raw_head.txt"

    cfg = SCENE_CONFIG.get(scene, {"title": scene, "desc": "custom scene"})
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write(f"scene={scene}\n")
        f.write(f"title={cfg['title']}\n")
        f.write(f"description={cfg['desc']}\n")
        f.write(f"port={port}\n")
        f.write(f"baud={baud}\n")
        f.write(f"start_time={now_str}\n")
        f.write("format=LOG marker based packet extraction\n")

    print(f"Scene: {scene}")
    print(f"Port:  {port}")
    print(f"CSV:   {csv_path}")
    print("Start recording. Press Ctrl+C to stop.")

    rows = 0
    start = time.time()
    extractor = LogPacketExtractor(expected_fields=len(COLUMNS))
    raw_head = ""

    try:
        with serial.Serial(port, baud, timeout=0.1) as ser, open(csv_path, "w", newline="", encoding="utf-8") as f:
            # Flush old bytes already in the USB serial buffer.
            ser.reset_input_buffer()
            time.sleep(0.05)

            writer = csv.writer(f)
            writer.writerow(["scene"] + COLUMNS)

            while True:
                if duration is not None and (time.time() - start) >= duration:
                    break

                data = ser.read(4096)
                if not data:
                    continue

                chunk = data.decode("utf-8", errors="ignore")

                if len(raw_head) < 2000:
                    raw_head += chunk
                    with open(raw_path, "w", encoding="utf-8", errors="ignore") as rf:
                        rf.write(raw_head[:2000])

                for parsed in extractor.feed(chunk):
                    writer.writerow([scene] + parsed)
                    rows += 1
                    if rows % 20 == 0:
                        print(f"recorded rows: {rows}")

            for parsed in extractor.flush():
                writer.writerow([scene] + parsed)
                rows += 1

    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
        # Flush the last buffered packet after Ctrl+C.
        try:
            with open(csv_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                for parsed in extractor.flush():
                    writer.writerow([scene] + parsed)
                    rows += 1
        except Exception:
            pass

    print(f"Total valid rows: {rows}")

    if rows < 2:
        print("Not enough data.")
        print("But if raw_head contains LOG data, the problem is field-count mismatch.")
        print(f"Raw head saved to: {raw_path}")
        return

    try:
        df = load_and_process(csv_path)
        plot_common(df, base_dir, scene, csv_path.stem)
        plot_scene_specific(df, base_dir, scene, csv_path.stem)
        print(f"Figures saved in: {base_dir}")
    except Exception as e:
        print(f"Plotting failed: {e}")
        print(f"CSV was saved to: {csv_path}")


def main():
    parser = argparse.ArgumentParser(description="Integrated rover LOG recorder and plotter.")
    parser.add_argument("--scene", required=True, choices=sorted(SCENE_CONFIG.keys()))
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--duration", type=float, default=None, help="optional recording time in seconds")
    args = parser.parse_args()
    record(args.scene, args.port, args.baud, args.duration)


if __name__ == "__main__":
    main()
