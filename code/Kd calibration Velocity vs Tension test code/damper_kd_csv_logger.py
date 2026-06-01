#!/usr/bin/env python3
"""
damper_kd_csv_logger.py
Moteus BLDC electronic brake logger for wire EDM tension testing.
Applies a single fixed kd (damping) value and logs torque/velocity to CSV.
Designed to be launched automatically by the MATLAB dashboard.
"""

import asyncio
import math
import argparse
import time
import threading
import csv
import os
from collections import deque

import moteus

DEFAULT_SAVE_DIR = r"C:\Users\Nathan\matlab\Wire_Tension"
DEFAULT_CSV_NAME = "moteus_latest_torque.csv"


def _stdin_stop_listener(stop_flag: threading.Event):
    """Listen for user stop commands on stdin."""
    try:
        while not stop_flag.is_set():
            s = input().strip().lower()
            if s in ("s", "stop", "q", "quit", "exit"):
                stop_flag.set()
                return
    except EOFError:
        stop_flag.set()


def _start_plot_thread(hist: deque, stop_flag: threading.Event, title: str):
    """Live matplotlib plot of torque vs time (runs in background thread)."""
    def _plotter():
        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            print(f"\n[plot] matplotlib not available: {e}")
            return

        plt.ion()
        fig, ax = plt.subplots()
        try:
            fig.canvas.manager.set_window_title(title)
        except Exception:
            pass

        (line,) = ax.plot([], [])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Torque (Nm)")
        ax.set_title(title)
        ax.grid(True)

        t0 = None
        while not stop_flag.is_set():
            if len(hist) >= 2:
                xs = [p[0] for p in hist]
                ys = [p[1] for p in hist]
                if t0 is None:
                    t0 = xs[0]
                xs = [x - t0 for x in xs]
                line.set_data(xs, ys)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()
            time.sleep(0.05)

        try:
            plt.ioff()
            plt.close(fig)
        except Exception:
            pass

    th = threading.Thread(target=_plotter, daemon=True)
    th.start()
    return th


async def hard_stop(c: moteus.Controller):
    """Send stop command multiple times to ensure motor halts."""
    for _ in range(3):
        try:
            await c.set_stop()
        except Exception:
            pass
        await asyncio.sleep(0.05)


def _try_get_value(res, names, default=None):
    """Try to read a register value from a moteus response."""
    try:
        if not hasattr(res, "values") or not isinstance(res.values, dict):
            return default
        for reg_name in names:
            reg = getattr(moteus.Register, reg_name, None)
            if reg is not None and reg in res.values:
                try:
                    return float(res.values[reg])
                except Exception:
                    return res.values[reg]
        for key in names:
            if key in res.values:
                try:
                    return float(res.values[key])
                except Exception:
                    return res.values[key]
    except Exception:
        pass
    return default


def _extract_feedback(res):
    """Extract position, velocity, and torque from moteus response."""
    position_rev = _try_get_value(
        res,
        ["POSITION", "POSITION_REV", "position", "position_rev"],
        math.nan,
    )
    velocity_rps = _try_get_value(
        res,
        ["VELOCITY", "VELOCITY_RPS", "velocity", "velocity_rps"],
        math.nan,
    )
    torque_nm = _try_get_value(
        res,
        ["TORQUE", "TORQUE_NM", "torque", "torque_nm", "measured_torque", "q_torque"],
        math.nan,
    )
    return position_rev, velocity_rps, torque_nm


def _make_csv_path(user_path=None):
    """Resolve the output CSV path."""
    os.makedirs(DEFAULT_SAVE_DIR, exist_ok=True)
    if user_path:
        if os.path.isabs(user_path):
            out_path = user_path
        else:
            out_path = os.path.join(DEFAULT_SAVE_DIR, user_path)
    else:
        out_path = os.path.join(DEFAULT_SAVE_DIR, DEFAULT_CSV_NAME)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    return out_path


async def main():
    p = argparse.ArgumentParser(description="Moteus electronic brake logger")
    p.add_argument("--id", type=int, default=1, help="Moteus controller ID")
    p.add_argument("--rate", type=float, default=200.0, help="Control loop rate [Hz]")
    p.add_argument("--kd", type=float, default=0.25, help="Damping coefficient (kd_scale)")
    p.add_argument("--tau_max", type=float, default=0.60, help="Maximum torque [Nm]")
    p.add_argument("--watchdog", type=float, default=0.10, help="Watchdog timeout [s]")
    p.add_argument("--plot", action="store_true", help="Show live torque plot")
    p.add_argument("--history", type=float, default=10.0, help="Plot history window [s]")
    p.add_argument("--print_hz", type=float, default=5.0, help="Console print rate [Hz]")
    p.add_argument("--csv", type=str, default=DEFAULT_CSV_NAME, help="Output CSV filename")
    p.add_argument("--flush_every", type=int, default=25, help="Flush CSV every N rows")
    args = p.parse_args()

    if args.kd < 0:
        raise ValueError("kd must be >= 0")

    dt = 1.0 / max(args.rate, 1.0)
    print_dt = 1.0 / max(args.print_hz, 0.1)

    c = moteus.Controller(id=args.id)

    maxlen = max(10, int(args.history * max(args.rate, 1.0)))
    torque_hist = deque(maxlen=maxlen)

    stop_flag = threading.Event()
    csv_path = _make_csv_path(args.csv)

    if os.path.exists(csv_path):
        try:
            os.remove(csv_path)
        except Exception:
            pass

    # Auto-start: print info then begin immediately
    print("\n--- moteus electronic brake logger (fixed kd) ---")
    print(f"  Controller ID : {args.id}")
    print(f"  kd            : {args.kd}")
    print(f"  tau_max       : {args.tau_max} Nm")
    print(f"  rate          : {args.rate} Hz")
    print(f"  CSV           : {csv_path}")
    print("  Running... type 's' + Enter to stop (or Ctrl+C).\n")

    # Start stop-listener thread
    threading.Thread(target=_stdin_stop_listener, args=(stop_flag,), daemon=True).start()

    if args.plot:
        _start_plot_thread(torque_hist, stop_flag, f"moteus id={args.id} torque vs time")

    await hard_stop(c)

    last_print = time.monotonic()
    row_count = 0
    t0_wall = time.time()
    t0_mono = time.monotonic()

    f = None
    writer = None

    try:
        f = open(csv_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(f)

        writer.writerow([
            "timestamp_unix",
            "elapsed_s",
            "controller_id",
            "kd_applied",
            "tau_max_nm",
            "watchdog_s",
            "position_rev",
            "velocity_rps",
            "velocity_rpm",
            "measured_torque_nm",
        ])
        f.flush()

        while True:
            if stop_flag.is_set():
                await hard_stop(c)
                break

            res = await c.set_position(
                position=math.nan,
                velocity=0.0,
                kp_scale=0.0,
                kd_scale=args.kd,
                maximum_torque=abs(args.tau_max),
                watchdog_timeout=args.watchdog,
                query=True,
            )

            now_mono = time.monotonic()
            now_wall = time.time()
            elapsed_s = now_mono - t0_mono

            position_rev, velocity_rps, torque_nm = _extract_feedback(res)

            velocity_rpm = math.nan
            if not math.isnan(velocity_rps):
                velocity_rpm = velocity_rps * 60.0

            if not math.isnan(torque_nm):
                torque_hist.append((now_mono, torque_nm))

            writer.writerow([
                f"{now_wall:.6f}",
                f"{elapsed_s:.6f}",
                args.id,
                f"{args.kd:.6f}",
                f"{args.tau_max:.6f}",
                f"{args.watchdog:.6f}",
                f"{position_rev:.9f}" if not math.isnan(position_rev) else "",
                f"{velocity_rps:.9f}" if not math.isnan(velocity_rps) else "",
                f"{velocity_rpm:.9f}" if not math.isnan(velocity_rpm) else "",
                f"{torque_nm:.9f}" if not math.isnan(torque_nm) else "",
            ])
            row_count += 1

            if row_count % max(1, args.flush_every) == 0:
                f.flush()

            if now_mono - last_print >= print_dt:
                last_print = now_mono
                rpm_txt = f"{velocity_rpm:+.2f}" if not math.isnan(velocity_rpm) else "(n/a)"
                tq_txt = f"{torque_nm:+.3f}" if not math.isnan(torque_nm) else "(n/a)"
                print(f"rpm: {rpm_txt} | kd: {args.kd:.3f} | torque: {tq_txt} Nm")

            await asyncio.sleep(dt)

    except KeyboardInterrupt:
        await hard_stop(c)

    except Exception as e:
        print(f"\n[error] {e}")
        await hard_stop(c)

    finally:
        await hard_stop(c)
        stop_flag.set()

        if f is not None:
            try:
                f.flush()
                f.close()
            except Exception:
                pass

        duration = time.time() - t0_wall
        print(f"\nStopped. CSV: {os.path.abspath(csv_path)}  |  Rows: {row_count}  |  Time: {duration:.2f} s")


if __name__ == "__main__":
    asyncio.run(main())
