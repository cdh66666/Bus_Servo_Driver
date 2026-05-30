#!/usr/bin/env python3
"""Serial-only auto tuning for the ESP32-C3 HLS-compatible servo boards.

This script talks to COM57 with the Feetech packet protocol. It does not use
the official FD UI and never moves the mouse.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import serial


INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

REG_MODEL = 0x03
REG_SETTING = 0x12
REG_P = 0x15
REG_PROTECT_CURRENT = 0x1C
REG_MODE = 0x21
REG_CURRENT_P = 0x22
REG_CURRENT_I = 0x23
REG_VELOCITY_P = 0x25
REG_VELOCITY_I = 0x27
REG_TORQUE_ENABLE = 0x28
REG_ACCEL = 0x29
REG_GOAL_POS = 0x2A
REG_GOAL_CURRENT = 0x2C
REG_RUN_SPEED = 0x2E
REG_TORQUE_LIMIT = 0x30
REG_PRESENT = 0x38
REG_DIAG = 0x57


def checksum(body: Iterable[int]) -> int:
    return (~sum(body)) & 0xFF


def packet(servo_id: int, inst: int, params: Iterable[int] = ()) -> bytes:
    params = list(params)
    length = len(params) + 2
    body = [servo_id, length, inst, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def parse_frame(buf: bytes) -> tuple[bytes | None, bytes]:
    for offset in range(max(0, len(buf) - 3)):
        if buf[offset] != 0xFF or buf[offset + 1] != 0xFF:
            continue
        length = buf[offset + 3]
        total = length + 4
        if length < 2 or length > 180:
            continue
        if offset + total > len(buf):
            return None, buf[offset:]
        frame = buf[offset : offset + total]
        if checksum(frame[2:-1]) == frame[-1]:
            return frame, buf[offset + total :]
    return None, buf[-1:]


def u16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def i16(data: bytes, offset: int) -> int:
    value = u16(data, offset)
    return value - 65536 if value & 0x8000 else value


def hls_signed_magnitude15(data: bytes, offset: int) -> int:
    value = u16(data, offset)
    magnitude = value & 0x7FFF
    return -magnitude if value & 0x8000 else magnitude


def encode_hls_signed_magnitude15(value: int) -> int:
    value = max(-32767, min(32767, int(value)))
    magnitude = abs(value) & 0x7FFF
    return (0x8000 | magnitude) if value < 0 else magnitude


def feetech_signed_magnitude(data: bytes, offset: int) -> int:
    return hls_signed_magnitude15(data, offset)


def feetech_load(data: bytes, offset: int) -> int:
    value = u16(data, offset)
    magnitude = value & 0x03FF
    return -magnitude if value & 0x0400 else magnitude


@dataclass(frozen=True)
class Tuning:
    name: str
    p: int
    d: int
    i: int
    min_pwm: int
    velocity_p: int
    velocity_i: int
    current_p: int
    current_i: int
    fd_accel: int
    fd_speed: int
    fd_current: int
    torque_limit: int = 1000
    protect_current: int = 1000


CANDIDATES = [
    Tuning("very_soft_85", 18, 9, 0, 32, 9, 0, 16, 3, 85, 85, 111, 850),
    Tuning("soft_90", 20, 8, 0, 34, 10, 0, 16, 3, 90, 90, 111, 900),
    Tuning("soft_95", 21, 8, 0, 35, 10, 0, 16, 3, 95, 95, 111, 900),
    Tuning("calm_100", 22, 8, 0, 36, 10, 1, 17, 3, 100, 100, 111, 920),
    Tuning("calm_105", 23, 8, 0, 37, 11, 1, 17, 3, 105, 105, 111, 940),
    Tuning("damped_105", 24, 7, 0, 38, 11, 1, 18, 4, 105, 105, 111, 960),
    Tuning("damped_111", 24, 7, 0, 38, 11, 1, 18, 4, 111, 111, 111, 960),
    Tuning("balanced_105", 25, 6, 0, 39, 12, 1, 18, 4, 105, 105, 111, 1000),
    Tuning("balanced_111", 26, 5, 0, 40, 12, 1, 18, 4, 111, 111, 111, 1000),
    Tuning("firm_111", 26, 4, 0, 42, 13, 1, 18, 4, 111, 111, 111, 1000),
    Tuning("quick_damped_115", 25, 7, 0, 39, 12, 1, 18, 4, 115, 115, 111, 980),
    Tuning("quick_120", 27, 5, 0, 41, 13, 1, 18, 4, 120, 120, 111, 1000),
]


class FeetechBus:
    def __init__(self, port: str, baud: int):
        self.serial = serial.Serial(port, baud, timeout=0.001, write_timeout=0.2)
        time.sleep(0.8)
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def request(
        self,
        servo_id: int,
        inst: int,
        params: Iterable[int] = (),
        timeout: float = 0.08,
    ) -> tuple[int, bytes]:
        self.serial.reset_input_buffer()
        self.serial.write(packet(servo_id, inst, params))
        self.serial.flush()
        end = time.time() + timeout
        buf = b""
        while time.time() < end:
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if chunk:
                buf += chunk
                while True:
                    frame, buf = parse_frame(buf)
                    if frame is None:
                        break
                    if frame[2] == servo_id:
                        return frame[4], bytes(frame[5:-1])
            else:
                time.sleep(0.001)
        raise TimeoutError(f"id={servo_id} inst=0x{inst:02X} timeout")

    def ping(self, servo_id: int) -> bool:
        err, _ = self.request(servo_id, INST_PING, timeout=0.35)
        return err == 0

    def read(self, servo_id: int, addr: int, length: int, timeout: float = 0.08, retries: int = 1) -> bytes:
        last_exc: Exception | None = None
        for _ in range(retries + 1):
            try:
                err, data = self.request(servo_id, INST_READ, [addr, length], timeout=timeout)
                if err != 0:
                    raise RuntimeError(f"id={servo_id} read 0x{addr:02X} err={err}")
                return data
            except TimeoutError as exc:
                last_exc = exc
                time.sleep(0.002)
        raise last_exc if last_exc is not None else TimeoutError("read timeout")

    def write(self, servo_id: int, addr: int, data: Iterable[int], timeout: float = 0.12, retries: int = 2) -> None:
        payload = list(data)
        last_exc: Exception | None = None
        for _ in range(retries + 1):
            try:
                err, _ = self.request(servo_id, INST_WRITE, [addr, *payload], timeout=timeout)
                if err != 0:
                    raise RuntimeError(f"id={servo_id} write 0x{addr:02X} err={err}")
                return
            except TimeoutError as exc:
                last_exc = exc
                time.sleep(0.004)
        raise last_exc if last_exc is not None else TimeoutError("write timeout")

    def write_u16(self, servo_id: int, addr: int, value: int) -> None:
        self.write(servo_id, addr, [value & 0xFF, (value >> 8) & 0xFF])

    def write_i16(self, servo_id: int, addr: int, value: int) -> None:
        self.write_u16(servo_id, addr, value & 0xFFFF)

    def write_hls_smag15(self, servo_id: int, addr: int, value: int) -> None:
        self.write_u16(servo_id, addr, encode_hls_signed_magnitude15(value))


def read_feedback(bus: FeetechBus, servo_id: int, timeout: float = 0.08, retries: int = 1) -> dict:
    data = bus.read(servo_id, REG_PRESENT, 16, timeout=timeout, retries=retries)
    return {
        "pos": hls_signed_magnitude15(data, 0),
        "speed": feetech_signed_magnitude(data, 2),
        "load": feetech_load(data, 4),
        "voltage": data[6],
        "temp": data[7],
        "err": data[9],
        "moving": data[10],
        "target": hls_signed_magnitude15(data, 11),
        "current": hls_signed_magnitude15(data, 13),
    }


def read_model(bus: FeetechBus, servo_id: int) -> list[int]:
    return list(bus.read(servo_id, REG_MODEL, 2, timeout=0.35, retries=2))


def read_diag(bus: FeetechBus, servo_id: int) -> dict:
    data = bus.read(servo_id, REG_DIAG, 9, timeout=0.35, retries=2)
    multi = int.from_bytes(data[2:6], "little", signed=True)
    return {
        "raw14": u16(data, 0),
        "multi": multi,
        "first_i2c": data[6],
        "active_i2c": data[7],
        "encoder_status": data[8],
    }


def configure_position(bus: FeetechBus, servo_id: int, phase: int, tune: Tuning) -> None:
    current_position = read_feedback(bus, servo_id, timeout=0.20, retries=3)["pos"]
    bus.write_hls_smag15(servo_id, REG_GOAL_POS, current_position)
    bus.write(servo_id, REG_SETTING, [phase])
    bus.write(servo_id, REG_P, [tune.p, tune.d, tune.i, tune.min_pwm])
    bus.write_u16(servo_id, REG_PROTECT_CURRENT, tune.protect_current)
    bus.write(servo_id, REG_CURRENT_P, [tune.current_p, tune.current_i])
    bus.write(servo_id, REG_VELOCITY_P, [tune.velocity_p])
    bus.write(servo_id, REG_VELOCITY_I, [tune.velocity_i])
    bus.write(servo_id, REG_MODE, [0])
    bus.write(servo_id, REG_ACCEL, [tune.fd_accel])
    bus.write_hls_smag15(servo_id, REG_GOAL_CURRENT, tune.fd_current)
    bus.write_hls_smag15(servo_id, REG_RUN_SPEED, tune.fd_speed)
    bus.write_u16(servo_id, REG_TORQUE_LIMIT, tune.torque_limit)
    bus.write(servo_id, REG_TORQUE_ENABLE, [1])
    time.sleep(0.12)


def unwrap_positions(samples: list[dict], start: int) -> list[int]:
    return [sample["pos"] for sample in samples]


def target_unwrapped(start: int, target: int) -> int:
    _ = start
    return target


def run_move(
    bus: FeetechBus,
    servo_id: int,
    target: int,
    duration: float,
    interval: float,
    fd_delay: float,
) -> dict:
    start = read_feedback(bus, servo_id)["pos"]
    bus.write_hls_smag15(servo_id, REG_GOAL_POS, target)
    t0 = time.time()
    next_t = t0
    samples = []
    missed_reads = 0
    while time.time() - t0 < duration:
        now = time.time()
        if now >= next_t:
            try:
                sample = read_feedback(bus, servo_id, timeout=max(0.012, interval * 1.2), retries=0)
                sample["t"] = now - t0
                samples.append(sample)
            except TimeoutError:
                missed_reads += 1
            next_t += interval
        time.sleep(0.002)

    positions = unwrap_positions(samples, start)
    expected = target_unwrapped(start, target)
    reach_s = None
    err_at_delay = None
    for sample, pos in zip(samples, positions):
        err = abs(pos - expected)
        if err_at_delay is None and sample["t"] >= fd_delay:
            err_at_delay = err
        if reach_s is None and err <= 35:
            reach_s = sample["t"]

    if err_at_delay is None and positions:
        err_at_delay = abs(positions[-1] - expected)

    direction = 1 if expected >= start else -1
    overshoot = 0
    if positions:
        if direction > 0:
            overshoot = max(0, max(positions) - expected)
        else:
            overshoot = max(0, expected - min(positions))

    speeds = [abs(s["speed"]) for s in samples]
    loads = [abs(s["load"]) for s in samples]
    currents = [abs(s["current"]) for s in samples]
    speed_deltas = [abs(samples[i]["speed"] - samples[i - 1]["speed"]) for i in range(1, len(samples))]
    load_deltas = [abs(samples[i]["load"] - samples[i - 1]["load"]) for i in range(1, len(samples))]
    raw_steps = []
    raw_rates = []
    for index in range(1, len(samples)):
        dt = max(0.001, samples[index]["t"] - samples[index - 1]["t"])
        step = abs(samples[index]["pos"] - samples[index - 1]["pos"])
        raw_steps.append(step if dt < 0.08 else 0)
        raw_rates.append(step / dt)
    max_raw_step = max(raw_steps or [0])
    max_raw_rate = max(raw_rates or [0.0])
    raw_jump_count = sum(1 for step in raw_steps if step > 180)

    monotonic_violations = 0
    if positions and abs(expected - start) > 80:
        direction = 1 if expected > start else -1
        for index in range(1, len(positions)):
            if (positions[index] - positions[index - 1]) * direction < -12:
                monotonic_violations += 1

    if len(samples) >= 2:
        sample_hz = (len(samples) - 1) / max(0.001, samples[-1]["t"] - samples[0]["t"])
    else:
        sample_hz = 0.0

    settle_start = max(fd_delay, (reach_s + 0.18) if reach_s is not None else fd_delay)
    settled_pairs = [(sample, pos) for sample, pos in zip(samples, positions) if sample["t"] >= settle_start]
    if len(settled_pairs) < 8 and samples:
        settled_pairs = list(zip(samples[-min(len(samples), 40):], positions[-min(len(positions), 40):]))
    settled_samples = [sample for sample, _ in settled_pairs]
    settled_positions = [pos for _, pos in settled_pairs]
    settle_position_range = (max(settled_positions) - min(settled_positions)) if settled_positions else 0
    settle_speed_avg = (
        sum(abs(sample["speed"]) for sample in settled_samples) / len(settled_samples)
        if settled_samples else 0.0
    )
    settle_speed_max = max([abs(sample["speed"]) for sample in settled_samples] or [0])
    settle_load_avg = (
        sum(abs(sample["load"]) for sample in settled_samples) / len(settled_samples)
        if settled_samples else 0.0
    )
    settle_load_max = max([abs(sample["load"]) for sample in settled_samples] or [0])
    speed_delta_avg = sum(speed_deltas) / len(speed_deltas) if speed_deltas else 0.0
    load_delta_avg = sum(load_deltas) / len(load_deltas) if load_deltas else 0.0
    speed_delta_max = max(speed_deltas or [0])
    load_delta_max = max(load_deltas or [0])

    return {
        "target": target,
        "start_raw": start,
        "end_raw": samples[-1]["pos"] if samples else start,
        "start_unwrapped": start,
        "end_unwrapped": positions[-1] if positions else start,
        "target_unwrapped": expected,
        "final_error": abs((positions[-1] if positions else start) - expected),
        "error_at_fd_delay": err_at_delay,
        "reach_s": reach_s,
        "overshoot": overshoot,
        "max_speed": max(speeds or [0]),
        "max_load": max(loads or [0]),
        "max_current": max(currents or [0]),
        "settle_position_range": round(settle_position_range, 2),
        "settle_speed_avg": round(settle_speed_avg, 2),
        "settle_speed_max": settle_speed_max,
        "settle_load_avg": round(settle_load_avg, 2),
        "settle_load_max": settle_load_max,
        "speed_delta_avg": round(speed_delta_avg, 2),
        "speed_delta_max": speed_delta_max,
        "load_delta_avg": round(load_delta_avg, 2),
        "load_delta_max": load_delta_max,
        "max_raw_step": max_raw_step,
        "max_raw_rate": round(max_raw_rate, 2),
        "raw_jump_count": raw_jump_count,
        "monotonic_violations": monotonic_violations,
        "sample_hz": round(sample_hz, 2),
        "sample_count": len(samples),
        "missed_reads": missed_reads,
    }


def score_move(move: dict) -> float:
    score = 100.0
    error_at_delay = move["error_at_fd_delay"]
    if error_at_delay is None:
        error_at_delay = 999
    score -= min(move["final_error"], 500) * 0.20
    score -= min(error_at_delay, 999) * 0.14
    score -= min(move["overshoot"], 250) * 0.38
    score -= move["raw_jump_count"] * 30.0
    if move["max_raw_step"] > 180:
        score -= (move["max_raw_step"] - 180) * 0.40
    score -= move["monotonic_violations"] * 3.0
    score -= move["settle_position_range"] * 1.30
    score -= move["settle_speed_avg"] * 1.60
    score -= max(0, move["settle_speed_max"] - 4) * 0.80
    score -= move["settle_load_avg"] * 0.05
    score -= max(0, move["settle_load_max"] - 80) * 0.03
    score -= move["speed_delta_avg"] * 0.85
    score -= max(0, move["speed_delta_max"] - 18) * 0.25
    score -= move["load_delta_avg"] * 0.07
    score -= max(0, move["load_delta_max"] - 160) * 0.04
    if move["sample_hz"] < 90.0:
        score -= (90.0 - move["sample_hz"]) * 0.8
    score -= move["missed_reads"] * 1.5

    reach_s = move["reach_s"] if move["reach_s"] is not None else 9.0
    if reach_s > 2.5:
        score -= (reach_s - 2.5) * 35.0
    # Keep a visible plateau at FD's 2500 ms scan delay, but avoid a lazy move.
    score -= abs(reach_s - 2.12) * 7.0

    speed = move["max_speed"]
    if speed < 35:
        score -= (35 - speed) * 0.70
    if speed > 85:
        score -= (speed - 85) * 0.55

    load = move["max_load"]
    if load < 70:
        score -= (70 - load) * 0.08
    if load > 850:
        score -= (load - 850) * 0.04
    return score


def evaluate_tuning(
    bus: FeetechBus,
    servo_id: int,
    phase: int,
    tune: Tuning,
    duration: float,
    interval: float,
    fd_delay: float,
) -> dict:
    configure_position(bus, servo_id, phase, tune)
    # Ensure each candidate starts from a known endpoint.
    home = run_move(bus, servo_id, 0, duration, interval, fd_delay)
    time.sleep(0.12)
    up = run_move(bus, servo_id, 4095, duration, interval, fd_delay)
    time.sleep(0.12)
    down = run_move(bus, servo_id, 0, duration, interval, fd_delay)
    time.sleep(0.12)
    up_again = run_move(bus, servo_id, 4095, duration, interval, fd_delay)
    moves = [up, down, up_again]
    score = sum(score_move(move) for move in moves) / len(moves)
    return {
        "servo_id": servo_id,
        "tuning": asdict(tune),
        "score": round(score, 3),
        "prepare_move": home,
        "moves": moves,
    }


def apply_best(bus: FeetechBus, servo_id: int, phase: int, tune: Tuning) -> None:
    configure_position(bus, servo_id, phase, tune)


def main() -> int:
    parser = argparse.ArgumentParser(description="Serial-only auto tune for ID2/ID3.")
    parser.add_argument("--port", default="COM57")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ids", nargs="+", type=int, default=[2, 3])
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--interval", type=float, default=0.01)
    parser.add_argument("--fd-delay", type=float, default=2.5)
    parser.add_argument("--output", default="tuning_results.json")
    parser.add_argument("--apply-best", action="store_true")
    parser.add_argument("--disable-torque-at-end", action="store_true")
    args = parser.parse_args()

    phase_by_id = {2: 1, 3: 0}
    report = {
        "port": args.port,
        "baud": args.baud,
        "fd_delay": args.fd_delay,
        "candidates": [asdict(c) for c in CANDIDATES],
        "servos": {},
    }

    bus = FeetechBus(args.port, args.baud)
    try:
        print(f"Opened {args.port} at {args.baud}.")
        for servo_id in [1, *args.ids]:
            try:
                ok = bus.ping(servo_id)
                model = read_model(bus, servo_id)
                print(f"ID{servo_id}: ping={ok} model={model}")
            except Exception as exc:
                print(f"ID{servo_id}: unavailable: {exc}")

        for servo_id in args.ids:
            phase = phase_by_id.get(servo_id, 0)
            try:
                diag = read_diag(bus, servo_id)
            except Exception as exc:
                diag = {"error": str(exc)}
            print(f"\nTuning ID{servo_id}, phase={phase}, diag={diag}")

            results = []
            for index, tune in enumerate(CANDIDATES, start=1):
                print(f"  [{index}/{len(CANDIDATES)}] {tune.name} ...", flush=True)
                try:
                    result = evaluate_tuning(
                        bus,
                        servo_id,
                        phase,
                        tune,
                        args.duration,
                        args.interval,
                        args.fd_delay,
                    )
                except Exception as exc:
                    result = {
                        "servo_id": servo_id,
                        "tuning": asdict(tune),
                        "score": -9999.0,
                        "error": str(exc),
                        "moves": [],
                    }
                results.append(result)
                print(f"      score={result['score']}")

            best = max(results, key=lambda item: item["score"])
            print(f"  BEST ID{servo_id}: {best['tuning']['name']} score={best['score']}")
            if args.apply_best:
                apply_best(bus, servo_id, phase, Tuning(**best["tuning"]))
                print(f"  Applied best tuning to ID{servo_id}.")

            report["servos"][str(servo_id)] = {
                "phase": phase,
                "diag": diag,
                "results": results,
                "best": best,
            }

    finally:
        if args.disable_torque_at_end:
            for servo_id in args.ids:
                try:
                    bus.write(servo_id, REG_TORQUE_ENABLE, [0])
                except Exception:
                    pass
        bus.close()

    output = Path(args.output)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nWrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
