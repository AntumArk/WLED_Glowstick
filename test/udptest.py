#!/usr/bin/env python3
"""UDP listener/player for WLED Drumstick packets.

Supports two packet schemas:
  v1 processed hit:  {"v":1,"seq":N,"t":N,"z":Z,"vel":V,"conf":C}
  v2 raw sensor:     {"v":2,"seq":N,"t":N,"ax":N,"ay":N,"az":N,
                      "gx":N,"gy":N,"gz":N,"mx":N,"my":N,"mz":N}
  heartbeat:         {"v":1,"seq":N,"t":N,"hb":1}

In raw mode (v2) all processing runs on this PC:
  - Per-axis Kalman filter on dynamic acceleration (after gravity removal).
  - Velocity integrated from Kalman accel with exponential decay + ZUPT.
  - Swing state machine: stick must be PULLED BACK before next forward hit.
  - Hit fires at the peak of forward velocity (impact moment).
  - Zone derived from magnetometer heading (forward-zeroed).
  - Live 3D matplotlib window shows velocity trail, swing phase, and hit markers.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import concurrent.futures
import queue
import socket
import sys
import threading
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

try:
    import sounddevice as _sd
    HAS_SD = True
except ImportError:
    HAS_SD = False
    _sd = None  # type: ignore[assignment]

import numpy as np

try:
    from vispy import app as vispy_app, scene as vispy_scene
    from vispy.color import Color  # noqa: F401
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

ZONE_NAMES:  Dict[int, str] = {0: "Snare", 1: "Kick", 2: "HiHat", 3: "Tom"}
ZONE_COLORS: List[str]      = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12"]

# ── Kalman filter ──────────────────────────────────────────────────────────────

class Kalman1D:
    """Scalar Kalman filter for smoothing a 1-D noisy signal.

    q (process noise): larger → filter tracks fast changes better.
    r (measurement noise): larger → smoother output, slower response.
    Tune r up to reduce sensitivity to vibration/noise.
    """

    def __init__(self, q: float = 0.8, r: float = 10.0) -> None:
        self.q = q
        self.r = r
        self._x = 0.0
        self._p = 1.0

    def update(self, z: float) -> float:
        self._p += self.q                    # predict: covariance grows
        k = self._p / (self._p + self.r)    # Kalman gain
        self._x += k * (z - self._x)        # state update
        self._p *= 1.0 - k                  # covariance update
        return self._x

    def reset(self, value: float = 0.0) -> None:
        self._x = value
        self._p = 1.0


# ── Swing state machine ────────────────────────────────────────────────────────

class SwingPhase(Enum):
    IDLE    = "idle"    # at rest; fires immediately on forward velocity
    FORWARD = "forward" # forward swing in progress, tracking velocity peak


_PHASE_COLORS: Dict[SwingPhase, str] = {
    SwingPhase.IDLE:    "#7f8c8d",
    SwingPhase.FORWARD: "#2ecc71",
}


# ── Neural-network zone classifier ───────────────────────────────────────────

class DrumNet:
    """Tiny 3-layer MLP: 6 → 32 → 16 → 4, trained with mini-batch SGD.

    Architecture: ReLU activations on hidden layers, softmax output.
    He initialisation for weights (appropriate for ReLU).
    Cross-entropy loss, vanilla SGD with momentum.

    This is intentionally all-numpy — no framework dependency.
    At 6 inputs and 200 samples it trains in well under 1 second.
    """

    WEIGHTS_FILE = Path(__file__).with_name("drumstick_weights.json")

    def __init__(self) -> None:
        rng = np.random.default_rng(42)
        # He initialisation: σ = sqrt(2 / fan_in)
        self._W1 = rng.standard_normal((6,  32)).astype(np.float32) * np.sqrt(2 / 6)
        self._b1 = np.zeros(32,  dtype=np.float32)
        self._W2 = rng.standard_normal((32, 16)).astype(np.float32) * np.sqrt(2 / 32)
        self._b2 = np.zeros(16,  dtype=np.float32)
        self._W3 = rng.standard_normal((16,  4)).astype(np.float32) * np.sqrt(2 / 16)
        self._b3 = np.zeros(4,   dtype=np.float32)
        self.trained = False

    # ── inference ─────────────────────────────────────────────────────────

    def predict(self, feat: np.ndarray) -> Dict[int, float]:
        """Return softmax probability dict for one 6-D feature vector."""
        x  = feat.astype(np.float32).reshape(1, -1)
        h1 = np.maximum(0.0, x  @ self._W1 + self._b1)
        h2 = np.maximum(0.0, h1 @ self._W2 + self._b2)
        lg = h2 @ self._W3 + self._b3
        e  = np.exp(lg - lg.max())
        p  = (e / e.sum())[0]
        return {z: float(p[z]) for z in range(4)}

    # ── training ──────────────────────────────────────────────────────────

    def fit(self, X: np.ndarray, y: np.ndarray,
            epochs: int = 300, lr: float = 0.02,
            batch_size: int = 32, momentum: float = 0.9) -> float:
        """Mini-batch SGD with momentum.  Returns final cross-entropy loss.

        X: (N, 6) float32   y: (N,) int32  labels 0-3
        """
        N   = len(X)
        rng = np.random.default_rng()

        # Momentum velocity buffers
        vW1 = np.zeros_like(self._W1); vb1 = np.zeros_like(self._b1)
        vW2 = np.zeros_like(self._W2); vb2 = np.zeros_like(self._b2)
        vW3 = np.zeros_like(self._W3); vb3 = np.zeros_like(self._b3)

        loss = 0.0
        for _ in range(epochs):
            idx = rng.permutation(N)
            for i in range(0, N, batch_size):
                bi  = idx[i : i + batch_size]
                Xb, yb = X[bi], y[bi]

                # Forward pass
                h1 = np.maximum(0.0, Xb @ self._W1 + self._b1)
                h2 = np.maximum(0.0, h1 @ self._W2 + self._b2)
                lg = h2 @ self._W3 + self._b3
                e  = np.exp(lg - lg.max(axis=1, keepdims=True))
                pr = e / e.sum(axis=1, keepdims=True)

                # Cross-entropy loss (for reporting, last batch only)
                loss = float(-np.log(pr[np.arange(len(yb)), yb] + 1e-9).mean())

                # Backward pass
                dL = pr.copy()
                dL[np.arange(len(yb)), yb] -= 1
                dL /= len(yb)

                dW3 = h2.T @ dL;          db3 = dL.sum(0)
                dh2 = (dL @ self._W3.T) * (h2 > 0)
                dW2 = h1.T @ dh2;         db2 = dh2.sum(0)
                dh1 = (dh2 @ self._W2.T) * (h1 > 0)
                dW1 = Xb.T @ dh1;         db1 = dh1.sum(0)

                # SGD with momentum
                vW1 = momentum * vW1 - lr * dW1;  self._W1 += vW1
                vb1 = momentum * vb1 - lr * db1;  self._b1 += vb1
                vW2 = momentum * vW2 - lr * dW2;  self._W2 += vW2
                vb2 = momentum * vb2 - lr * db2;  self._b2 += vb2
                vW3 = momentum * vW3 - lr * dW3;  self._W3 += vW3
                vb3 = momentum * vb3 - lr * db3;  self._b3 += vb3

        self.trained = True
        return loss

    # ── persistence ───────────────────────────────────────────────────────

    def save(self) -> None:
        data = {k: getattr(self, k).tolist()
                for k in ("_W1", "_b1", "_W2", "_b2", "_W3", "_b3")}
        self.WEIGHTS_FILE.write_text(json.dumps(data))

    def load(self) -> bool:
        if not self.WEIGHTS_FILE.exists():
            return False
        try:
            data = json.loads(self.WEIGHTS_FILE.read_text())
            for k in ("_W1", "_b1", "_W2", "_b2", "_W3", "_b3"):
                setattr(self, k, np.array(data[k], dtype=np.float32))
            self.trained = True
            return True
        except Exception:
            return False


class ZoneClassifier:
    """Collects motion-feature templates and trains a DrumNet for classification.

    Feature vector (6-D): [mag_norm(3), vel_dir(3)]
      - mag_norm: normalised 3-D magnetometer vector → stick orientation
      - vel_dir:  normalised velocity direction at swing peak → motion direction

    Workflow:
      T       toggle training mode ON/OFF
      0-3     select which zone to train
      C       clear all templates for the current zone
      F       fit the neural network on all stored templates (requires ≥2 zones)
      S       save templates + network weights to disk
      L       load templates + weights from disk
      R       reset magnetometer forward-reference to current orientation

    Before F is pressed, falls back to cosine-similarity for instant single-sample
    feedback.  After F, the network is used and interpolates smoothly between zones.
    """

    TEMPLATE_FILE = Path(__file__).with_name("drumstick_templates.json")

    def __init__(self) -> None:
        self._templates: Dict[int, List[np.ndarray]] = {z: [] for z in range(4)}
        self._net = DrumNet()

    @property
    def trained(self) -> bool:
        return any(len(t) > 0 for t in self._templates.values())

    @property
    def net_trained(self) -> bool:
        return self._net.trained

    def add_template(self, zone: int, feat: np.ndarray) -> int:
        self._templates.setdefault(zone, []).append(feat.astype(np.float32))
        return len(self._templates[zone])

    def fit(self) -> int:
        """Train DrumNet on all stored templates.  Returns number of samples used."""
        X_list: List[np.ndarray] = []
        y_list: List[int] = []
        for z, tmpl in self._templates.items():
            for t in tmpl:
                X_list.append(t)
                y_list.append(z)
        n = len(X_list)
        if n < 4:
            return 0
        X = np.array(X_list, dtype=np.float32)
        y = np.array(y_list, dtype=np.int32)
        loss = self._net.fit(X, y)
        print(f"\n[NET] Trained on {n} samples — final loss {loss:.4f}", flush=True)
        return n

    def classify(self, feat: np.ndarray) -> Dict[int, float]:
        """Return zone probability dict.

        Uses the neural network when trained; falls back to cosine similarity
        for instant feedback before F is pressed.
        """
        if self._net.trained:
            return self._net.predict(feat)
        # Cosine-similarity fallback (works with even a single template per zone)
        scores = np.zeros(4, dtype=np.float32)
        any_tmpl = False
        for z in range(4):
            tmpl = self._templates.get(z, [])
            if tmpl:
                na = np.linalg.norm(feat)
                scores[z] = max(
                    float(np.dot(feat, t) / max(na * float(np.linalg.norm(t)), 1e-9))
                    for t in tmpl
                )
                any_tmpl = True
            else:
                scores[z] = -1.0
        if not any_tmpl:
            return {0: 1.0, 1: 0.0, 2: 0.0, 3: 0.0}
        s = scores * 8.0;  s -= s.max()
        w = np.exp(s);     w /= w.sum()
        return {z: float(w[z]) for z in range(4)}

    def clear_zone(self, zone: int) -> None:
        self._templates[zone] = []
        # Invalidate trained network — it no longer reflects the data
        self._net.trained = False

    def save(self) -> None:
        data = {str(z): [t.tolist() for t in ts]
                for z, ts in self._templates.items()}
        self.TEMPLATE_FILE.write_text(json.dumps(data, indent=2))
        if self._net.trained:
            self._net.save()

    def load(self) -> bool:
        if not self.TEMPLATE_FILE.exists():
            return False
        try:
            data = json.loads(self.TEMPLATE_FILE.read_text())
            for z_s, ts in data.items():
                self._templates[int(z_s)] = [
                    np.array(t, dtype=np.float32) for t in ts
                ]
            self._net.load()   # loads weights if the file exists; silent if not
            return True
        except Exception:
            return False



# ── Sensor processor ───────────────────────────────────────────────────────────

class SensorProcessor:
    """Processes raw v2 sensor packets and emits hit events.

    Pipeline:
      raw accel  →  gravity LPF removal  →  per-axis Kalman filter
      →  velocity integration (with decay + ZUPT)
      →  primary-axis projection  →  swing state machine  →  hit dict

    MPU6050 at ±8g: 1 g = 4096 LSB.  At 200 Hz with decay 0.90, a clean
    5-g hit lasting ~15 ms produces a peak velocity of roughly 1 200 LSB/s.
    The default vel_forward threshold of 500 LSB/s is deliberately low;
    raise it if phantom triggers occur at rest.

    Reference for ZUPT technique:
    https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5017361/
    """

    _VEL_DECAY    = 0.90    # per-sample velocity decay (prevents integration drift)
    _ZUPT_THRESH  = 400.0   # |Kalman-accel| below this → "still" frame
    _ZUPT_FRAMES  = 30      # consecutive still frames required for ZUPT (~150 ms)

    def __init__(
        self,
        vel_forward: float = 500.0,   # min forward velocity to count as a swing
        vel_still:   float = 180.0,   # |vel| below this → stick considered still
        cooldown_ms: int   = 150,
    ) -> None:
        self._VF = vel_forward
        self._VS = vel_still
        self._cooldown_ms = cooldown_ms

        # Gravity estimate (very heavy LPF; only changes when stick is repositioned)
        self._grav: List[float] = [0.0, 0.0, 0.0]
        self._grav_init = False

        # Per-axis Kalman on dynamic accel (gravity-removed)
        self._kf = [Kalman1D(q=0.8, r=10.0) for _ in range(3)]

        # Velocity (integrated Kalman accel with exponential decay)
        self._vel: List[float]  = [0.0, 0.0, 0.0]
        # Position (integrated velocity, partially reset by ZUPT)
        self._pos: List[float]  = [0.0, 0.0, 0.0]

        # Timing
        self._last_t_ms = 0
        self._default_dt = 0.005   # 200 Hz fallback

        # ZUPT state
        self._still_cnt = 0

        # Primary swing axis auto-detection (highest velocity variance)
        self._vel_var: List[float] = [100.0, 100.0, 100.0]
        self._swing_axis = 2        # Z by default; updates while idle/armed

        # Swing state
        self._phase     = SwingPhase.IDLE
        self._peak_vel  = 0.0
        self._last_hit_ms = 0

        # Magnetometer heading (2D, for compass display)
        self._hdg_filt = 0.0
        self._hdg_zero = 0.0
        self._hdg_init = False

        # Magnetometer — 3D normalised vector (replaces 2D heading for classification)
        self._mag_vec_norm: np.ndarray        = np.zeros(3, dtype=np.float32)
        self._forward_mag_vec: np.ndarray     = np.zeros(3, dtype=np.float32)
        self._mag_3d_init: bool               = False

        # Classifier (injected after construction; None = heading fallback)
        self.classifier: Optional[ZoneClassifier] = None
        # Training mode: hits are stored as templates instead of playing sound
        self.training_mode: bool = False
        self.training_zone: int  = 0

        # ── Public observables (read by plot/audio threads) ──────────────
        self.phase:         SwingPhase = SwingPhase.IDLE
        self.current_zone:  int        = 0
        self.zone_weights:  Dict[int, float] = {z: 0.0 for z in range(4)}
        self.vel_fwd:       float      = 0.0
        self.motion_mag:    float      = 0.0   # |Kalman-filtered dynamic accel|
        self.heading_abs:   float      = 0.0
        self.heading_rel:   float      = 0.0
        self.mag_vec:       np.ndarray = np.zeros(3, dtype=np.float32)  # 3D normalised
        self.mag_valid:     bool       = False
        self.swing_axis:    int        = 2
        self.last_peak_vel: float      = 0.0

        # Position trail for 3D plot (written here, read by plot thread)
        # We store the Kalman-filtered velocity vector as the trail, not position,
        # because position double-integrates and drifts faster than velocity.
        self.vel_trail:  Deque[Tuple[float, float, float]] = collections.deque(maxlen=400)
        self.hit_markers: Deque[Tuple[float, float, float, int]] = collections.deque(maxlen=30)
        # Monotonically incremented each time hit_markers changes.
        # _tick compares this instead of copying + comparing the full deque.
        self.hit_gen: int = 0
        # Throttle HUD classify: only re-run every 10th packet (~20 Hz at 200 Hz input).
        # Hit-time classification in _state_machine always runs at full rate.
        self._classify_skip: int = 0

    # ── public API ─────────────────────────────────────────────────────────

    def process(self, pkt: dict) -> Optional[dict]:
        """Process one raw v2 packet; return hit dict {zone, vel, peak} or None."""
        t_ms  = int(pkt.get("t",  0))
        ax_r  = float(pkt.get("ax", 0))
        ay_r  = float(pkt.get("ay", 0))
        az_r  = float(pkt.get("az", 0))
        mx    = float(pkt.get("mx", 0))
        my    = float(pkt.get("my", 0))
        mz    = float(pkt.get("mz", 0))

        # Adaptive dt from packet timestamps; clamp to ≤20 ms (handles late bursts)
        dt = self._default_dt
        if self._last_t_ms > 0 and 0 < (t_ms - self._last_t_ms) <= 20:
            dt = (t_ms - self._last_t_ms) * 0.001
        self._last_t_ms = t_ms

        self._update_mag(mx, my, mz)

        # Gravity low-pass (α = 0.97 → τ ≈ 160 ms at 200 Hz)
        if not self._grav_init:
            self._grav = [ax_r, ay_r, az_r]
            self._grav_init = True
            return None
        a = 0.97
        self._grav[0] = a * self._grav[0] + (1 - a) * ax_r
        self._grav[1] = a * self._grav[1] + (1 - a) * ay_r
        self._grav[2] = a * self._grav[2] + (1 - a) * az_r

        # Dynamic accel (high-pass residual = motion only)
        da = [ax_r - self._grav[0], ay_r - self._grav[1], az_r - self._grav[2]]

        # Kalman filter on each axis
        da_k = [self._kf[i].update(da[i]) for i in range(3)]
        self.motion_mag = math.sqrt(sum(x * x for x in da_k))

        # Velocity: integrate Kalman accel with exponential decay
        for i in range(3):
            self._vel[i] = self._vel[i] * self._VEL_DECAY + da_k[i] * dt

        # ZUPT: soft-zero velocity when stick has been still for _ZUPT_FRAMES
        if self.motion_mag < self._ZUPT_THRESH:
            self._still_cnt += 1
        else:
            self._still_cnt = 0
        if self._still_cnt >= self._ZUPT_FRAMES:
            for i in range(3):
                self._vel[i] *= 0.75
                self._pos[i] *= 0.90

        # Position integration (used only for hit marker placement in 3D plot)
        for i in range(3):
            self._pos[i] += self._vel[i] * dt

        # Store Kalman-filtered velocity as trail (avoids double-integration drift)
        self.vel_trail.append((self._vel[0], self._vel[1], self._vel[2]))

        # Velocity variance per axis (running estimate; τ ≈ 50 samples = 250 ms)
        for i in range(3):
            self._vel_var[i] = 0.98 * self._vel_var[i] + 0.02 * self._vel[i] ** 2

        # Update primary swing axis only while not mid-swing (prevents axis flip)
        if self._phase is SwingPhase.IDLE:
            self._swing_axis = max(range(3), key=lambda i: self._vel_var[i])
        self.swing_axis = self._swing_axis

        v_fwd = self._vel[self._swing_axis]
        self.vel_fwd = v_fwd
        # Only re-classify for HUD display every 10th packet (~20 Hz).
        # Hit-time classification happens inside _state_machine at full rate.
        self._classify_skip += 1
        if self._classify_skip >= 10:
            self._classify_skip = 0
            self.current_zone, self.zone_weights = self._classify()

        hit = self._state_machine(v_fwd, t_ms)
        self.phase = self._phase
        return hit

    # ── private ────────────────────────────────────────────────────────────

    def _state_machine(self, v_fwd: float, t_ms: int) -> Optional[dict]:
        """Two-state machine: IDLE fires directly when forward velocity threshold met."""
        ph = self._phase

        if ph == SwingPhase.IDLE:
            if v_fwd > self._VF:
                in_front = (not self._mag_3d_init) or float(
                    np.dot(self._mag_vec_norm, self._forward_mag_vec)
                ) >= 0.0
                if in_front:
                    self._phase    = SwingPhase.FORWARD
                    self._peak_vel = v_fwd

        elif ph == SwingPhase.FORWARD:
            if v_fwd > self._peak_vel:
                self._peak_vel = v_fwd  # track growing peak

            # Impact moment: velocity has dropped ≥35% from its peak.
            if self._peak_vel > self._VF and v_fwd < self._peak_vel * 0.65:
                self._phase = SwingPhase.IDLE
                in_front = (not self._mag_3d_init) or float(
                    np.dot(self._mag_vec_norm, self._forward_mag_vec)
                ) >= 0.0
                if in_front and t_ms - self._last_hit_ms >= self._cooldown_ms:
                    self._last_hit_ms  = t_ms
                    self.last_peak_vel = self._peak_vel
                    vel  = self._calc_vel(self._peak_vel)
                    feat = self._extract_features()
                    if self.training_mode and self.classifier is not None:
                        count = self.classifier.add_template(self.training_zone, feat)
                        zone  = self.training_zone
                        weights = {self.training_zone: 1.0}
                        print(f"\n[TRAIN] z{zone}({ZONE_NAMES[zone]}) "
                              f"+1 template ({count} total)", flush=True)
                    else:
                        zone, weights = self._classify_feat(feat)
                    self.current_zone  = zone
                    self.zone_weights  = weights
                    self.hit_markers.append(
                        (self._pos[0], self._pos[1], self._pos[2], zone)
                    )
                    self.hit_gen += 1
                    return {"zone": zone, "vel": vel, "peak": self._peak_vel,
                            "weights": weights}

            # Stalled without clean swing → return to idle
            elif v_fwd < self._VS and self.motion_mag < self._ZUPT_THRESH:
                self._phase    = SwingPhase.IDLE
                self._peak_vel = 0.0

        return None

    def _extract_features(self) -> np.ndarray:
        """6-D feature vector at hit time: [mag_norm(3), vel_dir(3)].

        mag_norm encodes stick spatial orientation; vel_dir encodes swing direction.
        Together they uniquely characterise each hit gesture.
        """
        mag = self._mag_vec_norm.copy() if self._mag_3d_init else np.zeros(3, np.float32)
        vel = np.array(self._vel, dtype=np.float32)
        vn  = float(np.linalg.norm(vel))
        vel_norm = vel / vn if vn > 1e-3 else np.zeros(3, np.float32)
        return np.concatenate([mag, vel_norm])

    def _classify_feat(self, feat: np.ndarray) -> tuple:
        """Return (best_zone, zone_weights) from classifier or heading fallback."""
        if self.classifier is not None and self.classifier.trained:
            weights  = self.classifier.classify(feat)
            best     = max(weights, key=lambda z: weights[z])
            return best, weights
        # Heading-quadrant fallback (used before any training)
        z = self._infer_zone_hdg()
        return z, {z: 1.0, **{i: 0.0 for i in range(4) if i != z}}

    def _classify(self) -> tuple:
        """Wrapper that calls _classify_feat with the current feature vector."""
        return self._classify_feat(self._extract_features())

    def _calc_vel(self, peak: float) -> int:
        t = (peak - self._VF) / max(1.0, 12000.0 - self._VF)
        return int(1 + max(0.0, min(1.0, t)) * 126)

    def _update_mag(self, mx: float, my: float, mz: float) -> None:
        """Update 3-D mag vector (classification) and 2-D heading (display)."""
        if mx == 0.0 and my == 0.0 and mz == 0.0:
            return

        # ── 3-D normalised vector ──────────────────────────────────────────
        # Mounting correction: X axis is reversed relative to accelerometer frame.
        raw = np.array([-mx, my, mz], dtype=np.float32)
        n   = float(np.linalg.norm(raw))
        if n > 1e-3:
            self._mag_vec_norm = raw / n
            self.mag_vec       = self._mag_vec_norm
            if not self._mag_3d_init:
                self._forward_mag_vec = self._mag_vec_norm.copy()
                self._mag_3d_init     = True
                self.mag_valid        = True

        # ── 2-D heading (for compass rose display) ─────────────────────────
        hdg = (math.atan2(my, -mx) * 57.2957795) % 360.0
        if not self._hdg_init:
            self._hdg_filt = self._hdg_zero = hdg
            self._hdg_init = True
        else:
            diff = hdg - self._hdg_filt
            if diff > 180.0:   diff -= 360.0
            elif diff < -180.0: diff += 360.0
            self._hdg_filt = (self._hdg_filt + diff * 0.20) % 360.0
        self.heading_abs = self._hdg_filt
        rel = (self._hdg_filt - self._hdg_zero) % 360.0
        self.heading_rel = rel - 360.0 if rel >= 180.0 else rel

    def _infer_zone_hdg(self) -> int:
        """Heading-quadrant zone (fallback when no templates are trained)."""
        if not self._hdg_init:
            return 0
        return int(((self._hdg_filt - self._hdg_zero) % 360.0) // 90.0) % 4

    # Keep old name as alias for the heading display used in the plot
    def rezero_heading(self) -> None:
        """Capture current 3-D mag vector and 2-D heading as the new forward reference."""
        if self._mag_3d_init:
            self._forward_mag_vec = self._mag_vec_norm.copy()
        if self._hdg_init:
            self._hdg_zero = self._hdg_filt
            self.heading_rel = 0.0


# ── 3D live plot (vispy — GPU-accelerated, 60 FPS) ───────────────────────────

class LivePlot3D:
    """Vispy-based 3D window.  Renders the velocity trail as a coloured line,
    hit markers as per-zone point clouds, and a heads-up text overlay.

    Strategy:
      - One LinePath trail (per-vertex RGBA fade old→bright orange)
      - One Markers scatter per zone for hit stars
      - vispy Timer at 60 Hz drives _tick() which updates only GPU buffers
      - Text overlay (vispy.scene.Text) for phase / zone / heading
      - No CPU-side matplotlib drawing — GPU does all rasterisation
    """

    _TRAIL_PTS  = 150
    # Colours converted to (R,G,B,A) float32 tuples for vispy
    _PHASE_RGBA: Dict  # populated in __init__
    _ZONE_RGBA:  List  # populated in __init__
    _TRAIL_PTS = 400   # max trail points passed to GPU each frame

    def __init__(self, processor: SensorProcessor) -> None:
        self._proc = processor

        # Hex → (r,g,b,1) float32
        def _hex(h: str) -> tuple:
            h = h.lstrip("#")
            r, g, b = int(h[0:2], 16)/255, int(h[2:4], 16)/255, int(h[4:6], 16)/255
            return (r, g, b, 1.0)

        self._phase_rgba = {ph: _hex(col) for ph, col in _PHASE_COLORS.items()}
        self._zone_rgba  = [_hex(c) for c in ZONE_COLORS]

        # ── vispy SceneCanvas ──────────────────────────────────────────────
        self._canvas = vispy_scene.SceneCanvas(
            title="Drumstick Live",
            bgcolor="#1a1a2e",
            size=(1280, 720),
            show=True,
            keys="interactive",
        )
        self._canvas.events.close.connect(self._on_close)
        self._canvas.events.key_press.connect(self._on_key)

        # ── Split canvas via grid (13 cols = 65% / 7 cols = 35%) ──────────
        _grid = self._canvas.central_widget.add_grid()

        # ── 3D viewport (left 65%) ─────────────────────────────────────────
        vp3 = _grid.add_view(row=0, col=0, col_span=13, bgcolor="#16213e")
        vp3.camera  = "turntable"
        vp3.camera.distance = 4.0

        # Velocity trail — single Line (strip), RGBA per vertex
        self._trail_vis = vispy_scene.visuals.Line(
            pos=np.zeros((2, 3), dtype=np.float32),
            color=np.ones((2, 4), dtype=np.float32),
            width=2,
            connect="strip",
            parent=vp3.scene,
        )

        # Current-position dot
        self._dot_vis = vispy_scene.visuals.Markers(parent=vp3.scene)
        self._dot_vis.set_data(
            np.zeros((1, 3), dtype=np.float32),
            face_color=(1, 1, 1, 1),
            size=14,
        )

        # Hit markers: one Markers visual per zone (different colours)
        self._hit_vis = {}
        for z in range(4):
            m = vispy_scene.visuals.Markers(parent=vp3.scene)
            m.set_data(
                np.zeros((1, 3), dtype=np.float32),
                face_color=self._zone_rgba[z],
                size=12,
            )
            m.visible = False
            self._hit_vis[z] = m

        self._trail_scale = 1.0

        # 3D axis lines (static)
        for axis, col in (([[-1,0,0],[1,0,0]], "#e74c3c"),
                           ([[0,-1,0],[0,1,0]], "#2ecc71"),
                           ([[0,0,-1],[0,0,1]], "#3498db")):
            vispy_scene.visuals.Line(
                pos=np.array(axis, dtype=np.float32),
                color=col,
                width=1,
                parent=vp3.scene,
            )

        # ── 2D HUD overlay (right 35%) ─────────────────────────────────────
        vp2 = _grid.add_view(row=0, col=13, col_span=7, bgcolor="#16213e")
        vp2.camera  = "panzoom"
        vp2.camera.set_range(x=(-1.4, 1.4), y=(-1.5, 1.5))

        def _txt(x, y, msg, **kw):
            t = vispy_scene.visuals.Text(
                msg, pos=(x, y, 0), parent=vp2.scene,
                font_size=kw.pop("font_size", 12), color=kw.pop("color", "white"),
                anchor_x="center", anchor_y="center",
            )
            return t

        self._t_phase = _txt(0,  1.25, "IDLE",         font_size=18, color="white")
        self._t_zone  = _txt(0,  0.85, "Zone 0  Snare", font_size=14, color=ZONE_COLORS[0])
        self._t_hdg   = _txt(0, -0.20, "",             font_size=9,  color="#aaa")
        self._t_vel   = _txt(0, -0.55, "",             font_size=9,  color="#aaa")
        self._t_peak  = _txt(0, -0.80, "",             font_size=9,  color="#888")
        self._t_mag   = _txt(0,  0.35, "No magnetometer", font_size=10, color="#e74c3c")
        self._t_train = _txt(0, -1.05, "",             font_size=10, color="#f1c40f")

        # Compass ring (static)
        n_pts = 64
        ang_arr = np.linspace(0, 2 * math.pi, n_pts, endpoint=False)
        ring_pts = np.column_stack([np.sin(ang_arr) * 0.40,
                                    np.cos(ang_arr) * 0.40 + 0.35,
                                    np.zeros(n_pts, dtype=np.float32)])
        vispy_scene.visuals.Line(
            pos=ring_pts.astype(np.float32),
            color="#444444",
            width=1,
            connect="strip",
            parent=vp2.scene,
        )
        for z in range(4):
            a = math.radians(z * 90)
            _txt(math.sin(a) * 0.52, 0.35 + math.cos(a) * 0.52,
                 ZONE_NAMES.get(z, "")[:1], font_size=9, color=ZONE_COLORS[z])

        # Compass needle — one Line (shaft) + one Markers (arrowhead tip).
        # Mutated each tick via set_data; never recreated (avoids per-frame GC).
        _needle_init = np.zeros((2, 3), dtype=np.float32)
        _needle_init[0] = [0.0, 0.35, 0.0]
        self._needle_line = vispy_scene.visuals.Line(
            pos=_needle_init, color="cyan", width=2,
            connect="strip", parent=vp2.scene,
        )
        self._needle_tip = vispy_scene.visuals.Markers(parent=vp2.scene)
        self._needle_tip.set_data(
            np.zeros((1, 3), dtype=np.float32), face_color="cyan", size=8,
        )
        self._needle_line.visible = False
        self._needle_tip.visible  = False
        self._vp2 = vp2
        self._last_hit_gen: int = -1

        self._stopped = False
        # Text cache: store the last string written to each text node.
        # Avoids GPU font-atlas re-upload when text hasn't changed.
        self._t_cache: Dict[str, str] = {}
        # Color buffer: pre-allocated (maxlen, 4) float32 array.
        # Recomputed only when trail length changes (gradient is index-based,
        # not position-based, so it's stable once the trail is full).
        self._trail_color_buf = np.zeros((400, 4), dtype=np.float32)
        self._trail_color_len = 0   # length for which _trail_color_buf is valid
        self._trail_last_gen  = -1  # trail_gen at last set_data call

    def _set_text(self, node, key: str, text: str, color: str = "") -> None:
        """Only forward text/color to the vispy node when the value changed.

        Avoids redundant font-atlas re-renders (expensive GPU texture uploads)
        when the text hasn't changed between ticks.
        """
        if self._t_cache.get(key) != text:
            self._t_cache[key] = text
            node.text = text
        color_key = key + "__c"
        if color and self._t_cache.get(color_key) != color:
            self._t_cache[color_key] = color
            node.color = color

    def _tick(self, event) -> None:
        """Timer callback — ~60 Hz. Updates GPU buffers only, no CPU rendering."""
        proc = self._proc

        # ── Trail ──────────────────────────────────────────────────────────
        # np.array() accepts a deque directly — no list() copy needed.
        trail = proc.vel_trail
        if len(trail) >= 3:
            pts  = np.array(trail, dtype=np.float32)
            peak = float(np.max(np.abs(pts))) or 1.0
            # Scale only grows immediately; decays very slowly (view stays stable)
            self._trail_scale = max(self._trail_scale * 0.998, peak)
            pts /= self._trail_scale
            n    = len(pts)

            # Recompute the gradient color array only when trail length changes.
            # The gradient is purely index-based (old=grey, new=orange) — once the
            # trail is at full depth (400 pts) it never needs recomputing again.
            if n != self._trail_color_len:
                alphas = np.linspace(0.15, 1.0, n, dtype=np.float32)
                orange = np.array([0.91, 0.49, 0.13], dtype=np.float32)
                grey   = np.array([0.25, 0.28, 0.30], dtype=np.float32)
                t_arr  = alphas[:, None]   # (n,1) for broadcasting
                colors = grey * (1 - t_arr) + orange * t_arr   # (n,3)
                self._trail_color_buf[:n, :3] = colors
                self._trail_color_buf[:n,  3] = alphas
                self._trail_color_len = n

            self._trail_vis.set_data(pos=pts, color=self._trail_color_buf[:n])

            phase_col = self._phase_rgba.get(proc.phase, (1, 1, 1, 1))
            self._dot_vis.set_data(
                pts[[-1]],
                face_color=phase_col,
                size=14,
            )

        # ── Hit markers ────────────────────────────────────────────────────
        # Use hit_gen counter to skip the list copy+compare when nothing changed.
        if proc.hit_gen != self._last_hit_gen:
            self._last_hit_gen = proc.hit_gen
            sc = self._trail_scale or 1.0
            by_zone: Dict[int, List] = {z: [] for z in range(4)}
            for hx, hy, hz, zone in proc.hit_markers:
                by_zone[zone % 4].append((hx / sc, hy / sc, hz / sc))
            for z, vis in self._hit_vis.items():
                pts2 = by_zone.get(z, [])
                if pts2:
                    vis.set_data(
                        np.array(pts2, dtype=np.float32),
                        face_color=self._zone_rgba[z],
                        size=12,
                    )
                    vis.visible = True
                else:
                    vis.visible = False

        # ── HUD text ───────────────────────────────────────────────────────
        ph_col    = _PHASE_COLORS.get(proc.phase, "#ffffff")
        zone_col  = ZONE_COLORS[proc.current_zone % 4]
        zone_name = ZONE_NAMES.get(proc.current_zone, "?")

        self._set_text(self._t_phase, "phase", proc.phase.value.upper(),  ph_col)
        self._set_text(self._t_zone,  "zone",
                       f"Zone {proc.current_zone}  {zone_name}", zone_col)
        self._set_text(self._t_vel,   "vel",
                       f"Motion {proc.motion_mag:.0f}   Vel fwd {proc.vel_fwd:+.0f}")
        self._set_text(self._t_peak,  "peak",
                       f"Peak {proc.last_peak_vel:.0f}  Thresh {proc._VF:.0f}")

        # Training status banner
        if proc.training_mode:
            zn = ZONE_NAMES.get(proc.training_zone, "?")
            n_tmpl = 0
            if proc.classifier is not None:
                n_tmpl = len(proc.classifier._templates.get(proc.training_zone, []))
            net_status = "NET✓" if (proc.classifier and proc.classifier.net_trained) else "cosine"
            tr_txt = (f"TRAIN z{proc.training_zone}:{zn}  "
                      f"({n_tmpl} samples)  [0-3]=zone  "
                      f"[F]=fit  [C]=clear  [{net_status}]")
            self._set_text(self._t_train, "train", tr_txt, "#f1c40f")
        else:
            clf = proc.classifier
            net_status = "NET✓" if (clf and clf.net_trained) else "cosine-sim"
            self._set_text(self._t_train, "train",
                           f"[{net_status}]  [T]=train",
                           "#2ecc71" if (clf and clf.net_trained) else "#7f8c8d")

        # Compass needle — mutate the persistent Line + Markers (no allocation).
        if proc.mag_valid:
            ang = math.radians(proc.heading_rel)
            nx, ny = math.sin(ang) * 0.36, math.cos(ang) * 0.36
            tip_pt = np.array([[nx, 0.35 + ny, 0.0]], dtype=np.float32)
            shaft  = np.array([[0.0, 0.35, 0.0], [nx, 0.35 + ny, 0.0]], dtype=np.float32)
            self._needle_line.set_data(pos=shaft)
            self._needle_tip.set_data(tip_pt, face_color="cyan", size=8)
            self._needle_line.visible = True
            self._needle_tip.visible  = True
            self._set_text(self._t_hdg, "hdg",
                           f"hdg {proc.heading_abs:.1f}°  rel {proc.heading_rel:+.1f}°",
                           "#aaaaaa")
            self._t_mag.visible = False
        else:
            self._needle_line.visible = False
            self._needle_tip.visible  = False
            self._set_text(self._t_hdg, "hdg", "")
            self._t_mag.visible = True

        self._canvas.update()

    def _on_key(self, event) -> None:
        """Keyboard handler for training commands.

        T          — toggle training mode on/off
        0/1/2/3    — (in training mode) select target zone
        C          — clear templates for current zone (also resets network)
        F          — fit neural network on all stored templates
        S          — save templates + weights to disk
        L          — load templates + weights from disk
        R          — reset forward reference to current mag/heading
        """
        key  = event.key.name if event.key is not None else ""
        proc = self._proc
        clf  = proc.classifier

        if key == "T":
            proc.training_mode = not proc.training_mode
            zn = ZONE_NAMES.get(proc.training_zone, "?")
            print(f"\n[KEY] Training {'ON' if proc.training_mode else 'OFF'} "
                  f"(zone {proc.training_zone}={zn})", flush=True)

        elif proc.training_mode and key in ("0", "1", "2", "3"):
            proc.training_zone = int(key)
            zn = ZONE_NAMES.get(proc.training_zone, "?")
            print(f"\n[KEY] Training zone → {proc.training_zone} ({zn})", flush=True)

        elif key == "C" and clf is not None:
            clf.clear_zone(proc.training_zone)
            zn = ZONE_NAMES.get(proc.training_zone, "?")
            print(f"\n[KEY] Cleared templates for z{proc.training_zone} ({zn})", flush=True)

        elif key == "F" and clf is not None:
            total = sum(len(t) for t in clf._templates.values())
            zones_with_data = sum(1 for t in clf._templates.values() if t)
            if zones_with_data < 2:
                print(f"\n[NET] Need templates for ≥2 zones to train (have {zones_with_data})",
                      flush=True)
            else:
                print(f"\n[NET] Training on {total} samples across "
                      f"{zones_with_data} zones...", flush=True, end="")
                n = clf.fit()
                if n:
                    print(f" done.  [S] to save weights.", flush=True)

        elif key == "S" and clf is not None:
            clf.save()
            print(f"\n[KEY] Saved → {clf.TEMPLATE_FILE}"
                  + (f" + {clf._net.WEIGHTS_FILE.name}" if clf.net_trained else ""),
                  flush=True)

        elif key == "L" and clf is not None:
            ok = clf.load()
            net_msg = " (+ weights)" if clf.net_trained else ""
            print(f"\n[KEY] Templates {'loaded from ' + str(clf.TEMPLATE_FILE) + net_msg if ok else 'file not found'}",
                  flush=True)

        elif key == "R":
            proc.rezero_heading()
            print("\n[KEY] Forward reference reset to current orientation", flush=True)

    def _on_close(self, event) -> None:
        self._stopped = True
        vispy_app.quit()

    def run(self) -> None:
        """Start the vispy event loop — blocks until window is closed."""
        timer = vispy_app.Timer(interval=1/60, connect=self._tick, start=True)  # noqa: F841
        vispy_app.run()


# ── audio helpers ──────────────────────────────────────────────────────────────

# ── Polyphonic audio mixer ─────────────────────────────────────────────────────

class SoundMixer:
    """Real-time polyphonic mixer via sounddevice.OutputStream.

    Supports unlimited simultaneous voices at independent volumes.
    All loaded audio is decoded to float32 stereo at the device sample rate.
    This replaces the old sd.play() single-stream approach (which cut off
    previous sounds) and enables zone-blended playback.
    """

    # 128 frames @ 44100 Hz ≈ 2.9 ms per callback — low enough for drum hits.
    _BLOCKSIZE = 128

    def __init__(self, samplerate: int = 44100) -> None:
        self._sr    = samplerate
        self._lock  = threading.Lock()
        # Each voice: [data_float32_stereo, current_offset_int]
        self._voices: List[List] = []
        self.master_volume: float = 1.0
        self.zone_volumes:  Dict[int, float] = {z: 1.0 for z in range(4)}
        if not HAS_SD:
            raise RuntimeError("sounddevice required: pip install sounddevice")
        self._stream = _sd.OutputStream(
            samplerate=samplerate,
            channels=2,
            dtype="float32",
            blocksize=self._BLOCKSIZE,
            latency="low",   # request minimum WASAPI device buffer
            callback=self._callback,
        )
        self._stream.start()

    def play(self, data: np.ndarray, volume: float = 1.0) -> None:
        """Enqueue a sound for mixing — thread-safe, non-blocking."""
        buf = (data * max(0.0, min(1.0, volume))).astype(np.float32)
        with self._lock:
            self._voices.append([buf, 0])

    def effective_volume(self, zone: int) -> float:
        """Return master × zone volume, clamped to [0, 1]."""
        return float(np.clip(self.master_volume * self.zone_volumes.get(zone % 4, 1.0),
                             0.0, 1.0))

    def get_volumes(self) -> Dict:
        """Return current volume state (thread-safe snapshot)."""
        return {"master": self.master_volume,
                "zones":  {z: self.zone_volumes[z] for z in range(4)}}

    def set_volume(self, zone: int, value: float) -> None:
        """Set master (zone=-1) or per-zone volume.  Value clamped to [0, 1]."""
        v = float(np.clip(value, 0.0, 1.0))
        if zone == -1:
            self.master_volume = v
        else:
            self.zone_volumes[zone % 4] = v

    def _callback(self, outdata: np.ndarray, frames: int, _time, _status) -> None:
        outdata.fill(0.0)
        with self._lock:
            done = []
            for i, voice in enumerate(self._voices):
                buf, offset = voice
                end = min(offset + frames, len(buf))
                n   = end - offset
                outdata[:n] += buf[offset:end]
                if end >= len(buf):
                    done.append(i)
                else:
                    voice[1] = end
            for i in reversed(done):
                self._voices.pop(i)
        np.clip(outdata, -1.0, 1.0, out=outdata)

    def close(self) -> None:
        self._stream.stop()
        self._stream.close()


# ── Drum synthesizer (no sample files required) ────────────────────────────────

# Default parameters per zone.  All values are mutable at runtime via the
# browser control page served on --synth-port (default 8765).
_SYNTH_DEFAULTS: Dict[int, Dict] = {
    0: {"name": "Snare",  "pitch":    220, "decay_ms": 120, "noise": 0.65, "click": 0.08},
    1: {"name": "Kick",   "pitch_hi": 120, "pitch_lo":  40, "decay_ms": 380, "click": 0.12},
    2: {"name": "HiHat",  "decay_ms":  60, "order":       3},
    3: {"name": "Tom",    "pitch_hi": 100, "pitch_lo":  45, "decay_ms": 320, "click": 0.08},
}

# Slider specs for each zone: (key, label, min, max, step)
_SYNTH_PARAM_UI: Dict[int, List[tuple]] = {
    0: [("pitch",    "Pitch Hz",    50, 500,  1),
        ("decay_ms", "Decay ms",    20, 500,  1),
        ("noise",    "Noise ratio",  0,   1, 0.01),
        ("click",    "Click",        0, 0.3, 0.01)],
    1: [("pitch_hi", "Pitch start Hz", 50, 300,  1),
        ("pitch_lo", "Pitch end Hz",   15, 150,  1),
        ("decay_ms", "Decay ms",      100, 800,  1),
        ("click",    "Click",           0, 0.3, 0.01)],
    2: [("decay_ms", "Decay ms", 10, 200, 1),
        ("order",    "Brightness (1–6)", 1, 6, 1)],
    3: [("pitch_hi", "Pitch start Hz", 50, 250,  1),
        ("pitch_lo", "Pitch end Hz",   15, 120,  1),
        ("decay_ms", "Decay ms",      100, 700,  1),
        ("click",    "Click",           0, 0.3, 0.01)],
}


class DrumSynth:
    """Real-time drum synthesizer — generates percussive PCM on demand.

    Each hit calls synthesize(zone, velocity) which returns a float32 stereo
    numpy array.  The buffer is then fed into SoundMixer.play() for output.

    All parameter dicts are protected by a lock so the HTTP control server
    can update them from its own thread without races.

    Synthesis approaches:
      Kick / Tom  — exponential pitch-envelope sine (classic analogue kick model)
      Snare       — pitched sine mixed with white noise, brief click transient
      Hi-hat      — white noise shaped by repeated first-difference (FIR highpass)
    """

    def __init__(self, samplerate: int = 44100) -> None:
        self._sr     = samplerate
        self._lock   = threading.Lock()
        self._params = {z: dict(p) for z, p in _SYNTH_DEFAULTS.items()}
        # Single persistent RNG — avoids OS entropy read (system call) per hit.
        # Pseudo-random sequence still gives natural variation between hits.
        self._rng = np.random.default_rng()

    def get_params(self) -> Dict[int, Dict]:
        """Return a deep copy of all zone parameter dicts (thread-safe)."""
        with self._lock:
            return {z: dict(p) for z, p in self._params.items()}

    def set_zone_params(self, zone: int, updates: Dict) -> None:
        """Merge updates into zone params — thread-safe."""
        with self._lock:
            self._params[zone % 4].update(updates)

    def synthesize(self, zone: int, velocity: float = 1.0) -> np.ndarray:
        """Return float32 stereo array for one hit, amplitude scaled by velocity."""
        with self._lock:
            p = dict(self._params.get(zone % 4, self._params[0]))
        z   = zone % 4
        vel = float(np.clip(velocity, 0.0, 1.0))
        sr  = self._sr
        rng = self._rng   # persistent RNG — avoids OS entropy syscall per hit

        if z in (1, 3):   # ── Kick / Tom: pitched sine with decaying frequency ──
            dec   = float(p.get("decay_ms", 380)) / 1000.0
            n     = int(dec * 3.5 * sr)
            t     = np.arange(n, dtype=np.float64) / sr
            f_hi  = float(p.get("pitch_hi", 120))
            f_lo  = float(p.get("pitch_lo",  40))
            # Frequency decays exponentially from f_hi to f_lo in ~25% of dec time
            freq  = f_lo + (f_hi - f_lo) * np.exp(-t / (dec * 0.25))
            # Cumsum gives correct instantaneous phase for time-varying pitch
            phase = 2.0 * np.pi * np.cumsum(freq) / sr
            tone  = np.sin(phase) * np.exp(-t / dec)
            click = float(p.get("click", 0.12)) * rng.standard_normal(n) * np.exp(-t / 0.002)
            mono  = (tone + click) * vel * 0.85

        elif z == 0:   # ── Snare: pitched tone + noise burst ──
            dec   = float(p.get("decay_ms", 120)) / 1000.0
            n     = int(dec * 3.5 * sr)
            t     = np.arange(n, dtype=np.float64) / sr
            nr    = float(p.get("noise", 0.65))
            amp   = np.exp(-t / dec)
            tone  = (1.0 - nr) * np.sin(2.0 * np.pi * float(p.get("pitch", 220)) * t)
            noise = nr * rng.standard_normal(n)
            click = float(p.get("click", 0.08)) * rng.standard_normal(n) * np.exp(-t / 0.002)
            mono  = (tone + noise + click) * amp * vel * 0.80

        else:   # ── Hi-hat: noise shaped by repeated first-difference (FIR highpass) ──
            dec   = float(p.get("decay_ms", 60)) / 1000.0
            order = max(1, int(p.get("order", 3)))   # higher = brighter
            n     = int(dec * 4.0 * sr)
            t     = np.arange(n, dtype=np.float64) / sr
            hp    = rng.standard_normal(n)
            for _ in range(order):
                hp = np.diff(hp, prepend=hp[0])      # each pass raises high-freq content
            mono  = hp * np.exp(-t / dec) * vel * 0.70

        mono = np.clip(mono, -1.0, 1.0).astype(np.float32)
        return np.column_stack([mono, mono])   # mono → stereo


def timestamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


# ── Browser synth control server ───────────────────────────────────────────────

import http.server as _http_server
import urllib.parse as _urllib_parse


def _build_synth_html(params: Dict[int, Dict], mixer_ref: "Optional[SoundMixer]") -> bytes:
    """Build the full synth-control HTML page from current params."""
    zone_color_css = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12"]

    # ── Volume section ────────────────────────────────────────────────────
    vols = mixer_ref.get_volumes() if mixer_ref is not None else \
           {"master": 1.0, "zones": {z: 1.0 for z in range(4)}}
    zone_vol_sliders = []
    for z in range(4):
        name = params.get(z, {}).get("name", f"Zone {z}")
        v    = vols["zones"].get(z, 1.0)
        zone_vol_sliders.append(
            f'<label>'
            f'<span style="color:{zone_color_css[z]}">z{z} {name}</span>'
            f'<input type="range" min="0" max="1" step="0.01" value="{v:.2f}" '
            f'oninput="setVol({z},this.value,this.nextElementSibling)">'
            f'<span class="val">{v:.2f}</span></label>'
        )
    mv = vols["master"]
    volume_html = (
        f'<div class="zone vol-section">'
        f'<h3 style="color:#ddd">&#128266; Volume</h3>'
        f'<label><span style="font-weight:600">Master</span>'
        f'<input type="range" min="0" max="1" step="0.01" value="{mv:.2f}" '
        f'oninput="setVol(-1,this.value,this.nextElementSibling)">'
        f'<span class="val">{mv:.2f}</span></label>'
        + "\n".join(zone_vol_sliders) +
        f'</div>'
    )

    # ── Synth parameter sections ──────────────────────────────────────────
    zone_html = []
    for z in range(4):
        p     = params.get(z, {})
        name  = p.get("name", f"Zone {z}")
        specs = _SYNTH_PARAM_UI.get(z, [])
        sliders = []
        for key, label, mn, mx, step in specs:
            val = p.get(key, mn)
            sliders.append(
                f'<label><span>{label}</span>'
                f'<input type="range" min="{mn}" max="{mx}" step="{step}" value="{val}" '
                f'oninput="update({z},\'{key}\',this.value,this.nextElementSibling)">'
                f'<span class="val">{val}</span></label>'
            )
        zone_html.append(
            f'<div class="zone" style="border-left:4px solid {zone_color_css[z]}">'
            f'<h3 style="color:{zone_color_css[z]}">z{z} \u2014 {name}</h3>'
            + "\n".join(sliders) +
            f'<button onclick="test({z})">&#9654; Test</button>'
            f'</div>'
        )
    zones_str = "\n".join(zone_html)
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drum Synth Controls</title>
<style>
  *{{box-sizing:border-box;margin:0;padding:0}}
  body{{font:14px/1.6 system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:12px}}
  h1{{font-size:1.1rem;margin-bottom:12px;color:#aaa;letter-spacing:.05em}}
  .zone{{background:#16213e;border-radius:8px;padding:12px;margin:8px 0}}
  .vol-section{{border-left:4px solid #7f8c8d}}
  .zone h3{{margin-bottom:8px;font-size:.95rem}}
  label{{display:flex;align-items:center;gap:6px;margin:5px 0;font-size:.82rem}}
  label span:first-child{{flex:1;color:#9aa}}
  input[type=range]{{flex:2;accent-color:#e67e22}}
  .vol-section input[type=range]{{accent-color:#7f8c8d}}
  .vol-section label:first-of-type input[type=range]{{accent-color:#ecf0f1}}
  .val{{width:38px;text-align:right;color:#fa0;font-variant-numeric:tabular-nums}}
  button{{margin-top:8px;padding:5px 14px;background:#0f3460;color:#e0e0e0;
          border:1px solid #334;border-radius:5px;cursor:pointer;font-size:.82rem}}
  button:active{{background:#e67e22;color:#111}}
  #status{{position:fixed;bottom:8px;right:10px;font-size:.75rem;color:#555}}
</style>
</head>
<body>
<h1>DRUM SYNTH \u2014 controls</h1>
{volume_html}
{zones_str}
<div id="status">ready</div>
<script>
function update(zone,key,val,disp){{
  disp.textContent=parseFloat(val);
  clearTimeout(update._t);
  update._t=setTimeout(()=>post(zone,key,parseFloat(val)),80);
}}
function post(zone,key,value){{
  fetch('/params',{{method:'POST',headers:{{'Content-Type':'application/json'}},
    body:JSON.stringify({{zone,key,value}})}})
    .then(r=>r.json())
    .then(()=>{{document.getElementById('status').textContent=
      key+'='+value+' (z'+zone+') \u2713';}})
    .catch(e=>document.getElementById('status').textContent='err: '+e);
}}
function setVol(zone,val,disp){{
  disp.textContent=parseFloat(val).toFixed(2);
  clearTimeout(setVol._t);
  setVol._t=setTimeout(()=>{{
    fetch('/volume',{{method:'POST',headers:{{'Content-Type':'application/json'}},
      body:JSON.stringify({{zone:zone,value:parseFloat(val)}})}})
      .then(r=>r.json())
      .then(()=>{{document.getElementById('status').textContent=
        (zone==-1?'master':'z'+zone)+' vol='+parseFloat(val).toFixed(2)+' \u2713';}})
      .catch(e=>document.getElementById('status').textContent='err: '+e);
  }},60);
}}
function test(zone){{
  fetch('/test?zone='+zone)
    .catch(()=>{{}});
}}
</script>
</body>
</html>"""
    return html.encode()


class _SynthHandler(_http_server.BaseHTTPRequestHandler):
    """Request handler for the drum synth control page.

    Attributes injected by the server: synth (DrumSynth), mixer (SoundMixer).
    """

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            body = _build_synth_html(self.server.synth.get_params(), self.server.mixer)
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path.startswith("/test"):
            qs  = _urllib_parse.parse_qs(_urllib_parse.urlparse(self.path).query)
            z   = int(qs.get("zone", ["0"])[0]) % 4
            if self.server.mixer is not None:
                buf = self.server.synth.synthesize(z, velocity=0.8)
                self.server.mixer.play(buf, volume=self.server.mixer.effective_volume(z))
            self._send(200, "application/json", b'{"ok":true}')
        elif self.path == "/volume":
            if self.server.mixer is not None:
                body = json.dumps(self.server.mixer.get_volumes()).encode()
            else:
                body = b'{"master":1.0,"zones":{"0":1.0,"1":1.0,"2":1.0,"3":1.0}}'
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(length))
        except Exception as exc:
            self._send(400, "application/json", json.dumps({"error": str(exc)}).encode())
            return
        if self.path == "/params":
            try:
                self.server.synth.set_zone_params(int(body["zone"]), {body["key"]: body["value"]})
                self._send(200, "application/json", b'{"ok":true}')
            except Exception as exc:
                self._send(400, "application/json", json.dumps({"error": str(exc)}).encode())
        elif self.path == "/volume":
            try:
                if self.server.mixer is not None:
                    self.server.mixer.set_volume(int(body["zone"]), float(body["value"]))
                self._send(200, "application/json", b'{"ok":true}')
            except Exception as exc:
                self._send(400, "application/json", json.dumps({"error": str(exc)}).encode())
        else:
            self._send(404, "text/plain", b"not found")

    def _send(self, code: int, ct: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_) -> None:
        pass   # silence per-request logging


def start_synth_server(synth: "DrumSynth", mixer: Optional["SoundMixer"],
                       port: int) -> _http_server.HTTPServer:
    """Start the synth control HTTP server on a daemon thread."""
    srv = _http_server.HTTPServer(("0.0.0.0", port), _SynthHandler)
    srv.synth = synth   # type: ignore[attr-defined]
    srv.mixer = mixer   # type: ignore[attr-defined]
    t = threading.Thread(target=srv.serve_forever, daemon=True, name="synth-http")
    t.start()
    return srv




# Colours for each zone — RGB tuples, full brightness.
# z0 Snare: bright white-amber  z1 Kick: deep red
# z2 Hi-hat: cyan               z3 Tom: purple
ZONE_FLASH_COLORS: Dict[int, List[int]] = {
    0: [255, 200,  80],   # snare  – warm amber
    1: [255,  20,   0],   # kick   – deep red
    2: [  0, 220, 255],   # hi-hat – cyan
    3: [200,   0, 255],   # tom    – purple
}

# Idle base effect: slow-breathe in a dim blue-indigo glow (fx=2 = Breathe).
_WLED_BASE_STATE: Dict = {
    "on": True, "bri": 60, "tt": 400,
    "seg": [{"fx": 2, "sx": 18, "ix": 80, "col": [[15, 20, 60]]}],
}


class WledController:
    """Sends hit-flash + restore-to-base commands to a WLED device via HTTP JSON API.

    Each hit instantly sets the zone's solid colour at full brightness, then
    schedules a fade back to the slow-breathe base state after `flash_ms`.
    Uses stdlib urllib (no extra dependencies). All calls are fire-and-forget
    on a background thread so they never block the audio worker.
    """

    def __init__(self, host: str, port: int = 80, flash_ms: int = 250) -> None:
        self._url      = f"http://{host}:{port}/json/state"
        self._flash_ms = flash_ms
        self._executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="wled-http"
        )

    def on_hit(self, zone: int, vel: int) -> None:
        """Flash zone colour scaled by velocity, then restore base effect."""
        col   = ZONE_FLASH_COLORS.get(zone, [255, 255, 255])
        # Scale brightness: vel 1-127 → bri 100-255
        bri   = int(100 + 155 * (max(1, min(127, vel)) / 127.0))
        flash = {"on": True, "bri": bri, "tt": 0,
                 "seg": [{"fx": 0, "col": [col]}]}
        self._executor.submit(self._post, flash)
        # Schedule restore after flash_ms
        t = threading.Timer(self._flash_ms / 1000.0,
                            lambda: self._executor.submit(self._post, _WLED_BASE_STATE))
        t.daemon = True
        t.start()

    def set_base(self) -> None:
        """Push the idle glow state (call once on startup)."""
        self._executor.submit(self._post, _WLED_BASE_STATE)

    def _post(self, body: Dict) -> None:
        import urllib.request as _ur, json as _j
        data = _j.dumps(body).encode()
        req  = _ur.Request(self._url, data=data,
                           headers={"Content-Type": "application/json"},
                           method="POST")
        try:
            with _ur.urlopen(req, timeout=0.5):
                pass
        except Exception:
            pass  # silently ignore — LED feedback is best-effort

    def close(self) -> None:
        self._executor.shutdown(wait=False)


# ── UDP worker (runs in a daemon thread) ───────────────────────────────────────

def _udp_worker(
    sock: socket.socket,
    processor: SensorProcessor,
    hit_q: "queue.Queue[tuple]",
    stop: threading.Event,
) -> None:
    while not stop.is_set():
        try:
            data, (ip, _port) = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            evt = json.loads(data.decode("utf-8", errors="replace"))
            if not isinstance(evt, dict):
                continue
        except Exception:
            continue

        if int(evt.get("hb", 0)) == 1:
            hit_q.put(("hb", timestamp(), ip, evt))
            continue

        version = int(evt.get("v", 1))

        if version == 2:
            hit = processor.process(evt)
            if hit is not None:
                hit_q.put(("hit", timestamp(), ip, hit))
        elif version == 1 and "z" in evt and "vel" in evt:
            hit_q.put(("v1hit", timestamp(), ip, evt))


# ── audio/log consumer (runs in a daemon thread) ──────────────────────────────

def _audio_worker(
    hit_q: "queue.Queue[tuple]",
    stop: threading.Event,
    synth: "DrumSynth",
    mixer: "Optional[SoundMixer]",
    no_play: bool,
    min_confidence: int,
    wled: "Optional[WledController]" = None,
) -> None:
    while not stop.is_set():
        try:
            item = hit_q.get(timeout=0.1)
        except queue.Empty:
            continue

        kind, ts, ip, evt = item

        if kind == "hb":
            print(f"\n{ts} {ip} HB imu={evt.get('imu')} mag={evt.get('mag')} "
                  f"sda={evt.get('sda')} scl={evt.get('scl')}")
            continue

        if kind == "hit":
            zone    = evt["zone"]
            vel     = evt["vel"]
            weights = evt.get("weights", {zone: 1.0})
            peak    = evt.get("peak", 0.0)
            if not no_play and mixer is not None:
                velocity = max(1, min(127, vel)) / 127.0
                try:
                    buf = synth.synthesize(zone, velocity=velocity)
                    mixer.play(buf, volume=mixer.effective_volume(zone))
                except Exception as exc:
                    print(f"\naudio_err z{zone}: {exc}", flush=True)
            if wled is not None:
                wled.on_hit(zone, vel)
            blend = "  ".join(
                f"z{z}={w:.2f}" for z, w in sorted(weights.items()) if w >= 0.05
            )
            zn = ZONE_NAMES.get(zone, "?")
            print(f"\n{ts} {ip} HIT z={zone}({zn}) vel={vel} peak={peak:.0f}  [{blend}]")
            continue

        if kind == "v1hit":
            zone = int(evt.get("z",  -1))
            vel  = int(evt.get("vel", 0))
            conf = int(evt.get("conf", 0))
            if conf < min_confidence:
                continue
            if not no_play and mixer is not None:
                velocity = max(1, min(127, vel)) / 127.0
                try:
                    buf = synth.synthesize(zone, velocity=velocity)
                    mixer.play(buf, volume=mixer.effective_volume(zone))
                except Exception as exc:
                    print(f"\naudio_err z{zone}: {exc}", flush=True)
            if wled is not None:
                wled.on_hit(zone, vel)
            print(f"\n{ts} {ip} HIT(v1) z={zone} vel={vel} conf={conf}")


# ── argument parser ────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="WLED Drumstick receiver — Kalman filter + 3D plot"
    )
    p.add_argument("--port", type=int, default=9125)
    p.add_argument("--no-play",  action="store_true", help="Log only, no audio")
    p.add_argument("--no-plot",  action="store_true", help="Disable 3D vispy window")
    p.add_argument("--min-confidence", type=int, default=0, help="v1 packet filter")
    # Swing detection thresholds
    p.add_argument("--vel-forward",  type=float, default=500.0,
                   help="Min forward velocity to start a swing (default 500 LSB/s)")
    p.add_argument("--vel-still",    type=float, default=180.0,
                   help="Max |velocity| to consider stick still (default 180 LSB/s)")
    p.add_argument("--cooldown",     type=int,   default=150,
                   help="Minimum ms between hits (default 150)")
    # Synth control UI
    p.add_argument("--synth-port",   type=int, default=8765,
                   help="HTTP port for browser synth control UI (default 8765)")
    # WLED LED feedback
    p.add_argument("--wled-host",     default="",
                   help="WLED device IP/hostname. If set, flashes LEDs on each hit.")
    p.add_argument("--wled-port",     type=int, default=80,
                   help="WLED HTTP port (default 80)")
    p.add_argument("--wled-flash-ms", type=int, default=250,
                   help="Duration of zone colour flash before fading to base glow (default 250 ms)")
    return p.parse_args()


# ── main ───────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()

    print(f"Listening on UDP port {args.port}")
    print(f"Swing thresholds — forward:{args.vel_forward:.0f}  "
          f"still:{args.vel_still:.0f}  cooldown:{args.cooldown}ms")

    # ── ZoneClassifier ────────────────────────────────────────────────────
    clf = ZoneClassifier()
    if clf.load():
        print(f"Templates loaded from {clf.TEMPLATE_FILE}")
    else:
        print("No saved templates — heading-quadrant fallback until trained (T key).")

    # ── WLED LED controller ───────────────────────────────────────────────
    wled_ctrl: Optional[WledController] = None
    if args.wled_host:
        wled_ctrl = WledController(args.wled_host, args.wled_port, args.wled_flash_ms)
        wled_ctrl.set_base()
        print(f"WLED LED feedback → http://{args.wled_host}:{args.wled_port}/")

    # ── Synthesizer + audio backend ───────────────────────────────────────
    synth = DrumSynth()
    mixer: Optional[SoundMixer] = None
    if not args.no_play:
        try:
            mixer = SoundMixer()
            print("Audio: SoundMixer (polyphonic synthesizer)")
        except Exception as exc:
            print(f"Audio init failed: {exc}", file=sys.stderr)
            return 3

    # ── Browser synth control server ──────────────────────────────────────
    if not args.no_play:
        start_synth_server(synth, mixer, args.synth_port)
        print(f"Synth controls → http://localhost:{args.synth_port}/")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        sock.bind(("0.0.0.0", args.port))
        sock.settimeout(0.2)
    except OSError as exc:
        sock.close()
        if getattr(exc, "winerror", None) == 10048:
            print(f"UDP port {args.port} already in use.", file=sys.stderr)
            return 2
        raise

    processor = SensorProcessor(
        vel_forward = args.vel_forward,
        vel_still   = args.vel_still,
        cooldown_ms = args.cooldown,
    )
    processor.classifier = clf

    hit_q: queue.Queue = queue.Queue()
    stop   = threading.Event()

    t_udp = threading.Thread(
        target=_udp_worker,
        args=(sock, processor, hit_q, stop),
        daemon=True, name="udp-rx",
    )
    t_audio = threading.Thread(
        target=_audio_worker,
        args=(hit_q, stop, synth, mixer,
              args.no_play, args.min_confidence, wled_ctrl),
        daemon=True, name="audio",
    )
    t_udp.start()
    t_audio.start()

    try:
        if HAS_PLOT and not args.no_plot:
            print("3D plot window opening (close it to quit).")
            plot = LivePlot3D(processor)
            plot.run()
        else:
            if not HAS_PLOT and not args.no_plot:
                print("vispy/numpy not available — running without plot.")
            while True:
                threading.Event().wait(1.0)
    finally:
        stop.set()
        if mixer is not None:
            try:
                mixer.close()
            except Exception:
                pass
        if wled_ctrl is not None:
            wled_ctrl.close()
        if clf.trained:
            clf.save()
            print(f"Templates saved → {clf.TEMPLATE_FILE}")
        sock.close()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)

