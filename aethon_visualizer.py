#!/usr/bin/env python3
"""
AETHON-X  ·  Python IMU Motion Visualiser  (ESP32 Bluetooth SPP edition)
-------------------------------------------------------------------------
Reads CSV lines from ESP32 via Bluetooth SPP or USB serial.
Renders:
  • Live 3D drone attitude with full directional thrust vector (X/Y/Z)
  • Rolling gyro-rate plots (X, Y, Z)
  • Pitch / Roll / Yaw angle history graphs
  • Thrust acceleration magnitude graph
  • Translational dynamics: position (X, Y, Z) and velocity (Vx, Vy, Vz)
  • Aerial status badge: LANDED / HOVERING / ASCENDING / DESCENDING /
    MOVING FORWARD / MOVING BACKWARD / MOVING LEFT / MOVING RIGHT /
    TILTED LEFT / TILTED RIGHT / ROTATING CW / ROTATING CCW
  • Full telemetry HUD

Dependencies:  pip install pyserial matplotlib numpy

USAGE
  macOS  : python3 aethon_visualizer.py --port /dev/cu.AETHON-X
  Linux  : python3 aethon_visualizer.py --port /dev/rfcomm0
  Windows: python3 aethon_visualizer.py --port COM5
  Sim    : python3 aethon_visualizer.py --sim
  USB    : python3 aethon_visualizer.py --port /dev/cu.SLAB_USBtoUART
  Alias : aethon
"""

import argparse
import math
import platform
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# ── constants ────────────────────────────────────────────────────
BUF        = 200    # rolling buffer length (samples)
GYRO_LIM   = 300    # deg/s plot y-limits
ANGLE_LIM  = 90     # deg angle plot y-limits
THRUST_LIM = 2.0    # g  thrust plot y-limit
ALPHA_IND  = 0.82   # arc indicator opacity


# ──────────────────────────────────────────────────────────────────
#  HELPERS
# ──────────────────────────────────────────────────────────────────
def _default_port():
    s = platform.system()
    if s == "Darwin": return "/dev/tty.AETHON-X"
    if s == "Linux":  return "/dev/rfcomm0"
    return "COM6"

def _list_ports():
    if serial is None:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


# ──────────────────────────────────────────────────────────────────
#  SERIAL READER THREAD
# ──────────────────────────────────────────────────────────────────
class IMUReader(threading.Thread):
    def __init__(self, port, baud=115200, simulated=False):
        super().__init__(daemon=True)
        self.port      = port
        self.baud      = baud
        self.simulated = simulated
        self.angleX = self.angleY = self.angleZ = 0.0
        self.gyroX  = self.gyroY  = self.gyroZ  = 0.0
        self.accelX = self.accelY = self.accelZ = 0.0
        self.vertZ  = 1.0
        # thrust vector
        self.thrX = 0.0; self.thrY = 0.0; self.thrZ = 1.0
        # landed flag
        self.landed = False
        # translational dynamics (sim only)
        self.posX = self.posY = self.posZ = 0.0
        self.velX = self.velY = self.velZ = 0.0
        self._lock     = threading.Lock()
        self._quit_evt = threading.Event()
        self.connected = False
        self.error_msg = ""

    def stop(self):
        self._quit_evt.set()

    def snapshot(self):
        with self._lock:
            return (self.angleX, self.angleY, self.angleZ,
                    self.gyroX,  self.gyroY,  self.gyroZ,
                    self.accelX, self.accelY, self.accelZ,
                    self.vertZ,
                    self.thrX,   self.thrY,   self.thrZ,
                    self.landed,
                    self.posX,   self.posY,   self.posZ,
                    self.velX,   self.velY,   self.velZ)

    # ── simulated motion ─────────────────────────────────────────
    def _run_sim(self):
        self.connected = True
        t0 = time.time()
        G  = 9.81          # m/s²
        DT = 0.02          # matches sleep below
        vx = vy = vz = 0.0
        px = py = pz = 0.0
        while not self._quit_evt.is_set():
            t   = time.time() - t0
            ax  = math.degrees(math.sin(t * 0.7) * 0.35)
            ay  = math.degrees(math.cos(t * 0.5) * 0.30)
            az  = math.degrees(math.sin(t * 0.2) * 0.60)
            gx  = math.degrees( 0.7 * 0.35 * math.cos(t * 0.7))
            gy  = math.degrees(-0.5 * 0.30 * math.sin(t * 0.5))
            gz  = math.degrees( 0.2 * 0.60 * math.cos(t * 0.2))
            pitch  = math.radians(ax); roll = math.radians(ay)
            agx    = -math.sin(pitch) + 0.05 * math.sin(t * 3.1)
            agy    = -math.cos(pitch) * math.sin(roll)
            agz_wf =  math.cos(pitch) * math.cos(roll)

            # ── translational dynamics ──────────────────────────
            # World-frame accelerations derived from fused angles.
            # waz - 1.0 removes gravity baseline so 0 = hover.
            waz      = agz_wf                          # vertical accel proxy (g)
            ax_w     =  math.sin(pitch) * G            # forward/back  (m/s²)
            ay_w     = -math.sin(roll)  * G            # left/right    (m/s²)
            az_w     =  (waz - 1.0)    * G             # up/down       (m/s²)
            # light drag so position doesn't drift to infinity
            drag     = 0.18
            vx = vx * (1 - drag) + ax_w * DT
            vy = vy * (1 - drag) + ay_w * DT
            vz = vz * (1 - drag) + az_w * DT
            px += vx * DT
            py += vy * DT
            pz += vz * DT

            # ── thrust vector (body-frame scaled by waz) ────────
            thr_x =  math.sin(pitch)                   * waz
            thr_y = -math.cos(pitch) * math.sin(roll)  * waz
            thr_z =  math.cos(pitch) * math.cos(roll)  * waz

            # ── landed detection ────────────────────────────────
            is_still = (abs(gx) < 2.0 and abs(gy) < 2.0 and abs(gz) < 2.0
                        and abs(ax) < 3.0 and abs(ay) < 3.0)

            with self._lock:
                self.angleX, self.angleY, self.angleZ = ax,    ay,    az
                self.gyroX,  self.gyroY,  self.gyroZ  = gx,    gy,    gz
                self.accelX, self.accelY, self.accelZ = agx,   agy,   agz_wf
                self.vertZ  = agz_wf
                self.thrX,  self.thrY,   self.thrZ    = thr_x, thr_y, thr_z
                self.landed = is_still
                self.posX,  self.posY,   self.posZ    = px,    py,    pz
                self.velX,  self.velY,   self.velZ    = vx,    vy,    vz
            time.sleep(DT)

    # ── live serial ───────────────────────────────────────────────
    def _run_serial(self):
        if serial is None:
            self.error_msg = "pyserial not installed — run: pip install pyserial"
            return
        available = _list_ports()

        # macOS Tahoe: bluetoothd accumulates stale RFCOMM state after a
        # connection closes, causing subsequent opens to see in_waiting=0
        # forever. Restarting the daemon before each connection gives a
        # clean slate. The daemon auto-relaunches via launchd within ~2 s.
        # macOS Tahoe: poll until the BT SPP port appears in /dev.
        # The cu. device node can take 10-20 s to appear after the
        # Python process starts, especially on second/subsequent runs.
        if platform.system() == "Darwin" and any(
            k in self.port for k in ('AETHON', 'tty.', 'cu.', 'rfcomm')
        ):
            print("AETHON-X: waiting for BT port...", flush=True)
            for _ in range(40):          # up to 20 s
                if self.port in _list_ports():
                    print("AETHON-X: port ready.", flush=True)
                    break
                time.sleep(0.5)
            else:
                print("AETHON-X: port not seen — attempting open anyway...",
                      flush=True)

        # Retry loop: BT SPP port can appear in /dev but not be ready
        # to accept a connection for several seconds after power-on.
        ser = None
        for attempt in range(10):
            try:
                # macOS Tahoe SPP: use 1 s timeout so each read(1) call
                # doesn't stall for 5 s on an empty buffer.
                # rtscts/dsrdtr/xonxoff must all be False — macOS Tahoe
                # holds bytes in the RFCOMM buffer waiting for flow-control
                # signals that SPP never asserts, starving the TTY entirely.
                ser = serial.Serial(
                    self.port, self.baud, timeout=1,
                    rtscts=False, dsrdtr=False, xonxoff=False
                )
                break
            except serial.SerialException:
                time.sleep(1.0)
        if ser is None:
            self.error_msg = (
                f"Cannot open '{self.port}' after 10 attempts.\n"
                f"Available: {available or ['(none)']}\n"
                "Use --sim to test without hardware."
            )
            return
        # BT SPP: skip reset_input_buffer — it can block or stall
        # on wireless connections. Drain manually instead.
        # cu. prefix (macOS callout device) is also a BT indicator.
        is_bt = ('AETHON' in self.port or 'tty.' in self.port
                 or 'cu.'  in self.port or 'rfcomm' in self.port)
        if not is_bt:
            ser.reset_input_buffer()
        else:
            time.sleep(2.0)  # let BT SPP stream settle before reading
            ser.flushInput() if hasattr(ser, 'flushInput') else None
        self.connected = True

        # ── raw byte accumulator ──────────────────────────────────
        # macOS Tahoe's SPP driver often never delivers a '\n' to the
        # serial buffer, so ser.readline() times out every call and
        # returns b"".  Reading one byte at a time and reassembling
        # lines manually bypasses that driver-level line-buffering
        # issue entirely.
        line_buf = bytearray()

        def _process_line(raw: str):
            """Parse one CSV line and update shared state."""
            raw = raw.strip()
            if not raw or len(raw) < 5:
                return
            # Skip any leading non-CSV noise (garbage bytes at BT connect)
            for i, ch in enumerate(raw):
                if ch in '0123456789-+':
                    raw = raw[i:]
                    break
            parts = raw.split(",")
            if len(parts) < 6:
                return
            try:
                nx,  ny,  nz  = float(parts[0]), float(parts[1]), float(parts[2])
                ngx, ngy, ngz = float(parts[3]), float(parts[4]), float(parts[5])
                if len(parts) >= 9:
                    nax  = float(parts[6])
                    nay  = float(parts[7])
                    nwaz = float(parts[8])
                else:
                    p    = math.radians(nx); r = math.radians(ny)
                    nax  = -math.sin(p)
                    nay  = -math.cos(p) * math.sin(r)
                    nwaz =  math.cos(p) * math.cos(r)
                # thrust vector — use fields 9-11 if present, else compute
                if len(parts) >= 12:
                    ntx = float(parts[9])
                    nty = float(parts[10])
                    ntz = float(parts[11])
                else:
                    p = math.radians(nx); r = math.radians(ny)
                    ntx =  math.sin(p)                  * nwaz
                    nty = -math.cos(p) * math.sin(r)    * nwaz
                    ntz =  math.cos(p) * math.cos(r)    * nwaz
                # landed flag — use field 12 if present
                nlanded = bool(int(float(parts[12]))) if len(parts) >= 13 else False
            except ValueError:
                return
            with self._lock:
                self.angleX, self.angleY, self.angleZ = nx,   ny,   nz
                self.gyroX,  self.gyroY,  self.gyroZ  = ngx,  ngy,  ngz
                self.accelX, self.accelY, self.accelZ = nax,  nay,  nwaz
                self.vertZ  = nwaz
                self.thrX,  self.thrY,   self.thrZ    = ntx,  nty,  ntz
                self.landed = nlanded

        while not self._quit_evt.is_set():
            try:
                byte = ser.read(1)   # returns b"" on timeout, never raises
            except Exception:
                continue
            if not byte:             # timeout — loop and retry
                continue
            ch = byte[0]
            if ch in (ord('\n'), ord('\r')):
                if line_buf:
                    _process_line(line_buf.decode("ascii", "ignore"))
                    line_buf.clear()
            else:
                line_buf.append(ch)
                # Safety valve: discard absurdly long garbage lines
                if len(line_buf) > 512:
                    line_buf.clear()
        ser.close()

    def run(self):
        self._run_sim() if self.simulated else self._run_serial()


# ──────────────────────────────────────────────────────────────────
#  3D DRONE GEOMETRY
# ──────────────────────────────────────────────────────────────────
def drone_geometry():
    segs = []
    bx, by, bz = 0.6, 0.15, 0.6
    box = np.array([
        [-bx,-by,-bz],[bx,-by,-bz],[bx,-by,bz],[-bx,-by,bz],[-bx,-by,-bz],
        [-bx, by,-bz],[bx, by,-bz],[bx, by,bz],[-bx, by,bz],[-bx, by,-bz],
    ])
    segs.append(box)
    for x in (-bx, bx):
        for z in (-bz, bz):
            segs.append(np.array([[x,-by,z],[x,by,z]]))
    arm_len = 1.4
    for sx, sz in [(1,1),(1,-1),(-1,1),(-1,-1)]:
        tip  = np.array([sx*arm_len, 0, sz*arm_len])
        segs.append(np.array([[0,0,0], tip]))
        th   = np.linspace(0, 2*math.pi, 32)
        disc = np.stack([
            tip[0] + 0.45*np.cos(th),
            np.full_like(th, tip[1]+0.05),
            tip[2] + 0.45*np.sin(th),
        ], axis=1)
        segs.append(disc)
    segs.append(np.array([[0,0,0],[0,0,1.6]]))   # nose (red)
    return segs


def rot_matrix(pitch_deg, roll_deg, yaw_deg):
    # Renderer: body-X→display-X, body-Y→display-vertical, body-Z→display-depth
    # pitch(angleX)→Rx   roll(angleY)→Rz(-r)   yaw(angleZ)→Ry
    # Composition: Ry(yaw) @ Rz(-roll) @ Rx(pitch)
    p, r, y = map(math.radians, (pitch_deg, roll_deg, yaw_deg))
    cx, sx = math.cos(p),  math.sin(p)
    cz, sz = math.cos(r), -math.sin(r)   # Rz(-r): +roll = right side DOWN
    cy, sy = math.cos(y),  math.sin(y)
    Rx = np.array([[1,  0,   0 ], [0,  cx, -sx], [0,  sx,  cx]])
    Rz = np.array([[cz,-sz,  0 ], [sz,  cz,  0], [0,  0,   1 ]])
    Ry = np.array([[cy,  0, sy ], [0,   1,   0], [-sy, 0,  cy]])
    return Ry @ Rz @ Rx


def _arc_points(rate_deg_s, axis, radius=1.85):
    span  = np.clip(rate_deg_s / GYRO_LIM, -1, 1) * 60
    th    = np.linspace(0, math.radians(span), 24)
    zeros = np.zeros_like(th)
    c, s  = radius * np.cos(th), radius * np.sin(th)
    if   axis == 'x': return np.stack([zeros, c, s], axis=1)
    elif axis == 'y': return np.stack([c, zeros, s], axis=1)
    else:             return np.stack([c, s, zeros], axis=1)


# ──────────────────────────────────────────────────────────────────
#  AERIAL STATUS
# ──────────────────────────────────────────────────────────────────
# Priority order:
#   1. LANDED            — still for >1.5 s
#   2. ASCENDING/DESC    — waz significantly above/below 1g
#   3. ROTATING CW/CCW   — gyroZ rate above threshold
#   4. MOVING FWD/BACK   — pitch angle (drone tilts to translate)
#   5. MOVING L/R        — roll angle  (drone tilts to translate)
#   6. TILTED R/L        — static roll without strong translation
#   7. HOVERING          — default when nothing else fires

TILT_MOVE  = 8.0    # deg  — pitch/roll threshold for "moving" intent
TILT_STATIC= 3.0    # deg  — roll threshold for "tilted" (less than move)
ROT_THRESH = 15.0   # °/s  — gyroZ rate threshold for rotation detection

STATUS_COLORS = {
    "LANDED":    "#888888",
    "HOVERING":  "#00FFAA",
    "ASCENDING": "#00FF44",
    "DESCENDING":"#FF4400",
    "FORWARD":   "#00AAFF",
    "BACKWARD":  "#FF00AA",
    "LEFT":      "#FFAA00",
    "RIGHT":     "#AA44FF",
    "TILT_R":    "#FFFF00",
    "TILT_L":    "#FFDD00",
    "ROT_CW":    "#FF8800",
    "ROT_CCW":   "#88FF00",
}

def _aerial_status(pitch, roll, waz, gz, landed):
    if landed:
        return "▣  LANDED",          STATUS_COLORS["LANDED"]
    if waz > 1.20:
        return "▲  ASCENDING",       STATUS_COLORS["ASCENDING"]
    if waz < 0.80:
        return "▼  DESCENDING",      STATUS_COLORS["DESCENDING"]
    if gz >  ROT_THRESH:
        return "↻  ROTATING CW",     STATUS_COLORS["ROT_CW"]
    if gz < -ROT_THRESH:
        return "↺  ROTATING CCW",    STATUS_COLORS["ROT_CCW"]
    if pitch >  TILT_MOVE:
        return "▶  MOVING FORWARD",  STATUS_COLORS["FORWARD"]
    if pitch < -TILT_MOVE:
        return "◀  MOVING BACKWARD", STATUS_COLORS["BACKWARD"]
    if roll  >  TILT_MOVE:
        return "▷  MOVING RIGHT",    STATUS_COLORS["RIGHT"]
    if roll  < -TILT_MOVE:
        return "◁  MOVING LEFT",     STATUS_COLORS["LEFT"]
    if roll  >  TILT_STATIC:
        return "↗  TILTED RIGHT",    STATUS_COLORS["TILT_R"]
    if roll  < -TILT_STATIC:
        return "↖  TILTED LEFT",     STATUS_COLORS["TILT_L"]
    return     "—  HOVERING",        STATUS_COLORS["HOVERING"]


# ──────────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="AETHON-X IMU visualiser")
    ap.add_argument("--port", default=_default_port())
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim",  action="store_true")
    args = ap.parse_args()

    reader = IMUReader(args.port, args.baud, simulated=args.sim)
    reader.start()
    # BT SPP needs more time to handshake than USB — wait up to 12s
    for _ in range(12):
        if reader.connected:
            break
        time.sleep(1.0)
    if not args.sim and not reader.connected:
        print("\n[AETHON-X ERROR]", reader.error_msg or "Connection failed.", file=sys.stderr)
        print("Available ports:", _list_ports() or "(none)", file=sys.stderr)
        reader.stop(); sys.exit(1)

    geom = drone_geometry()
    src  = "SIM" if args.sim else args.port

    # ── figure layout ─────────────────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(22, 8))
    gs  = GridSpec(4, 4,
                   width_ratios=[2.4, 1.0, 1.0, 1.0],
                   height_ratios=[1, 1, 1, 1],
                   figure=fig, hspace=0.55, wspace=0.38)

    ax3d    = fig.add_subplot(gs[:, 0], projection="3d")
    axGx    = fig.add_subplot(gs[0, 1])
    axGy    = fig.add_subplot(gs[1, 1])
    axGz    = fig.add_subplot(gs[2, 1])
    axHud   = fig.add_subplot(gs[3, 1]); axHud.axis("off")
    axPitch  = fig.add_subplot(gs[0, 2])
    axRoll   = fig.add_subplot(gs[1, 2])
    axYaw    = fig.add_subplot(gs[2, 2])
    axThrust = fig.add_subplot(gs[3, 2])
    # translational panels (col 3)
    axPosX   = fig.add_subplot(gs[0, 3])
    axPosY   = fig.add_subplot(gs[1, 3])
    axPosZ   = fig.add_subplot(gs[2, 3])
    axVel    = fig.add_subplot(gs[3, 3])

    # ── 3D scene ──────────────────────────────────────────────────
    L = 2.2
    ax3d.set_xlim(-L, L); ax3d.set_ylim(-L, L); ax3d.set_zlim(-L, L)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.view_init(elev=25, azim=-60)
    ax3d.set_facecolor("#06080F")
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor("#1a2235")
    ax3d.grid(color="#13203a", linestyle=":")
    ax3d.set_xlabel("X", color="#334")
    ax3d.set_ylabel("Y", color="#334")
    ax3d.set_zlabel("Z", color="#334")
    ax3d.set_title(f"AETHON-X  Drone Attitude  [{src}]",
                   color="#00FFAA", pad=14, fontsize=11)

    drone_lines = [ax3d.plot([], [], [], color="#00FFAA", lw=1.4)[0] for _ in geom]
    drone_lines[-1].set_color("#FF2244"); drone_lines[-1].set_linewidth(2.4)

    arc_x_line, = ax3d.plot([], [], [], color="#3DD6FF", lw=2.2, alpha=ALPHA_IND)
    arc_y_line, = ax3d.plot([], [], [], color="#FF44AA", lw=2.2, alpha=ALPHA_IND)
    arc_z_line, = ax3d.plot([], [], [], color="#FFB800", lw=2.2, alpha=ALPHA_IND)
    ax3d.text2D(0.01, 0.01, "■ gyroX  ■ gyroY  ■ gyroZ  ▶ thrust",
                transform=ax3d.transAxes, fontsize=7, color="#557",
                verticalalignment="bottom")

    thrust_line, = ax3d.plot([], [], [], color="#FFFFFF", lw=2.0,
                              alpha=0.85, linestyle="--")
    thrust_tip,  = ax3d.plot([], [], [], color="#FFFFFF", marker="^",
                              markersize=7, alpha=0.85, linestyle="none")

    # Status badge — updated each frame by draw_frame
    status_txt = ax3d.text2D(0.50, 0.97, "—  HOVERING",
                              transform=ax3d.transAxes,
                              ha="center", va="top",
                              fontsize=11, fontweight="bold",
                              color="#00FFAA",
                              bbox=dict(boxstyle="round,pad=0.35",
                                        facecolor="#0a1020",
                                        edgecolor="#00FFAA", linewidth=1.4))

    # ── rolling plots ─────────────────────────────────────────────
    xs = np.arange(BUF)

    def _make_plot(ax, color, title, ylim, unit=""):
        buf   = deque([0.0] * BUF, maxlen=BUF)
        line, = ax.plot(xs, list(buf), color=color, lw=1.1)
        ax.set_ylim(-ylim, ylim); ax.set_xlim(0, BUF)
        ax.set_facecolor("#09101c")
        ax.set_title(f"{title}  ({unit})" if unit else title,
                     color=color, fontsize=8)
        ax.tick_params(colors="#334", labelsize=7)
        ax.axhline(0, color="#1a2235", lw=0.8)
        ax.set_xticks([])
        return buf, line

    gxBuf,  lineGx     = _make_plot(axGx,     "#3DD6FF", "gyroX",      GYRO_LIM,  "°/s")
    gyBuf,  lineGy     = _make_plot(axGy,     "#FF44AA", "gyroY",      GYRO_LIM,  "°/s")
    gzBuf,  lineGz     = _make_plot(axGz,     "#FFB800", "gyroZ",      GYRO_LIM,  "°/s")
    pBuf,   linePitch  = _make_plot(axPitch,  "#00FFAA", "Pitch",      ANGLE_LIM, "deg")
    rBuf,   lineRoll   = _make_plot(axRoll,   "#FF6600", "Roll",       ANGLE_LIM, "deg")
    yBuf,   lineYaw    = _make_plot(axYaw,    "#AA88FF", "Yaw",        180,       "deg")
    thBuf,  lineThrust = _make_plot(axThrust, "#FFFFFF", "Thrust |a|", THRUST_LIM, "g")
    axThrust.set_ylim(0, THRUST_LIM)
    # translational panels
    POS_LIM = 5.0   # metres
    VEL_LIM = 3.0   # m/s
    pxBuf, linePosX = _make_plot(axPosX, "#00FFFF",  "Pos X",  POS_LIM, "m")
    pyBuf, linePosY = _make_plot(axPosY, "#FF88FF",  "Pos Y",  POS_LIM, "m")
    pzBuf, linePosZ = _make_plot(axPosZ, "#88FF88",  "Pos Z",  POS_LIM, "m")
    # velocity: three lines on one axes
    velXBuf = deque([0.0] * BUF, maxlen=BUF)
    velYBuf = deque([0.0] * BUF, maxlen=BUF)
    velZBuf = deque([0.0] * BUF, maxlen=BUF)
    axVel.set_facecolor("#09101c"); axVel.set_ylim(-VEL_LIM, VEL_LIM)
    axVel.set_xlim(0, BUF); axVel.set_xticks([])
    axVel.axhline(0, color="#1a2235", lw=0.8)
    axVel.set_title("Velocity  (m/s)", color="#AAAAAA", fontsize=8)
    axVel.tick_params(colors="#334", labelsize=7)
    lineVx, = axVel.plot(xs, list(velXBuf), color="#00FFFF", lw=1.1, label="Vx")
    lineVy, = axVel.plot(xs, list(velYBuf), color="#FF88FF", lw=1.1, label="Vy")
    lineVz, = axVel.plot(xs, list(velZBuf), color="#88FF88", lw=1.1, label="Vz")
    axVel.legend(fontsize=6, loc="upper right", framealpha=0.3)

    hud = axHud.text(0.02, 0.97, "", color="#00FFAA",
                     family="monospace", fontsize=8.2, va="top",
                     transform=axHud.transAxes)

    # ── draw loop ────────────────────────────────────────────────
    # plt.pause() manual loop replaces FuncAnimation entirely.
    # FuncAnimation(blit=False) does not repaint 3D axes on macOS,
    # Windows, or Linux reliably. plt.pause() pumps the GUI event
    # loop AND forces a full canvas redraw on every call — the only
    # portable fix that works across all backends and all OS.
    fps_c = [0]; fps_v = [0]; last_t = [time.time()]

    def draw_frame():
        ax_, ay_, az_, gx_, gy_, gz_, acx, acy, acz, vert_g, \
            thr_x, thr_y, thr_z, landed, \
            px_, py_, pz_, vx_, vy_, vz_ = reader.snapshot()
        R = rot_matrix(ax_, ay_, az_)

        for line, seg in zip(drone_lines, geom):
            w = seg @ R.T
            line.set_data(w[:, 0], w[:, 2])
            line.set_3d_properties(w[:, 1])

        for arc_line, rate, axis in [
            (arc_x_line, gx_, 'x'),
            (arc_y_line, gy_, 'y'),
            (arc_z_line, gz_, 'z'),
        ]:
            pts = _arc_points(rate, axis) @ R.T
            arc_line.set_data(pts[:, 0], pts[:, 2])
            arc_line.set_3d_properties(pts[:, 1])

        # ── Full 3D directional thrust vector ─────────────────────
        # thrX/Y/Z from IMU are body-frame; rotate into display frame.
        thrust_body = np.array([thr_x, thr_y, thr_z])
        t_mag = np.linalg.norm(thrust_body)
        if t_mag > 1e-3:
            thrust_body = thrust_body / t_mag * min(t_mag * 1.4, 1.8)
        tv = thrust_body @ R.T
        thrust_line.set_data([0, tv[0]], [0, tv[2]])
        thrust_line.set_3d_properties([0, tv[1]])
        thrust_tip.set_data([tv[0]], [tv[2]])
        thrust_tip.set_3d_properties([tv[1]])

        gxBuf.append(gx_); gyBuf.append(gy_); gzBuf.append(gz_)
        lineGx.set_ydata(list(gxBuf))
        lineGy.set_ydata(list(gyBuf))
        lineGz.set_ydata(list(gzBuf))

        pBuf.append(ax_); rBuf.append(ay_); yBuf.append(az_)
        linePitch.set_ydata(list(pBuf))
        lineRoll.set_ydata(list(rBuf))
        lineYaw.set_ydata(list(yBuf))

        thrust_mag = abs(vert_g)
        thBuf.append(thrust_mag)
        lineThrust.set_ydata(list(thBuf))

        # ── translational dynamics ─────────────────────────────────
        pxBuf.append(px_); linePosX.set_ydata(list(pxBuf))
        pyBuf.append(py_); linePosY.set_ydata(list(pyBuf))
        pzBuf.append(pz_); linePosZ.set_ydata(list(pzBuf))
        velXBuf.append(vx_); lineVx.set_ydata(list(velXBuf))
        velYBuf.append(vy_); lineVy.set_ydata(list(velYBuf))
        velZBuf.append(vz_); lineVz.set_ydata(list(velZBuf))
        for buf, line, ax_p in [
            (pxBuf, linePosX, axPosX),
            (pyBuf, linePosY, axPosY),
            (pzBuf, linePosZ, axPosZ),
        ]:
            lo, hi = min(buf), max(buf)
            pad = max(abs(hi - lo) * 0.15, 0.5)
            ax_p.set_ylim(lo - pad, hi + pad)

        # ── Aerial status ──────────────────────────────────────────
        status_str, status_color = _aerial_status(ax_, ay_, vert_g, gz_, landed)
        status_txt.set_text(status_str)
        status_txt.set_color(status_color)
        status_txt.get_bbox_patch().set_edgecolor(status_color)

        fps_c[0] += 1
        now = time.time()
        if now - last_t[0] >= 1.0:
            fps_v[0] = fps_c[0]; fps_c[0] = 0; last_t[0] = now

        # Horizontal motion indicators for HUD
        fwd_str = f"{-acx:+.3f} g" + (" ▶FWD"  if acx < -0.12 else
                                        " ◀BACK" if acx >  0.12 else "")
        lat_str = f"{-acy:+.3f} g" + (" ▷RGT"  if acy >  0.12 else
                                        " ◁LFT"  if acy < -0.12 else "")

        hud.set_text(
            f"// AETHON-X  TELEMETRY\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f" STATUS  : {status_str}\n"
            f"\n"
            f" pitch X : {ax_:+7.2f} deg\n"
            f" roll  Y : {ay_:+7.2f} deg\n"
            f" yaw   Z : {az_:+7.2f} deg\n"
            f"\n"
            f" gyroX   : {gx_:+7.1f} °/s\n"
            f" gyroY   : {gy_:+7.1f} °/s\n"
            f" gyroZ   : {gz_:+7.1f} °/s (yaw)\n"
            f"\n"
            f" fwd/bk  : {fwd_str}\n"
            f" lft/rgt : {lat_str}\n"
            f" vert-g  : {vert_g:+7.3f} g\n"
            f" thrust  : {thrust_mag:+7.3f} g\n"
            f"\n"
            f" pos X   : {px_:+7.2f} m\n"
            f" pos Y   : {py_:+7.2f} m\n"
            f" pos Z   : {pz_:+7.2f} m\n"
            f" vel X   : {vx_:+7.2f} m/s\n"
            f" vel Y   : {vy_:+7.2f} m/s\n"
            f" vel Z   : {vz_:+7.2f} m/s\n"
            f"\n"
            f" fps     : {fps_v[0]}\n"
            f" source  : {src}"
        )

    try:
        try:
            plt.get_current_fig_manager().set_window_title(
                "AETHON-X · IMU Live Visualiser")
        except Exception:
            pass
        plt.ion()
        plt.show(block=False)
        while plt.get_fignums():
            draw_frame()
            plt.pause(0.033)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        reader.join(timeout=1)
        plt.ioff()



if __name__ == "__main__":
    main()
