std_msgs/Header header

# ─── POSITION  (world / odom frame) ───────────────────────────────── [m]
float64 x
float64 y
float64 z

# ─── LINEAR VELOCITY  (body frame) ────────────────────────────────── [m/s]
float64 u       # surge  (forward)
float64 v       # sway   (lateral, +left)
float64 w       # heave  (vertical, +up)

# ─── EULER ANGLES  (RPY, ZYX convention) ──────────────────────────── [rad]
float64 phi     # roll
float64 theta   # pitch
float64 psi     # yaw

# ─── ANGULAR RATES  (body frame) ──────────────────────────────────── [rad/s]
float64 p       # roll  rate
float64 q       # pitch rate
float64 r       # yaw   rate

# ─── WHEEL SPIN VELOCITIES  [fl fr ml mr rl rr] ──────────────────── [rad/s]
float64[6] omega

# ─── STEERING ─────────────────────────────────────────────────────── [rad]
float64 delta       # commanded centre delta (input to Ackermann)
float64 delta_fl    # actual front-left  steer joint angle
float64 delta_fr    # actual front-right steer joint angle

# ─── CONTROL INPUTS ───────────────────────────────────────────────────────
float64 u_throttle  # drive-wheel velocity command [rad/s]
float64 p_brake     # brake pressure normalised [0..1]

# ─── SUSPENSION DEFLECTION  [fl fr ml mr rl rr] ──────────────────── [m]
float64[6] z_suspension
