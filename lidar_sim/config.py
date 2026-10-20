"""Default parameter values (docs/01_PROJECT_PLAN.md section 4).

All powers in the code are normalised by the detection threshold, so a return is
"above threshold" when its normalised power p = P_r / P_th >= 1.
"""

import numpy as np

C = 299_792_458.0  # speed of light [m/s]

# --- Range noise (D1) -------------------------------------------------------
SIGMA_R = 0.01  # range noise std. dev. [m]
SIGMA_T = 2.0 * SIGMA_R / C  # equivalent timing jitter (~66.7 ps)

# --- Power calibration (D5) -------------------------------------------------
R_MAX_REF = 5.6  # clear-air max range [m] for a white wall head-on (Hokuyo spec)
RHO_REF = 0.9  # reflectivity of that white wall
K_OVER_PTH = R_MAX_REF**2 / RHO_REF  # K / P_th [m^2], ~34.84

# --- Specular lobe for head-on mirror/glass returns (D5, modelling assumption)
SPEC_LOBE_DEG = 1.5  # theta_w
# The lobe exp(-theta_inc^2 / (2 theta_w^2)) is written in incidence angle, but the mirror direction
# leaves the surface 2 theta_inc away from the sensor, so in outgoing angle the lobe is 2 theta_w wide.
# Normalising that Gaussian over solid angle (energy-conserving) and comparing with the Lambertian
# 1/pi lobe gives K_s / K = 1 / (8 theta_w^2)  (~182 for 1.5 deg). See D5 and DECISIONS.md #2.
SPEC_GAIN = 1.0 / (8.0 * np.radians(SPEC_LOBE_DEG) ** 2)

# --- Detection (D6) ---------------------------------------------------------
SIGMA_P_REL = 0.1  # power noise std. dev. as a fraction of P_th

# --- Materials ----------------------------------------------------------------
N_GLASS = 1.5
R_MIRROR = 0.95
RHO_WALL = 0.7

# --- Tracing --------------------------------------------------------------------
MAX_BOUNCES = 5
EPS_SELF_2D = 1e-7  # offset for secondary rays [m] (float64 segments)
EPS_SELF_3D = 1e-4  # offset for secondary rays [m] (Embree is float32)
PRUNE_REL = 1e-3  # drop child rays whose best-case power < PRUNE_REL * P_th

# --- 2D sensor (Hokuyo-like) ---------------------------------------------------
FOV_DEG = 240.0
N_BEAMS = 241
R_MIN = 0.02

# --- 3D sensor (VLP-16-like geometry, same power model) ------------------------
N_RINGS = 16
ELEV_MIN_DEG = -15.0
ELEV_MAX_DEG = 15.0
AZ_STEP_DEG = 1.0
SENSOR_HEIGHT = 0.6
