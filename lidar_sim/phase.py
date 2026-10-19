"""Continuous-wave (phase-shift) ranging and beam-divergence mixed pixels (D2, D12).

CW ranging (Heiden et al. eq. 3): the phase shift of an amplitude-modulated wave is
    Δφ = 4π f R / c  (mod 2π)   ->   R = c Δφ / (4π f),  ambiguous beyond R_amb = c / (2 f).
Two frequencies f1, f2 resolve the ambiguity up to c / (2 |f1 - f2|).
"""

import numpy as np

from . import config

F1, F2 = 46.55e6, 53.2e6  # Hokuyo URG-04LX modulation frequencies


def ambiguity_range(f):
    return config.C / (2.0 * f)


def phase_of(R, f):
    """Wrapped phase shift Δφ ∈ [0, 2π) for true range R."""
    return np.mod(4.0 * np.pi * f * np.asarray(R, dtype=float) / config.C, 2.0 * np.pi)


def phase_range(phi, f):
    """Single-frequency range from a wrapped phase (rolls over every R_amb)."""
    return config.C * np.asarray(phi) / (4.0 * np.pi * f)


def two_frequency_range(phi1, phi2, f1=F1, f2=F2):
    """Unwrap with the beat frequency, then refine with f1.

    coarse = c ((phi1 - phi2) mod 2π) / (4π |f1 - f2|)  (unambiguous to c / 2|f1 - f2|),
    k = round((coarse - R1) / R_amb1),  R = R1 + k R_amb1.
    """
    if f2 > f1:
        phi1, phi2, f1, f2 = phi2, phi1, f2, f1
    coarse = config.C * np.mod(phi1 - phi2, 2.0 * np.pi) / (4.0 * np.pi * (f1 - f2))
    r1 = phase_range(phi1, f1)
    k = np.round((coarse - r1) / ambiguity_range(f1))
    return r1 + k * ambiguity_range(f1)


def mixed_range(ranges, weights, model="linear", f=F1):
    """Range reported for a beam whose sub-rays return at `ranges` with power `weights` (D12).

    linear: power-weighted average (small-separation approximation)
    phasor: exact CW model, z = c/(4π f) arg(Σ w_k exp(i 4π f R_k / c))
    Arrays: (..., n_sub). Sub-rays with no return have weight 0.
    """
    R = np.nan_to_num(np.asarray(ranges, dtype=float))
    w = np.asarray(weights, dtype=float)
    tot = w.sum(axis=-1)
    with np.errstate(invalid="ignore", divide="ignore"):
        if model == "linear":
            z = (w * R).sum(axis=-1) / tot
        elif model == "phasor":
            ph = np.angle((w * np.exp(1j * 4.0 * np.pi * f * R / config.C)).sum(axis=-1))
            z = phase_range(np.mod(ph, 2.0 * np.pi), f)
            # keep the unwrapping branch of the strongest sub-ray (ranges here are < R_amb anyway)
            k = np.round((np.take_along_axis(R, np.argmax(w, axis=-1)[..., None], -1)[..., 0] - z)
                         / ambiguity_range(f))
            z = z + k * ambiguity_range(f)
        else:
            raise ValueError(model)
    return np.where(tot > 0, z, np.nan)
