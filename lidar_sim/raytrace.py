"""Batched, breadth-first, backend-agnostic ray tracer that turns beams into returns.

Rules (plan/03_CODE_PLAN.md section 5):
- every hit on a surface with rho > 0 makes a diffuse return; matte surfaces stop the ray
- mirror and glass hits make a head-on specular-lobe return and spawn child rays:
  mirror -> reflected (x R_m); glass -> straight through (x (1-R_F)^2) and reflected (x R_F)
- all active rays are cast together, one backend call per bounce depth
- throughput T is one-way; powers use T^2 (out and back) and the total path length L
"""

from dataclasses import dataclass

import numpy as np

from . import config
from .materials import GLASS, MIRROR
from .physics import fresnel, reflect


@dataclass
class Returns:
    """Struct of arrays: one row per return (a beam can have several)."""

    beam_idx: np.ndarray
    path_length: np.ndarray
    power: np.ndarray
    hit_point: np.ndarray
    apparent_point: np.ndarray
    surface_id: np.ndarray
    kind: np.ndarray  # 0 = diffuse, 1 = specular
    reflections: np.ndarray  # mirror/glass reflections before this return (ghost if > 0)
    interactions: np.ndarray  # all mirror/glass interactions (reflections + transmissions)
    theta_inc: np.ndarray

    def __len__(self):
        return len(self.beam_idx)


def _concat(parts, d):
    if not parts:
        e = np.zeros(0)
        return Returns(e.astype(int), e, e, np.zeros((0, d)), np.zeros((0, d)), e.astype(int), e.astype(int),
                       e.astype(int), e.astype(int), e)
    cols = list(zip(*parts))
    return Returns(*(np.concatenate(c) for c in cols))


def trace(backend, origins, dirs, *, fog_alpha=0.0, max_bounces=config.MAX_BOUNCES,
          k=config.K_OVER_PTH, spec_gain=config.SPEC_GAIN, lobe_deg=config.SPEC_LOBE_DEG,
          eps_self=None, prune=config.PRUNE_REL, min_power=1e-6):
    """Trace a batch of beams (one row of origins/dirs per beam) and return every return."""
    origins = np.asarray(origins, dtype=float)
    dirs = np.asarray(dirs, dtype=float)
    M, d = origins.shape
    if eps_self is None:
        eps_self = config.EPS_SELF_2D if d == 2 else config.EPS_SELF_3D
    mat = backend.mat
    w2 = 2.0 * np.radians(lobe_deg) ** 2
    sensor_o, u0 = origins, dirs

    o, u = origins, dirs
    beam = np.arange(M)
    L = np.zeros(M)
    T = np.ones(M)
    n_ref = np.zeros(M, dtype=int)
    n_int = np.zeros(M, dtype=int)
    parts = []

    for depth in range(max_bounces + 1):
        if len(o) == 0:
            break
        h = backend.cast(o, u)
        ok = h.hit
        if not ok.any():
            break
        o, u, beam, L, T, n_ref, n_int = o[ok], u[ok], beam[ok], L[ok], T[ok], n_ref[ok], n_int[ok]
        t, pt, nrm, sid = h.t[ok], h.point[ok], h.normal[ok], h.surface_id[ok]

        Lh = L + t
        cosi = np.clip(-np.sum(u * nrm, axis=1), 0.0, 1.0)
        theta = np.arccos(cosi)
        code = mat.code[sid]
        rho = mat.rho[sid]
        base = k * T * T * np.exp(-2.0 * fog_alpha * Lh) / (Lh * Lh)
        apparent = sensor_o[beam] + Lh[:, None] * u0[beam]

        def emit(mask, power, kind):
            if mask.any():
                parts.append((beam[mask], Lh[mask], power[mask], pt[mask], apparent[mask], sid[mask],
                              np.full(mask.sum(), kind), n_ref[mask], n_int[mask], theta[mask]))

        p_diff = base * rho * cosi
        emit((rho > 0) & (p_diff > min_power), p_diff, 0)

        is_m, is_g = code == MIRROR, code == GLASS
        RF = np.where(is_g, fresnel(cosi, 1.0, mat.n[sid]), 0.0)
        Rs = np.where(is_m, mat.reflectance[sid], RF)
        p_spec = base * spec_gain * Rs * np.exp(-theta * theta / w2)
        emit((is_m | is_g) & (p_spec > min_power), p_spec, 1)

        if depth == max_bounces:
            break
        refl = reflect(u, nrm)
        kids = []  # (index into current arrays, direction, throughput, is_reflection)
        im, ig = np.flatnonzero(is_m), np.flatnonzero(is_g)
        kids.append((im, refl[im], T[im] * mat.reflectance[sid[im]], 1))
        kids.append((ig, u[ig], T[ig] * (1.0 - RF[ig]) ** 2, 0))
        kids.append((ig, refl[ig], T[ig] * RF[ig], 1))

        no, nu, nb, nL, nT, nr, ni = [], [], [], [], [], [], []
        for idx, ndir, nthr, is_ref in kids:
            if len(idx) == 0:
                continue
            best = k * spec_gain * nthr**2 * np.exp(-2.0 * fog_alpha * Lh[idx]) / Lh[idx] ** 2
            keep = best >= prune
            idx, ndir, nthr = idx[keep], ndir[keep], nthr[keep]
            no.append(pt[idx] + eps_self * ndir)
            nu.append(ndir)
            nb.append(beam[idx])
            nL.append(Lh[idx])
            nT.append(nthr)
            nr.append(n_ref[idx] + is_ref)
            ni.append(n_int[idx] + 1)
        if not no:
            break
        o, u, beam = np.concatenate(no), np.concatenate(nu), np.concatenate(nb)
        L, T = np.concatenate(nL), np.concatenate(nT)
        n_ref, n_int = np.concatenate(nr), np.concatenate(ni)

    return _concat(parts, d)


def select(returns: Returns, n_beams, policy="strongest", threshold=1.0):
    """Index of the chosen return per beam (-1 if the beam has no return).

    strongest: max power. first: nearest return with power >= threshold, else the strongest.
    """
    chosen = np.full(n_beams, -1, dtype=int)
    if len(returns) == 0:
        return chosen
    b, p, L = returns.beam_idx, returns.power, returns.path_length
    order = np.lexsort((-p, b))  # by beam, then descending power
    first_of_beam = np.unique(b[order], return_index=True)[1]
    chosen[b[order][first_of_beam]] = order[first_of_beam]
    if policy == "first":
        strong = np.flatnonzero(p >= threshold)
        if len(strong):
            order = strong[np.lexsort((L[strong], b[strong]))]
            fo = np.unique(b[order], return_index=True)[1]
            chosen[b[order][fo]] = order[fo]
    return chosen
