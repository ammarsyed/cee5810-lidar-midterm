"""Stage 10: CW phase ranging and mixed pixels (D2, D12)."""

import numpy as np

from lidar_sim.phase import F1, F2, ambiguity_range, mixed_range, phase_of, phase_range, two_frequency_range


def test_ambiguity_ranges():
    assert np.isclose(ambiguity_range(F1), 3.220, atol=1e-3)
    assert np.isclose(ambiguity_range(F2), 2.818, atol=1e-3)
    assert np.isclose(ambiguity_range(abs(F1 - F2)), 22.54, atol=0.01)


def test_single_frequency_wraps_and_two_frequency_unwraps():
    R = np.linspace(0.1, 22.0, 500)
    single = phase_range(phase_of(R, F1), F1)
    assert np.allclose(single, np.mod(R, ambiguity_range(F1)))
    assert np.allclose(two_frequency_range(phase_of(R, F1), phase_of(R, F2)), R, atol=1e-9)


def test_mixed_pixel_example_from_d12():
    assert np.isclose(mixed_range([1.0, 2.0], [0.7, 0.3], "linear"), 1.30)
    assert np.isclose(mixed_range([1.0, 2.0], [0.7, 0.3], "phasor"), 1.2265, atol=1e-3)
    assert np.isclose(mixed_range([2.0, 2.0], [0.5, 0.5], "phasor"), 2.0)  # no edge -> exact
    assert np.isnan(mixed_range([np.nan, np.nan], [0.0, 0.0]))
