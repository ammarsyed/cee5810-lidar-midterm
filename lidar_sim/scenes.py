"""Scene builders (default_room, variants) so each teammate can render distinct media.

Coordinates are metres, world frame, origin at the room's lower-left corner.
"""

import numpy as np

from . import config
from .materials import glass, matte, mirror
from .world import MovingBox, Segment, World, box_segments, ping_pong


def _wall(x0, y0, x1, y1, rho=config.RHO_WALL, name="wall"):
    return Segment((x0, y0), (x1, y1), matte(rho, name), name)


def _line_segment(center, length, angle_deg, material, name):
    d = np.array([np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))])
    c = np.asarray(center, dtype=float)
    return Segment(c - 0.5 * length * d, c + 0.5 * length * d, material, name)


def default_room(
    with_mirror=True,
    with_glass=True,
    with_behind_glass=True,
    with_moving_box=True,
    mirror_center=(6.6, 4.2),
    mirror_angle_deg=45.0,
    mirror_length=1.6,
    box_rho=0.5,
    box_path=((1.2, 4.6), (5.2, 4.6)),
    box_speed=0.5,
    fog_alpha=0.0,
):
    """8 m x 6 m room (Ammar's scene). See plan/01_PROJECT_PLAN.md section 5.

    - matte walls (rho 0.7) with a dark section (rho 0.1, bottom) and a bright one (rho 0.9, top)
    - mirror on the right at 45 deg, so reflections hit the nearby top wall and ghosts appear once
      the sensor is within ~3 m (rotatable via mirror_angle_deg = direction of the mirror line)
    - vertical glass pane at x = 4.2 with a box behind it
    - a box moving back and forth along y = 4.6
    """
    segs = [
        _wall(0, 0, 3, 0),
        _wall(3, 0, 5, 0, rho=0.1, name="dark wall"),
        _wall(5, 0, 8, 0),
        _wall(8, 0, 8, 6),
        _wall(8, 6, 6, 6),
        _wall(6, 6, 4, 6, rho=0.9, name="bright wall"),
        _wall(4, 6, 0, 6),
        _wall(0, 6, 0, 0),
    ]
    if with_mirror:
        segs.append(_line_segment(mirror_center, mirror_length, mirror_angle_deg, mirror(), "mirror"))
    if with_glass:
        segs.append(Segment((4.2, 0.4), (4.2, 2.6), glass(), "glass"))
    if with_behind_glass:
        segs += box_segments((5.6, 1.2), (0.5, 0.5), 0.0, matte(0.6, "object"), "object")
    movers = []
    if with_moving_box:
        movers.append(MovingBox((0.5, 0.5), matte(box_rho, "box"), ping_pong(*box_path, box_speed), "box"))
    return World(segs, movers, fog_alpha, "default")


def variant_b(
    with_mirror=True,
    with_glass=True,
    with_behind_glass=True,
    with_moving_box=True,
    mirror_center=(1.4, 4.8),
    mirror_angle_deg=30.0,
    mirror_length=1.4,
    box_rho=0.5,
    box_path=((3.2, 1.0), (3.2, 5.0)),
    box_speed=0.5,
    fog_alpha=0.0,
):
    """9 m x 6 m room with a different layout (partner's scene, for distinct media)."""
    segs = [
        _wall(0, 0, 4, 0),
        _wall(4, 0, 6.5, 0, rho=0.9, name="bright wall"),
        _wall(6.5, 0, 9, 0),
        _wall(9, 0, 9, 2.5),
        _wall(9, 2.5, 9, 4.5, rho=0.1, name="dark wall"),
        _wall(9, 4.5, 9, 6),
        _wall(9, 6, 0, 6),
        _wall(0, 6, 0, 0),
    ]
    if with_mirror:
        segs.append(_line_segment(mirror_center, mirror_length, mirror_angle_deg, mirror(), "mirror"))
    if with_glass:
        segs.append(Segment((5.0, 1.0), (6.5, 3.2), glass(), "glass"))
    if with_behind_glass:
        segs += box_segments((7.6, 1.6), (0.6, 0.4), 0.3, matte(0.6, "object"), "object")
    movers = []
    if with_moving_box:
        movers.append(MovingBox((0.5, 0.5), matte(box_rho, "box"), ping_pong(*box_path, box_speed), "box"))
    return World(segs, movers, fog_alpha, "variant_b")


def glass_corridor(with_mirror=True, fog_alpha=0.0):
    """10 m x 6 m: a 2.5 m corridor (bottom) separated from a room by a full-length glass wall.

    Used for localization (E11). Inside the corridor the walls are parallel, so along-corridor
    (x) information mostly comes from objects seen *through* the glass (D3 insight + D13).
    """
    segs = [
        _wall(0, 0, 10, 0),
        _wall(10, 0, 10, 6),
        _wall(10, 6, 0, 6),
        _wall(0, 6, 0, 0),
        Segment((0, 2.5), (10, 2.5), glass(), "glass"),
    ]
    if with_mirror:
        segs.append(Segment((9.95, 0.4), (9.95, 2.1), mirror(), "mirror"))
    for c, size, ang in [((2.5, 4.0), (0.8, 0.6), 0.2), ((5.5, 3.6), (0.6, 0.6), 0.0), ((7.8, 4.4), (0.9, 0.5), -0.4)]:
        segs += box_segments(c, size, ang, matte(0.8, "object"), "object")
    return World(segs, [], fog_alpha, "glass_corridor")


def single_wall(distance=3.0, rho=0.9, angle_deg=0.0, length=40.0, fog_alpha=0.0):
    """One long matte wall in front of a sensor at the origin facing +x.

    angle_deg tilts the wall, i.e. sets the incidence angle of the beam along +x.
    """
    return World([_line_segment((distance, 0.0), length, 90.0 + angle_deg, matte(rho, "wall"), "wall")],
                 [], fog_alpha, "single_wall")


def glass_test(incidence_deg=0.0, glass_dist=2.0, wall_dist=4.0, glass_length=2.0, wall_rho=0.7):
    """Sensor at the origin facing +x, a glass pane at glass_dist rotated by incidence_deg,
    and a matte back wall at wall_dist (E6)."""
    pane = _line_segment((glass_dist, 0.0), glass_length, 90.0 + incidence_deg, glass(), "glass")
    back = _line_segment((wall_dist, 0.0), 40.0, 90.0, matte(wall_rho, "wall"), "wall")
    return World([pane, back], [], 0.0, "glass_test")


SCENES = {"default": default_room, "variant_b": variant_b, "glass_corridor": glass_corridor}


def get_scene(name="default", **kwargs):
    return SCENES[name](**kwargs)
