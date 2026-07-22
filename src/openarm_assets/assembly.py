# openarm_assets/src/openarm_assets/assembly.py
"""Robot assembly utilities for building OpenArm configurations."""

import argparse
from pathlib import Path
from typing import Literal

import mujoco

from openarm_assets import MODELS_DIR

Side = Literal["left", "right"]

# Default source model and the decorated output this module produces.
_MODEL_DIR = MODELS_DIR / "openarm"
_DEFAULT_SRC = _MODEL_DIR / "vendor" / "openarm_bimanual.xml"
_GENERATED_DIR = _MODEL_DIR  / "generated"
_OUT_PATHS: dict[str, Path] = {
    "bimanual": _GENERATED_DIR / "openarm_bimanual_decorated.xml",
    "left": _GENERATED_DIR / "openarm_left_decorated.xml",
    "right": _GENERATED_DIR / "openarm_right_decorated.xml",
}

_SIDES = {
    "left": {
        "ee_site_name": "openarm_left_ee_site",
        "tcp_body": "openarm_left_hand_tcp",
        "base_body": "openarm_left_link0",
        "left_finger_body": "openarm_left_left_finger",
        "right_finger_body": "openarm_left_right_finger",
    },
    "right": {
        "ee_site_name": "openarm_right_ee_site",
        "tcp_body": "openarm_right_hand_tcp",
        "base_body": "openarm_right_link0",
        "left_finger_body": "openarm_right_left_finger",
        "right_finger_body": "openarm_right_right_finger",
    }
}


"""Scene decoration"""

def add_ee_site(spec: mujoco.MjSpec, tcp_body: str, site_name: str) -> None:
    """Add an end-effector site to the OpenArm left hand TCP."""
    tcp = spec.body(tcp_body)
    if tcp is None:
        raise RuntimeError(f"No body named {tcp_body!r} in this spec")
    site = tcp.add_site()
    site.name = site_name
    site.pos = [0.0, 0.0, 0.0]

def add_gravcomp(spec: mujoco.MjSpec, root_body: str) -> None:
    """Enable gravity compensation on the arm's kinematic subtree."""
    from mj_manipulator.arm import add_subtree_gravcomp

    add_subtree_gravcomp(spec, root_body)

def add_finger_exclude(spec: mujoco.MjSpec, left_finger: str, right_finger: str) -> None:
    """Exclude finger-finger self-collision at the closed position."""
    exclude = spec.add_exclude()
    exclude.bodyname1 = left_finger
    exclude.bodyname2 = right_finger

""" Build """

def build_openarm(
        src: Path | str = _DEFAULT_SRC, 
        sides: tuple[Side, ...] = ("left", "right"),
) -> mujoco.MjSpec:
    """Load the OpenArm scene and apply all decorations"""

    spec = mujoco.MjSpec.from_file(str(src))
    spec.compiler.meshdir = "../meshes"

    for side_name in sides:
        side = _SIDES[side_name]
        add_ee_site(spec, side["tcp_body"], side["ee_site_name"])
        add_gravcomp(spec, side["base_body"])
        add_finger_exclude(spec, side["left_finger_body"], side["right_finger_body"])

    return spec

def build_openarm_bimanual(src: Path | str = _DEFAULT_SRC) -> mujoco.MjSpec:
    """Decorate both arms."""
    return build_openarm(src, sides=("left", "right"))

def build_openarm_left(src: Path | str = _DEFAULT_SRC) -> mujoco.MjSpec:
    """Decorate only the left arm."""
    return build_openarm(src, sides=("left"))

def build_openarm_right(src: Path | str = _DEFAULT_SRC) -> mujoco.MjSpec:
    """Decorate only the right arm."""
    return build_openarm(src, sides=("right"))

""" CLI """

def main() -> None:
   parser = argparse.ArgumentParser(
       description="Decorate the OpenArm bimanual scene (EE site, gravcomp, "
       "finger exclude) for one or both arms and write the result to disk."
   )
   parser.add_argument(
       "-s", "--src", type=Path, default=_DEFAULT_SRC,
       help=f"Source scene XML (default: {_DEFAULT_SRC}).",
   )
   parser.add_argument(
       "--sides", choices=["bimanual", "left", "right"], default="bimanual",
       help="Which arm(s) to decorate (default: bimanual, both arms).",
   )
   parser.add_argument(
       "-o", "--out", type=Path, default=None,
       help="Output XML path (default: generated/openarm_<sides>_decorated.xml).",
   )
   args = parser.parse_args()
 
   sides = ("left", "right") if args.sides == "bimanual" else (args.sides,)
   spec = build_openarm(args.src, sides=sides)
 
   # Sanity check: does it actually compile?
   model = spec.compile()
   print(f"Compiled OK — {model.nq} qpos, {model.nu} actuators")
 
   out = args.out or _OUT_PATHS[args.sides]
   out.parent.mkdir(parents=True, exist_ok=True)
   out.write_text(spec.to_xml())
   print(f"Wrote {out}")
 
 
if __name__ == "__main__":
   main()
