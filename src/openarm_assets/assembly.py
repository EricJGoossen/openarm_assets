# openarm_assets/src/openarm_assets/assembly.py
"""Robot assembly utilities for building OpenArm configurations."""

import argparse
from pathlib import Path
import mujoco
from openarm_assets import MODELS_DIR

# Default source scene and the decorated output this module produces.
_DEFAULT_SRC = MODELS_DIR / "openarm" / "vendor" / "scene.xml"
_DEFAULT_OUT = MODELS_DIR / "openarm" / "openarm.xml"

_EE_SITE_NAME = "openarm_left_ee_site"
_TCP_BODY_NAME = "openarm_left_hand_tcp"
_BASE_BODY_NAME = "openarm_left_link0"
_LEFT_FINGER_BODY = "openarm_left_left_finger"
_RIGHT_FINGER_BODY = "openarm_left_right_finger"


"""Scene decoration"""

def add_ee_site(spec: mujoco.MjSpec, site_name: str = _EE_SITE_NAME) -> None:
    """Add an end-effector site to the OpenArm left hand TCP."""
    tcp = spec.body(_TCP_BODY_NAME)
    if tcp is None:
        raise RuntimeError(f"No body named {_TCP_BODY_NAME!r} in this spec")
    site = tcp.add_site()
    site.name = site_name
    site.pos = [0.0, 0.0, 0.0]

def add_gravcomp(spec: mujoco.MjSpec, root_body: str = _BASE_BODY_NAME) -> None:
    """Enable gravity compensation on the arm's kinematic subtree."""
    from mj_manipulator.arm import add_subtree_gravcomp

    add_subtree_gravcomp(spec, root_body)

def add_finger_exclude(spec: mujoco.MjSpec) -> None:
    """Exclude finger-finger self-collision at the closed position."""
    exclude = spec.add_exclude()
    exclude.bodyname1 = _LEFT_FINGER_BODY
    exclude.bodyname2 = _RIGHT_FINGER_BODY

""" Build """

def build_openarm(src: Path | str = _DEFAULT_SRC) -> mujoco.MjSpec:
    """Load the OpenArm scene and apply all decorations"""

    spec = mujoco.MjSpec.from_file(str(src))
    add_ee_site(spec)
    add_gravcomp(spec)
    add_finger_exclude(spec)
    return spec

""" CLI """

def main() -> None:
   parser = argparse.ArgumentParser(
       description="Decorate the OpenArm scene (EE site, gravcomp, finger "
       "exclude) and write the result to disk."
   )
   parser.add_argument(
       "-s", "--src", type=Path, default=_DEFAULT_SRC,
       help=f"Source scene XML (default: {_DEFAULT_SRC}).",
   )
   parser.add_argument(
       "-o", "--out", type=Path, default=_DEFAULT_OUT,
       help=f"Output scene XML (default: {_DEFAULT_OUT}).",
   )
   args = parser.parse_args()
 
   spec = build_openarm(args.src)
 
   # Sanity check: does it actually compile?
   model = spec.compile()
   print(f"Compiled OK — {model.nq} qpos, {model.nu} actuators")
 
   args.out.write_text(spec.to_xml())
   print(f"Wrote {args.out}")

if __name__ == "__main__":
    main()
