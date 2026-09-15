"""Generate a registered, single convex-hull STL for a rigid wrist mount."""
import argparse
from pathlib import Path

from g1_wrist_camera.model.collision import generate_collision_mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--force", action="store_true", help="Overwrite an existing collision STL")
    parser.add_argument("--method", choices=("convex-hull",), default="convex-hull")
    args = parser.parse_args()
    try:
        output, original, hull = generate_collision_mesh(
            args.input, args.output, force=args.force, method=args.method)
    except (ValueError, OSError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    for label, path, stats in (("Input mesh", args.input, original), ("Collision mesh", output, hull)):
        print(f"{label}:\n  path: {path}")
        for name, value in stats.items():
            print(f"  {name}: {value if value is not None else 'unavailable (not a valid closed, consistently wound solid)'}")
    print("Volumes are in input coordinate units cubed; no unit conversion applied.")
    ratio = hull['volume'] / original['volume'] if original['volume'] else None
    print(f"Volume ratio:\n  convex hull / original = {ratio if ratio is not None else 'unavailable'}")


if __name__ == "__main__":
    main()
