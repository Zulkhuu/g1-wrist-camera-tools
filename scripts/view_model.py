"""Launch the offline V0.1 model viewer."""
import argparse
import logging
from pathlib import Path
from g1_wrist_camera.config import DEFAULT_MOUNTS
from g1_wrist_camera.viewer.app import ViewerApp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--config", type=Path, default=DEFAULT_MOUNTS)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--smoke-test", action="store_true", help="Build viewer, exercise transforms, then exit")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        app = ViewerApp(args.side, args.config, args.host, args.port)
        if args.smoke_test:
            app.update()
            for s in app.selected:
                app.set_transform(s, "mount", app.config[s].mount)
            app.server.stop()
        else:
            app.run()
    except (ValueError, OSError) as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
