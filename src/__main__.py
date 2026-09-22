import sys
import argparse
from src.parsing_map import MapParser
from src.map_viewer import Viewer
from typing import Any


class Application:
    """class to launch the simulation and retrieving the arguments"""

    def __init__(self) -> None:
        self.path_map = self._parse_args()

    @staticmethod
    def _parse_args() -> str | Any:
        """method for retrieving the arguments"""
        parser = argparse.ArgumentParser()
        parser.add_argument("--map", default=None)
        args = parser.parse_args()
        if args.map is None:
            print("Error no map selected")
            sys.exit(1)
        else:
            return args.map

    def run(self) -> None:
        """Method for launching the simulation"""
        drone_map = MapParser(self.path_map).parse()
        Viewer(drone_map).run()


def main() -> None:
    """main function"""
    try:
        Application().run()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
