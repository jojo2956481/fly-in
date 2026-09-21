import sys
import argparse

from src.parsing_map import MapParser
from src.dijkstra_reservation_tab import Scheduler
from src.map_viewer import Viewer


class Application:
    """Point d'entrée : lit les arguments, parse la map, puis lance soit
    l'affichage graphique, soit une sortie texte des données."""

    def __init__(self) -> None:
        self.path_map = self._parse_args()

    @staticmethod
    def _parse_args() -> tuple[str, bool]:
        parser = argparse.ArgumentParser()
        parser.add_argument("--map", default=None)
        args = parser.parse_args()
        if args.map is None:
            print("Error no map selected")
            sys.exit(1)
        return args.map


    def run(self) -> None:
        drone_map = MapParser(self.path_map).parse()
        Viewer(drone_map).run()


def main() -> None:
    # try:
    #     Application().run()
    # except Exception as e:
    #     print(e)
    Application().run()


if __name__ == "__main__":
    main()
