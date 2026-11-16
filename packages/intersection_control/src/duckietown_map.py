"""
Duckietown city map + turn lookup table.

The map is described at TILE level on the 8x8 whiteboard grid (rows A-H top to
bottom, columns 1-8 left to right). Each road is the exact list of tiles it
passes through, from one intersection tile to another. Everything else is
derived from that:

  * which side (N/E/S/W) of each intersection a road attaches to
  * the type of every tile (straight, curve, 3-way, 4-way, empty)
  * road lengths in meters (straights = 1 tile, curves = quarter circle)
  * the turn lookup table: (current, previous, turn) -> (next intersection, road length)
  * an ASCII map, a JSON export, and matplotlib drawings of the city and routes

Planning a route is up to you. Everything you need is importable:

    from duckietown_map import LOOKUP, TURNS, legal_turns

    LOOKUP[(1, 3, "straight")]   # -> (2, 2.799): at 1, came from 3, go straight,
                                 #    you reach 2 after 2.799 m of road
    legal_turns(1, 3)            # -> {"straight": (2, 2.799), "right": (4, 1.755)}
    TURNS                        # -> {"left": ..., "straight": ..., "right": ...}

A state is (current, previous): where you are and where you came from. That
is enough to know which way you're facing, because no two roads join the same
pair of intersections (City raises if you add one).

Coordinate frame (meters): origin at the bottom-left (SW) corner of tile H1,
x points east (toward column 8), y points north (toward row A).
"""

import json
import math

# ---------------------------------------------------------------------------
# Grid settings
# ---------------------------------------------------------------------------
TILE_SIZE = 0.585          # meters per tile (standard Duckietown tile); change if yours differ
ROWS = "ABCDEFGH"          # top (north) to bottom (south)
N_COLS = 8                 # columns 1..8, left (west) to right (east)

# ---------------------------------------------------------------------------
# The city, traced from the whiteboard
# ---------------------------------------------------------------------------
INTERSECTIONS = {
    1: "A3",
    2: "C6",
    3: "D1",
    4: "D3",
    5: "F6",
    6: "H6",
}

# Each road: every tile it covers, in order, starting and ending on an intersection.
ROADS = [
    ["A3", "A2", "A1", "B1", "C1", "D1"],                        # 1 -> 3: row A west, down University Pkwy
    ["A3", "A4", "A5", "A6", "B6", "C6"],                        # 1 -> 2: row A east, down column 6
    ["A3", "B3", "C3", "D3"],                                    # 1 -> 4: straight south
    ["D1", "D2", "D3"],                                          # 3 -> 4: N 450 E (upper)
    ["D1", "E1", "F1", "G1", "G2", "H2", "H3", "H4", "H5", "H6"],# 3 -> 6: Pkwy south, 3-turn wiggle around H1, University Ave
    ["D3", "E3", "F3", "F4", "F5", "F6"],                        # 4 -> 5: south, curve east onto N 450 E (lower)
    ["C6", "D6", "E6", "F6"],                                    # 2 -> 5: column 6
    ["F6", "G6", "H6"],                                          # 5 -> 6: column 6
    ["C6", "C7", "C8", "D8", "E8", "F8", "G8", "H8", "H7", "H6"],# 2 -> 6: big east loop
]

# ---------------------------------------------------------------------------
# Direction helpers
# ---------------------------------------------------------------------------
DIRS = ["N", "E", "S", "W"]                                       # clockwise
GRID_STEP = {"N": (-1, 0), "E": (0, 1), "S": (1, 0), "W": (0, -1)}  # (d_row, d_col)
WORLD_VEC = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}  # (dx, dy) in meters frame
TURNS = {"left": -1, "straight": 0, "right": 1}


def opposite(d):
    return DIRS[(DIRS.index(d) + 2) % 4]


def parse_tile(tile):
    """'A3' -> (row_index, col_index) = (0, 2)"""
    r, c = ROWS.index(tile[0].upper()), int(tile[1:]) - 1
    if not (0 <= r < len(ROWS) and 0 <= c < N_COLS):
        raise ValueError(f"Tile {tile} is outside the grid")
    return r, c


def tile_name(r, c):
    return f"{ROWS[r]}{c + 1}"


def tile_center(tile):
    """Tile center in meters (x east, y north, origin at SW corner of the grid)."""
    r, c = parse_tile(tile)
    return ((c + 0.5) * TILE_SIZE, (len(ROWS) - r - 0.5) * TILE_SIZE)


def side_toward(a, b):
    """Which side of tile a is tile b on?"""
    (r1, c1), (r2, c2) = parse_tile(a), parse_tile(b)
    for d, step in GRID_STEP.items():
        if (r2 - r1, c2 - c1) == step:
            return d
    raise ValueError(f"Tiles {a} and {b} are not adjacent")


def classify(sides):
    """Tile type from the set of sides that have road."""
    n = len(sides)
    if n == 0:
        return "empty"
    if n == 2:
        a, b = sorted(sides, key=DIRS.index)
        return "straight" if opposite(a) == b else "curve"
    return {1: "dead_end", 3: "3way", 4: "4way"}[n]


# ---------------------------------------------------------------------------
# City model
# ---------------------------------------------------------------------------
class City:
    def __init__(self, intersections, roads):
        self.intersections = dict(intersections)
        self.node_at = {t: n for n, t in self.intersections.items()}
        self.tiles = {}            # tile -> set of sides with road
        self.roads = []            # list of road dicts
        self.exits = {n: {} for n in self.intersections}   # exits[node][side] = road info
        used = set()

        for rid, path in enumerate(roads):
            if len(path) < 2:
                raise ValueError(f"Road {rid} needs at least two tiles")
            a, b = self.node_at.get(path[0]), self.node_at.get(path[-1])
            if a is None or b is None:
                raise ValueError(f"Road {rid} must start and end on intersection tiles: {path}")

            for i, tile in enumerate(path):
                parse_tile(tile)
                interior = 0 < i < len(path) - 1
                if interior:
                    if tile in self.node_at:
                        raise ValueError(f"Road {rid} passes through intersection {tile}; split it there")
                    if tile in used:
                        raise ValueError(f"Tile {tile} is used by two roads")
                    used.add(tile)
                sides = self.tiles.setdefault(tile, set())
                if i > 0:
                    sides.add(side_toward(tile, path[i - 1]))
                if i < len(path) - 1:
                    sides.add(side_toward(tile, path[i + 1]))

            # length along the lane centerline: half a tile in each intersection,
            # full tile per straight, quarter circle (radius = half tile) per curve
            length_tiles, curves = 1.0, 0
            for i in range(1, len(path) - 1):
                s_in, s_out = side_toward(path[i], path[i - 1]), side_toward(path[i], path[i + 1])
                if opposite(s_in) == s_out:
                    length_tiles += 1.0
                else:
                    length_tiles += math.pi / 4
                    curves += 1

            side_a, side_b = side_toward(path[0], path[1]), side_toward(path[-1], path[-2])
            for node, side in ((a, side_a), (b, side_b)):
                if side in self.exits[node]:
                    raise ValueError(f"Intersection {node} has two roads on its {side} side")
            road = {
                "id": rid, "a": a, "b": b, "side_a": side_a, "side_b": side_b,
                "tiles": list(path), "length_m": round(length_tiles * TILE_SIZE, 3),
                "length_tiles": round(length_tiles, 3), "curves": curves,
            }
            self.roads.append(road)
            # store both driving directions
            self.exits[a][side_a] = {"road": rid, "to": b, "arrive_side": side_b,
                                     "tiles": list(path), "length_m": road["length_m"]}
            self.exits[b][side_b] = {"road": rid, "to": a, "arrive_side": side_a,
                                     "tiles": list(reversed(path)), "length_m": road["length_m"]}

    # --- driving -----------------------------------------------------------
    def step(self, node, entry_side, turn):
        """Entered `node` from `entry_side`, make `turn`. Returns the exit info or None."""
        heading = opposite(entry_side)
        exit_side = DIRS[(DIRS.index(heading) + TURNS[turn]) % 4]
        return self.exits[node].get(exit_side)

    def entry_side(self, node, came_from):
        sides = [s for s, e in self.exits[node].items() if e["to"] == came_from]
        if not sides:
            raise ValueError(f"No road from {came_from} into {node}")
        if len(sides) > 1:
            raise ValueError(f"{len(sides)} roads join {came_from} and {node}; give the entry side instead")
        return sides[0]

    def road_between(self, a, b):
        for road in self.roads:
            if {road["a"], road["b"]} == {a, b}:
                return road
        raise ValueError(f"No road between {a} and {b}")

    def neighbors(self, node):
        """Intersections one road away from `node`."""
        return sorted({e["to"] for e in self.exits[node].values()})

    def lookup_table(self):
        """(current, previous, turn) -> (next_intersection, road_length_m)"""
        table = {}
        for node, sides in self.exits.items():
            for entry, info in sides.items():
                for turn in TURNS:
                    out = self.step(node, entry, turn)
                    if out:
                        table[(node, info["to"], turn)] = (out["to"], out["length_m"])
        return table

    # --- map output -----------------------------------------------------------
    def tile_grid(self):
        grid = []
        for r in range(len(ROWS)):
            row = []
            for c in range(N_COLS):
                name = tile_name(r, c)
                sides = self.tiles.get(name, set())
                row.append({"tile": name, "type": classify(sides),
                            "sides": "".join(sorted(sides, key=DIRS.index)),
                            "intersection": self.node_at.get(name)})
            grid.append(row)
        return grid

    def ascii_map(self):
        glyph = {"EW": "──", "NS": "│ ", "ES": "┌─", "SW": "┐ ", "NE": "└─", "NW": "┘ "}
        lines = ["   " + "".join(f"{c + 1:<2}" for c in range(N_COLS))]
        for r, row in enumerate(self.tile_grid()):
            text = ""
            for cell in row:
                if cell["intersection"]:
                    n = str(cell["intersection"])
                    text += n + ("─" if "E" in cell["sides"] else " ")
                else:
                    text += glyph.get(cell["sides"], ". ")
            lines.append(f"{ROWS[r]}  {text}")
        return "\n".join(lines)

    def to_dict(self):
        return {
            "tile_size_m": TILE_SIZE,
            "frame": "origin at SW corner of H1, x east, y north (meters)",
            "grid": {"rows": ROWS, "cols": N_COLS},
            "intersections": {
                str(n): {"tile": t, "center_m": [round(v, 4) for v in tile_center(t)],
                         "sides": {s: e["to"] for s, e in sorted(self.exits[n].items(),
                                                                  key=lambda kv: DIRS.index(kv[0]))}}
                for n, t in self.intersections.items()},
            "roads": self.roads,
            "tiles": self.tile_grid(),
        }


    # --- drawing -------------------------------------------------------------
    # matplotlib is imported inside these so the robot can use the map without it,
    # and nothing here picks a backend: the PNG export renders with Agg directly
    # and the GUI embeds its own Tk canvas.
    def _oriented_tiles(self, a, b):
        road = self.road_between(a, b)
        return road["tiles"] if road["a"] == a else road["tiles"][::-1]

    def _centerline_point(self, path, i):
        """Point on the lane centerline halfway through tile path[i] (meters)."""
        cx, cy = tile_center(path[i])
        if i == 0 or i == len(path) - 1:
            return cx, cy
        v1 = WORLD_VEC[side_toward(path[i], path[i - 1])]
        v2 = WORLD_VEC[side_toward(path[i], path[i + 1])]
        # Straights: v1 + v2 = 0, the tile center. Curves: midpoint of the quarter
        # circle of radius half a tile around the corner between the two sides.
        k = TILE_SIZE / 2 * (1 - 1 / math.sqrt(2))
        return cx + k * (v1[0] + v2[0]), cy + k * (v1[1] + v2[1])

    def _draw_path(self, ax, path, color, lw, z):
        from matplotlib.patches import Arc

        half, artists = TILE_SIZE / 2, []

        def segment(x0, y0, x1, y1):
            artists.extend(ax.plot([x0, x1], [y0, y1], color=color, lw=lw, zorder=z,
                                   solid_capstyle="butt"))

        for i in range(1, len(path) - 1):
            s1, s2 = side_toward(path[i], path[i - 1]), side_toward(path[i], path[i + 1])
            cx, cy = tile_center(path[i])
            v1, v2 = WORLD_VEC[s1], WORLD_VEC[s2]
            if opposite(s1) == s2:
                segment(cx + half * v1[0], cy + half * v1[1], cx + half * v2[0], cy + half * v2[1])
                continue
            corner = (cx + half * (v1[0] + v2[0]), cy + half * (v1[1] + v2[1]))
            a1 = math.degrees(math.atan2(-v2[1], -v2[0]))
            a2 = math.degrees(math.atan2(-v1[1], -v1[0]))
            if (a2 - a1) % 360 != 90:
                a1, a2 = a2, a1
            artists.append(ax.add_patch(Arc(corner, TILE_SIZE, TILE_SIZE, theta1=a1, theta2=a2,
                                            color=color, lw=lw, zorder=z)))
        for end, nb in ((path[0], path[1]), (path[-1], path[-2])):
            cx, cy = tile_center(end)
            v = WORLD_VEC[side_toward(end, nb)]
            segment(cx, cy, cx + half * v[0], cy + half * v[1])
        return artists

    def draw_base(self, ax):
        """Grid, roads with their lengths, and numbered intersections."""
        from matplotlib.patches import Circle

        W, H = N_COLS * TILE_SIZE, len(ROWS) * TILE_SIZE
        for i in range(N_COLS + 1):
            ax.plot([i * TILE_SIZE] * 2, [0, H], color="#bfe3dc", lw=1, zorder=0)
        for i in range(len(ROWS) + 1):
            ax.plot([0, W], [i * TILE_SIZE] * 2, color="#bfe3dc", lw=1, zorder=0)
        for road in self.roads:
            self._draw_path(ax, road["tiles"], "#1f4fd1", 6, 2)
            mid = road["tiles"][len(road["tiles"]) // 2]
            mx, my = tile_center(mid)
            ax.text(mx, my + 0.09, f'{road["length_m"]:.2f} m', ha="center", fontsize=7,
                    color="#444", zorder=5,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
        for n, t in self.intersections.items():
            cx, cy = tile_center(t)
            ax.add_patch(Circle((cx, cy), TILE_SIZE * 0.28, color="#1f4fd1", zorder=4))
            ax.text(cx, cy, str(n), color="white", ha="center", va="center",
                    fontsize=13, fontweight="bold", zorder=6)
        ax.set_xticks([(c + 0.5) * TILE_SIZE for c in range(N_COLS)])
        ax.set_xticklabels([str(c + 1) for c in range(N_COLS)])
        ax.set_yticks([(len(ROWS) - r - 0.5) * TILE_SIZE for r in range(len(ROWS))])
        ax.set_yticklabels(list(ROWS))
        ax.xaxis.tick_top()
        ax.set_xlim(0, W)
        ax.set_ylim(0, H)
        ax.set_aspect("equal")

    def draw_leg(self, ax, a, b, color, lw=3, z=3, arrow=False):
        """The road from a to b, optionally with an arrow halfway along it pointing
        toward b. Returns the artists so a live plot can remove them."""
        path = self._oriented_tiles(a, b)
        artists = self._draw_path(ax, path, color, lw, z)
        if arrow and len(path) > 2:
            # On the straight tile nearest the middle (any tile if it's all curves),
            # pointing along the centerline: entering heading -v1, leaving along v2.
            interior = range(1, len(path) - 1)
            straight = [i for i in interior if opposite(side_toward(path[i], path[i - 1]))
                        == side_toward(path[i], path[i + 1])]
            m = min(straight or interior, key=lambda i: abs(i - (len(path) - 1) / 2))
            x0, y0 = self._centerline_point(path, m)
            v1 = WORLD_VEC[side_toward(path[m], path[m - 1])]
            v2 = WORLD_VEC[side_toward(path[m], path[m + 1])]
            tx, ty = v2[0] - v1[0], v2[1] - v1[1]
            d = math.hypot(tx, ty)
            dx, dy = 0.2 * TILE_SIZE * tx / d, 0.2 * TILE_SIZE * ty / d
            artists.append(ax.annotate("", xy=(x0 + dx, y0 + dy), xytext=(x0 - dx, y0 - dy),
                                       zorder=z + 4, arrowprops=dict(
                                           arrowstyle="-|>", color=color, lw=3,
                                           mutation_scale=22, shrinkA=0, shrinkB=0)))
        return artists

    def draw_goals(self, ax, goals, color="#27ae60", linestyle="-"):
        """A ring around each goal, labeled with its place(s) in the visit order."""
        from matplotlib.patches import Circle

        order = {}
        for i, g in enumerate(goals, 1):
            order.setdefault(g, []).append(str(i))
        artists = []
        for g, labels in order.items():
            cx, cy = tile_center(self.intersections[g])
            artists.append(ax.add_patch(Circle((cx, cy), TILE_SIZE * 0.40, fill=False, ec=color,
                                               lw=3, ls=linestyle, zorder=7)))
            artists.append(ax.text(cx + TILE_SIZE * 0.36, cy + TILE_SIZE * 0.36, ",".join(labels),
                                   color="white", fontsize=9, fontweight="bold", ha="center",
                                   va="center", zorder=8,
                                   bbox=dict(boxstyle="round,pad=0.2", fc=color, ec="none")))
        return artists

    def draw_route(self, ax, nodes, progress=None, goals=()):
        """Draw a route [came_from, start, ..., goal].

        progress is how many legs the robot has finished (None: just the plan).
        Finished legs are grey, the current one is thick with an arrow for the
        robot, the rest are orange. Returns the artists."""
        legs = list(zip(nodes, nodes[1:]))
        artists = []
        for k, (a, b) in enumerate(legs):
            if progress is None or k > progress:
                artists += self.draw_leg(ax, a, b, "#f39c12", lw=3)
            elif k < progress:
                artists += self.draw_leg(ax, a, b, "#8c8c8c", lw=3)
            else:
                artists += self.draw_leg(ax, a, b, "#d35400", lw=5, z=4, arrow=True)
        if progress is not None and legs and progress >= len(legs):
            from matplotlib.patches import Circle
            cx, cy = tile_center(self.intersections[nodes[-1]])
            artists.append(ax.add_patch(Circle((cx, cy), TILE_SIZE * 0.34, fill=False,
                                               ec="#d35400", lw=4, zorder=7)))
        return artists + self.draw_goals(ax, goals)

    def draw(self, filename, route_nodes=None, title=None):
        """Save the city, and optionally a route, as a PNG."""
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        fig = Figure(figsize=(8, 8))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot()
        self.draw_base(ax)
        if route_nodes:
            self.draw_route(ax, route_nodes, goals=route_nodes[-1:])
        ax.set_title(title or f"Duckietown city ({TILE_SIZE} m tiles)", fontsize=10, pad=24)
        fig.tight_layout()
        fig.savefig(filename, dpi=150)


# ---------------------------------------------------------------------------
# What you import
# ---------------------------------------------------------------------------
CITY = City(INTERSECTIONS, ROADS)

# (current, previous, turn) -> (next_intersection, road_length_m)
LOOKUP = CITY.lookup_table()

# Turns go over ROS as numbers: route_planner_node/turns is a list of these.
TURN_CODES = {"left": 0, "straight": 1, "right": 2}
TURN_NAMES = {code: turn for turn, code in TURN_CODES.items()}


def legal_turns(current, previous):
    """At `current`, having come from `previous`: {turn: (next, road_length_m)}
    for every turn the road allows."""
    return {turn: LOOKUP[(current, previous, turn)] for turn in TURNS
            if (current, previous, turn) in LOOKUP}


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Tile map\n")
    print(CITY.ascii_map())

    print("\nIntersections")
    for n, t in CITY.intersections.items():
        x, y = tile_center(t)
        sides = ", ".join(f"{s}->{e['to']}" for s, e in
                          sorted(CITY.exits[n].items(), key=lambda kv: DIRS.index(kv[0])))
        print(f"  {n} at {t}  ({x:.3f}, {y:.3f}) m   {sides}")

    print("\nRoads")
    for r in CITY.roads:
        print(f"  {r['a']}({r['side_a']}) <-> {r['b']}({r['side_b']})  "
              f"{r['length_m']:.3f} m  {r['length_tiles']:.2f} tiles  {r['curves']} curves")

    print("\nLookup table (current, previous, turn) -> next, road length")
    for (cur, prev, turn), (nxt, length) in sorted(LOOKUP.items()):
        print(f"  at {cur} from {prev}, {turn:8s} -> {nxt}  ({length:.2f} m)")

    with open("city_map.json", "w") as f:
        json.dump(CITY.to_dict(), f, indent=2)
    print("\nWrote city_map.json")

    try:
        CITY.draw("city_map.png")
        print("Wrote city_map.png")
    except ImportError:
        print("matplotlib not installed; skipped city_map.png")
