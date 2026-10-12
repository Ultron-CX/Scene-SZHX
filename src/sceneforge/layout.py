"""Deterministic single-floor hospital layout planning and validation."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .canonical import fingerprint
from .models import BuildingSpec, Identifier, SceneSpec, StrictModel


EPSILON = 1e-8


class Rect2D(StrictModel):
    x_m: Annotated[float, Field(ge=0)]
    y_m: Annotated[float, Field(ge=0)]
    width_m: Annotated[float, Field(gt=0)]
    depth_m: Annotated[float, Field(gt=0)]

    @property
    def x_max_m(self) -> float:
        return self.x_m + self.width_m

    @property
    def y_max_m(self) -> float:
        return self.y_m + self.depth_m

    @property
    def area_m2(self) -> float:
        return self.width_m * self.depth_m


class PlacedSpace(StrictModel):
    space_id: Identifier
    space_type: Identifier
    public: bool
    rect: Rect2D


class DoorOpening(StrictModel):
    door_id: Identifier
    space_a: Identifier
    space_b: Identifier | Literal["exterior"]
    wall_axis: Literal["x", "y"]
    center_x_m: float
    center_y_m: float
    clear_width_m: Annotated[float, Field(gt=0)]


class WallSegment(StrictModel):
    wall_id: Identifier
    wall_axis: Literal["x", "y"]
    center_x_m: float
    center_y_m: float
    length_m: Annotated[float, Field(gt=0)]
    thickness_m: Annotated[float, Field(gt=0)] = 0.15
    height_m: Annotated[float, Field(gt=0)] = 2.8


class BuildingLayout(StrictModel):
    schema_version: Literal["sceneforge/building-layout/v1"]
    building_id: Identifier
    scene_id: Identifier
    seed_applied: int
    variant: Literal["spine", "mirror_x", "mirror_y"]
    building_spec_sha256: str
    scene_spec_sha256: str
    spaces: list[PlacedSpace]
    doors: list[DoorOpening]
    walls: list[WallSegment]
    real_robot_commands: Literal[False] = False

    @model_validator(mode="after")
    def check_unique_ids(self) -> "BuildingLayout":
        for label, values in (
            ("space", [item.space_id for item in self.spaces]),
            ("door", [item.door_id for item in self.doors]),
            ("wall", [item.wall_id for item in self.walls]),
        ):
            if len(values) != len(set(values)):
                raise ValueError(f"{label} identifiers must be unique")
        return self


def _base_hospital_rectangles() -> dict[str, Rect2D]:
    return {
        "entrance": Rect2D(x_m=1.0, y_m=0.0, width_m=4.0, depth_m=5.8),
        "reception": Rect2D(x_m=1.0, y_m=5.8, width_m=5.3, depth_m=4.0),
        "waiting": Rect2D(x_m=6.3, y_m=1.0, width_m=10.0, depth_m=8.8),
        "triage": Rect2D(x_m=16.3, y_m=5.8, width_m=4.5, depth_m=4.0),
        "service_window": Rect2D(x_m=21.1, y_m=5.8, width_m=4.5, depth_m=4.0),
        "staff_zone": Rect2D(x_m=26.0, y_m=3.3, width_m=4.0, depth_m=6.5),
        "primary_corridor": Rect2D(x_m=1.0, y_m=9.8, width_m=30.0, depth_m=2.4),
        "consult_01": Rect2D(x_m=1.0, y_m=12.2, width_m=6.0, depth_m=3.0),
        "consult_02": Rect2D(x_m=7.5, y_m=12.2, width_m=6.0, depth_m=3.0),
        "consult_03": Rect2D(x_m=14.0, y_m=12.2, width_m=6.0, depth_m=3.0),
        "consult_04": Rect2D(x_m=20.5, y_m=12.2, width_m=6.0, depth_m=3.0),
    }


def _transform(rect: Rect2D, width: float, depth: float, variant: str) -> Rect2D:
    if variant == "mirror_x":
        return Rect2D(
            x_m=round(width - rect.x_max_m, 6),
            y_m=rect.y_m,
            width_m=rect.width_m,
            depth_m=rect.depth_m,
        )
    if variant == "mirror_y":
        return Rect2D(
            x_m=rect.x_m,
            y_m=round(depth - rect.y_max_m, 6),
            width_m=rect.width_m,
            depth_m=rect.depth_m,
        )
    return rect


def _overlap_length(a0: float, a1: float, b0: float, b1: float) -> float:
    return max(0.0, min(a1, b1) - max(a0, b0))


def _shared_edge(a: Rect2D, b: Rect2D) -> tuple[str, float, float, float] | None:
    y_overlap = _overlap_length(a.y_m, a.y_max_m, b.y_m, b.y_max_m)
    if y_overlap > EPSILON and (
        abs(a.x_max_m - b.x_m) <= EPSILON or abs(b.x_max_m - a.x_m) <= EPSILON
    ):
        coordinate = a.x_max_m if abs(a.x_max_m - b.x_m) <= EPSILON else a.x_m
        start = max(a.y_m, b.y_m)
        return "x", coordinate, start, start + y_overlap
    x_overlap = _overlap_length(a.x_m, a.x_max_m, b.x_m, b.x_max_m)
    if x_overlap > EPSILON and (
        abs(a.y_max_m - b.y_m) <= EPSILON or abs(b.y_max_m - a.y_m) <= EPSILON
    ):
        coordinate = a.y_max_m if abs(a.y_max_m - b.y_m) <= EPSILON else a.y_m
        start = max(a.x_m, b.x_m)
        return "y", coordinate, start, start + x_overlap
    return None


def _door_for_edge(
    index: int, a_id: str, b_id: str, edge: tuple[str, float, float, float], width: float
) -> DoorOpening:
    axis, coordinate, start, end = edge
    if end - start + EPSILON < width:
        raise ValueError(f"shared boundary between {a_id} and {b_id} is too short for a door")
    center = (start + end) / 2
    return DoorOpening(
        door_id=f"door_{index:03d}",
        space_a=a_id,
        space_b=b_id,
        wall_axis=axis,
        center_x_m=coordinate if axis == "x" else center,
        center_y_m=center if axis == "x" else coordinate,
        clear_width_m=width,
    )


def _rectangles_overlap(a: Rect2D, b: Rect2D) -> bool:
    return (
        _overlap_length(a.x_m, a.x_max_m, b.x_m, b.x_max_m) > EPSILON
        and _overlap_length(a.y_m, a.y_max_m, b.y_m, b.y_max_m) > EPSILON
    )


def _merge_intervals(intervals: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    merged: list[list[float]] = []
    for start, end in sorted(intervals):
        if not merged or start > merged[-1][1] + EPSILON:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    return [(start, end) for start, end in merged]


def _subtract_openings(
    interval: tuple[float, float], openings: Iterable[tuple[float, float]]
) -> list[tuple[float, float]]:
    pieces = [interval]
    for opening_start, opening_end in _merge_intervals(openings):
        next_pieces: list[tuple[float, float]] = []
        for start, end in pieces:
            if opening_end <= start + EPSILON or opening_start >= end - EPSILON:
                next_pieces.append((start, end))
                continue
            if opening_start > start + EPSILON:
                next_pieces.append((start, min(end, opening_start)))
            if opening_end < end - EPSILON:
                next_pieces.append((max(start, opening_end), end))
        pieces = next_pieces
    return [(start, end) for start, end in pieces if end - start > EPSILON]


def _build_walls(
    spaces: list[PlacedSpace], doors: list[DoorOpening], footprint_width: float, footprint_depth: float
) -> list[WallSegment]:
    lines: dict[tuple[str, float], list[tuple[float, float]]] = {}

    def add(axis: str, coordinate: float, start: float, end: float) -> None:
        lines.setdefault((axis, round(coordinate, 6)), []).append((start, end))

    add("x", 0.0, 0.0, footprint_depth)
    add("x", footprint_width, 0.0, footprint_depth)
    add("y", 0.0, 0.0, footprint_width)
    add("y", footprint_depth, 0.0, footprint_width)
    for space in spaces:
        rect = space.rect
        add("x", rect.x_m, rect.y_m, rect.y_max_m)
        add("x", rect.x_max_m, rect.y_m, rect.y_max_m)
        add("y", rect.y_m, rect.x_m, rect.x_max_m)
        add("y", rect.y_max_m, rect.x_m, rect.x_max_m)

    door_intervals: dict[tuple[str, float], list[tuple[float, float]]] = {}
    for door in doors:
        coordinate = door.center_x_m if door.wall_axis == "x" else door.center_y_m
        center = door.center_y_m if door.wall_axis == "x" else door.center_x_m
        half = door.clear_width_m / 2
        door_intervals.setdefault((door.wall_axis, round(coordinate, 6)), []).append(
            (center - half, center + half)
        )

    walls: list[WallSegment] = []
    for axis_coordinate in sorted(lines):
        axis, coordinate = axis_coordinate
        for interval in _merge_intervals(lines[axis_coordinate]):
            for start, end in _subtract_openings(interval, door_intervals.get(axis_coordinate, [])):
                center = (start + end) / 2
                walls.append(
                    WallSegment(
                        wall_id=f"wall_{len(walls):03d}",
                        wall_axis=axis,
                        center_x_m=coordinate if axis == "x" else center,
                        center_y_m=center if axis == "x" else coordinate,
                        length_m=end - start,
                    )
                )
    return walls


def plan_hospital(building: BuildingSpec, scene: SceneSpec) -> BuildingLayout:
    if scene.family.value != "hospital":
        raise ValueError("hospital planner requires a hospital SceneSpec")
    if scene.building_id != building.building_id:
        raise ValueError("SceneSpec building_id does not match BuildingSpec")
    if scene.seed != building.seed:
        raise ValueError("BuildingSpec and SceneSpec seeds must match")
    if building.footprint.width_m < 32 or building.footprint.depth_m < 22:
        raise ValueError("hospital outpatient v1 requires a footprint of at least 32m x 22m")

    base = _base_hospital_rectangles()
    space_specs = {space.space_id: space for space in building.spaces}
    if set(space_specs) != set(base):
        missing = sorted(set(base) - set(space_specs))
        extra = sorted(set(space_specs) - set(base))
        raise ValueError(f"hospital outpatient v1 space mismatch; missing={missing}, extra={extra}")

    variant = ("spine", "mirror_x", "mirror_y")[building.seed % 3]
    placed = [
        PlacedSpace(
            space_id=space.space_id,
            space_type=space.space_type,
            public=space.public,
            rect=_transform(
                base[space.space_id],
                building.footprint.width_m,
                building.footprint.depth_m,
                variant,
            ),
        )
        for space in building.spaces
    ]
    rects = {space.space_id: space.rect for space in placed}

    for placed_space in placed:
        rect = placed_space.rect
        source = space_specs[placed_space.space_id]
        if rect.x_max_m > building.footprint.width_m + EPSILON or rect.y_max_m > building.footprint.depth_m + EPSILON:
            raise ValueError(f"space {placed_space.space_id} exceeds the footprint")
        if not source.area_min_m2 - EPSILON <= rect.area_m2 <= source.area_max_m2 + EPSILON:
            raise ValueError(f"space {placed_space.space_id} violates its area range")
    for index, first in enumerate(placed):
        for second in placed[index + 1 :]:
            if _rectangles_overlap(first.rect, second.rect):
                raise ValueError(f"spaces overlap: {first.space_id}, {second.space_id}")

    door_width = building.clearance_profile.minimum_door_clear_width_m
    doors: list[DoorOpening] = []
    for adjacency in building.adjacency:
        edge = _shared_edge(rects[adjacency.a], rects[adjacency.b])
        if adjacency.relation == "required":
            if edge is None:
                raise ValueError(f"required spaces do not share a boundary: {adjacency.a}, {adjacency.b}")
            doors.append(_door_for_edge(len(doors), adjacency.a, adjacency.b, edge, door_width))
        elif adjacency.relation == "forbidden" and edge is not None:
            raise ValueError(f"forbidden spaces share a boundary: {adjacency.a}, {adjacency.b}")

    entrance = rects["entrance"]
    if abs(entrance.y_m) <= EPSILON:
        exterior_edge = ("y", 0.0, entrance.x_m, entrance.x_max_m)
    elif abs(entrance.y_max_m - building.footprint.depth_m) <= EPSILON:
        exterior_edge = ("y", building.footprint.depth_m, entrance.x_m, entrance.x_max_m)
    else:
        raise ValueError("entrance must touch an exterior footprint edge")
    doors.append(_door_for_edge(len(doors), "entrance", "exterior", exterior_edge, door_width))

    return BuildingLayout(
        schema_version="sceneforge/building-layout/v1",
        building_id=building.building_id,
        scene_id=scene.scene_id,
        seed_applied=building.seed,
        variant=variant,
        building_spec_sha256=fingerprint(building.model_dump(mode="json")),
        scene_spec_sha256=fingerprint(scene.model_dump(mode="json")),
        spaces=placed,
        doors=doors,
        walls=_build_walls(placed, doors, building.footprint.width_m, building.footprint.depth_m),
        real_robot_commands=False,
    )

