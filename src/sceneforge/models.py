"""Versioned SceneForge input contracts.

These models deliberately describe intent and authority. They do not contain
Isaac Sim or Genie Sim runtime objects.
"""

from __future__ import annotations

from enum import Enum
import math
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


Identifier = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_-]{1,63}$")]
PositiveMeters = Annotated[float, Field(gt=0)]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class AssetMode(str, Enum):
    RESEARCH = "research"
    PRODUCTION = "production"


class SceneFamily(str, Enum):
    HOSPITAL = "hospital"
    HOTEL = "hotel"
    EXHIBITION = "exhibition"


class FootprintSpec(StrictModel):
    width_m: PositiveMeters
    depth_m: PositiveMeters


class ClearanceProfile(StrictModel):
    profile_id: Identifier
    body_width_m: PositiveMeters
    safety_margin_each_side_m: Annotated[float, Field(ge=0)]
    minimum_door_clear_width_m: PositiveMeters
    minimum_primary_corridor_width_m: PositiveMeters
    minimum_secondary_corridor_width_m: PositiveMeters
    minimum_turning_diameter_m: PositiveMeters
    minimum_head_clearance_m: PositiveMeters

    @property
    def required_clear_width_m(self) -> float:
        return self.body_width_m + 2 * self.safety_margin_each_side_m

    @model_validator(mode="after")
    def check_design_clearances(self) -> "ClearanceProfile":
        required = self.required_clear_width_m
        if self.minimum_door_clear_width_m < required and not math.isclose(
            self.minimum_door_clear_width_m, required, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("door clear width is smaller than the configured robot envelope")
        if self.minimum_secondary_corridor_width_m < required and not math.isclose(
            self.minimum_secondary_corridor_width_m, required, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("secondary corridor is smaller than the configured robot envelope")
        if self.minimum_primary_corridor_width_m < self.minimum_secondary_corridor_width_m:
            raise ValueError("primary corridor must not be narrower than secondary corridor")
        return self


class SpaceSpec(StrictModel):
    space_id: Identifier
    space_type: Identifier
    public: bool = True
    area_min_m2: Annotated[float, Field(gt=0)]
    area_max_m2: Annotated[float, Field(gt=0)]

    @model_validator(mode="after")
    def check_area_range(self) -> "SpaceSpec":
        if self.area_max_m2 < self.area_min_m2:
            raise ValueError("area_max_m2 must be greater than or equal to area_min_m2")
        return self


class AdjacencySpec(StrictModel):
    a: Identifier
    b: Identifier
    relation: Literal["required", "allowed", "forbidden"] = "required"

    @model_validator(mode="after")
    def check_distinct_spaces(self) -> "AdjacencySpec":
        if self.a == self.b:
            raise ValueError("adjacency endpoints must be distinct")
        return self


class BuildingSpec(StrictModel):
    schema_version: Literal["sceneforge/building-spec/v1"]
    building_id: Identifier
    seed: Annotated[int, Field(ge=0, le=2_147_483_647)]
    units: Literal["meters"] = "meters"
    up_axis: Literal["Z"] = "Z"
    floor_count: Literal[1] = 1
    footprint: FootprintSpec
    clearance_profile: ClearanceProfile
    spaces: Annotated[list[SpaceSpec], Field(min_length=1)]
    adjacency: list[AdjacencySpec] = Field(default_factory=list)

    @model_validator(mode="after")
    def check_graph_references(self) -> "BuildingSpec":
        ids = [space.space_id for space in self.spaces]
        if len(ids) != len(set(ids)):
            raise ValueError("space_id values must be unique")
        known = set(ids)
        for edge in self.adjacency:
            if edge.a not in known or edge.b not in known:
                raise ValueError(f"adjacency references an unknown space: {edge.a}, {edge.b}")
        return self


class AssetSlot(StrictModel):
    slot_id: Identifier
    semantic_type: Identifier
    space_id: Identifier
    minimum_count: Annotated[int, Field(ge=0)] = 0
    maximum_count: Annotated[int, Field(ge=0)] = 1
    placement: Literal["floor", "wall", "ceiling", "support_surface"] = "floor"

    @model_validator(mode="after")
    def check_count_range(self) -> "AssetSlot":
        if self.maximum_count < self.minimum_count:
            raise ValueError("maximum_count must be greater than or equal to minimum_count")
        return self


class PointOfInterest(StrictModel):
    poi_id: Identifier
    semantic_type: Identifier
    space_id: Identifier
    public: bool = True


class SceneSpec(StrictModel):
    schema_version: Literal["sceneforge/scene-spec/v1"]
    scene_id: Identifier
    building_id: Identifier
    family: SceneFamily
    seed: Annotated[int, Field(ge=0, le=2_147_483_647)]
    asset_mode: AssetMode
    asset_slots: list[AssetSlot] = Field(default_factory=list)
    points_of_interest: Annotated[list[PointOfInterest], Field(min_length=1)]
    patrol_route: Annotated[list[Identifier], Field(min_length=2)]
    real_robot_commands: Literal[False] = False

    @model_validator(mode="after")
    def check_poi_route(self) -> "SceneSpec":
        poi_ids = [poi.poi_id for poi in self.points_of_interest]
        if len(poi_ids) != len(set(poi_ids)):
            raise ValueError("poi_id values must be unique")
        known = set(poi_ids)
        missing = [poi for poi in self.patrol_route if poi not in known]
        if missing:
            raise ValueError(f"patrol_route references unknown POIs: {missing}")
        return self


class LicenseInfo(StrictModel):
    spdx_id: str
    commercial_use_allowed: bool
    redistribution_allowed: bool
    attribution_required: bool = False
    source_url: str


class AssetGeometry(StrictModel):
    width_m: PositiveMeters
    depth_m: PositiveMeters
    height_m: PositiveMeters
    up_axis: Literal["Z"] = "Z"
    units: Literal["meters"] = "meters"


class AssetPhysics(StrictModel):
    collision_status: Literal["missing", "generated", "authored", "qualified"]
    rigid_body: bool = False
    mass_kg: Annotated[float | None, Field(gt=0)] = None


class AssetRecord(StrictModel):
    schema_version: Literal["sceneforge/asset-record/v1"]
    asset_id: Identifier
    source: Identifier
    source_version: str
    uri: str
    sha256: Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
    license: LicenseInfo
    geometry: AssetGeometry
    physics: AssetPhysics
    semantic_tags: Annotated[list[Identifier], Field(min_length=1)]
    approval: Literal["unreviewed", "research_only", "production_approved"]


def assert_asset_allowed(asset: AssetRecord, mode: AssetMode) -> None:
    """Fail closed when an asset is not allowed in the selected build mode."""

    if mode is AssetMode.RESEARCH:
        if asset.approval == "unreviewed":
            raise ValueError(f"asset {asset.asset_id} has not been reviewed")
        return
    if asset.approval != "production_approved":
        raise ValueError(f"asset {asset.asset_id} is not production approved")
    if not asset.license.commercial_use_allowed:
        raise ValueError(f"asset {asset.asset_id} forbids commercial use")
