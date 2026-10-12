"""Deterministic, dependency-free SVG previews for SceneForge layouts."""

from __future__ import annotations

from html import escape
from pathlib import Path

from .layout import BuildingLayout
from .models import BuildingSpec


PALETTE = {
    "entrance": "#dbeafe",
    "reception": "#fef3c7",
    "waiting": "#dcfce7",
    "triage": "#fee2e2",
    "consultation": "#ede9fe",
    "service_window": "#ffedd5",
    "corridor": "#e2e8f0",
    "staff": "#f1f5f9",
}


def _fmt(value: float) -> str:
    return f"{value:.4f}".rstrip("0").rstrip(".") or "0"


def render_layout_svg(layout: BuildingLayout, building: BuildingSpec) -> str:
    width = building.footprint.width_m
    depth = building.footprint.depth_m
    offset_x = 1.0
    offset_y = 2.0
    view_width = width + 11.0
    view_height = depth + 3.0

    def sx(x: float) -> float:
        return offset_x + x

    def sy(y: float) -> float:
        return offset_y + depth - y

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" ',
        f'viewBox="0 0 {_fmt(view_width)} {_fmt(view_height)}" ',
        'role="img" aria-labelledby="title desc">\n',
        f'<title id="title">SceneForge {escape(layout.scene_id)} structure preview</title>\n',
        '<desc id="desc">Deterministic hospital floor plan with rooms, walls and door openings.</desc>\n',
        '<rect width="100%" height="100%" fill="#f8fafc"/>\n',
        f'<text x="1" y="1.1" font-family="sans-serif" font-size="0.65" font-weight="700" fill="#0f172a">SceneForge · {escape(layout.scene_id)}</text>\n',
        f'<text x="1" y="1.7" font-family="sans-serif" font-size="0.34" fill="#475569">seed {layout.seed_applied} · {layout.variant} · static structure preview · not X2 qualified</text>\n',
        f'<rect x="{_fmt(offset_x)}" y="{_fmt(offset_y)}" width="{_fmt(width)}" height="{_fmt(depth)}" fill="#ffffff" stroke="#0f172a" stroke-width="0.18"/>\n',
    ]
    for space in layout.spaces:
        rect = space.rect
        y = sy(rect.y_max_m)
        fill = PALETTE.get(space.space_type, "#f1f5f9")
        parts.append(
            f'<rect data-space="{escape(space.space_id)}" x="{_fmt(sx(rect.x_m))}" y="{_fmt(y)}" '
            f'width="{_fmt(rect.width_m)}" height="{_fmt(rect.depth_m)}" fill="{fill}" '
            'fill-opacity="0.86" stroke="#94a3b8" stroke-width="0.045"/>\n'
        )
        cx = sx(rect.x_m + rect.width_m / 2)
        cy = sy(rect.y_m + rect.depth_m / 2)
        parts.append(
            f'<text x="{_fmt(cx)}" y="{_fmt(cy - 0.1)}" text-anchor="middle" '
            'font-family="sans-serif" font-size="0.36" font-weight="600" fill="#0f172a">'
            f'{escape(space.space_id)}</text>\n'
        )
        parts.append(
            f'<text x="{_fmt(cx)}" y="{_fmt(cy + 0.34)}" text-anchor="middle" '
            'font-family="sans-serif" font-size="0.25" fill="#475569">'
            f'{_fmt(rect.area_m2)} m²</text>\n'
        )

    for wall in layout.walls:
        if wall.wall_axis == "x":
            x1 = x2 = sx(wall.center_x_m)
            y1 = sy(wall.center_y_m - wall.length_m / 2)
            y2 = sy(wall.center_y_m + wall.length_m / 2)
        else:
            x1 = sx(wall.center_x_m - wall.length_m / 2)
            x2 = sx(wall.center_x_m + wall.length_m / 2)
            y1 = y2 = sy(wall.center_y_m)
        parts.append(
            f'<line data-wall="{wall.wall_id}" x1="{_fmt(x1)}" y1="{_fmt(y1)}" '
            f'x2="{_fmt(x2)}" y2="{_fmt(y2)}" stroke="#111827" stroke-width="0.16" stroke-linecap="square"/>\n'
        )

    for door in layout.doors:
        half = door.clear_width_m / 2
        if door.wall_axis == "x":
            x1 = x2 = sx(door.center_x_m)
            y1 = sy(door.center_y_m - half)
            y2 = sy(door.center_y_m + half)
        else:
            x1 = sx(door.center_x_m - half)
            x2 = sx(door.center_x_m + half)
            y1 = y2 = sy(door.center_y_m)
        parts.append(
            f'<line data-door="{door.door_id}" x1="{_fmt(x1)}" y1="{_fmt(y1)}" '
            f'x2="{_fmt(x2)}" y2="{_fmt(y2)}" stroke="#06b6d4" stroke-width="0.09" stroke-dasharray="0.18 0.12"/>\n'
        )

    legend_x = width + 2.3
    parts.extend(
        [
            f'<text x="{_fmt(legend_x)}" y="3" font-family="sans-serif" font-size="0.45" font-weight="700" fill="#0f172a">空间类型</text>\n',
        ]
    )
    for index, (space_type, color) in enumerate(PALETTE.items()):
        y = 3.7 + index * 0.85
        parts.append(
            f'<rect x="{_fmt(legend_x)}" y="{_fmt(y - 0.38)}" width="0.55" height="0.55" rx="0.08" fill="{color}" stroke="#94a3b8" stroke-width="0.04"/>\n'
        )
        parts.append(
            f'<text x="{_fmt(legend_x + 0.75)}" y="{_fmt(y)}" font-family="sans-serif" font-size="0.31" fill="#334155">{escape(space_type)}</text>\n'
        )
    parts.append(
        f'<line x1="{_fmt(legend_x)}" y1="11.2" x2="{_fmt(legend_x + 1.2)}" y2="11.2" stroke="#06b6d4" stroke-width="0.09" stroke-dasharray="0.18 0.12"/>\n'
    )
    parts.append(
        f'<text x="{_fmt(legend_x + 1.45)}" y="11.3" font-family="sans-serif" font-size="0.31" fill="#334155">1.2 m 门洞</text>\n'
    )
    parts.append(
        f'<text x="{_fmt(legend_x)}" y="13" font-family="sans-serif" font-size="0.3" fill="#64748b">spaces {len(layout.spaces)}</text>\n'
    )
    parts.append(
        f'<text x="{_fmt(legend_x)}" y="13.5" font-family="sans-serif" font-size="0.3" fill="#64748b">doors {len(layout.doors)}</text>\n'
    )
    parts.append(
        f'<text x="{_fmt(legend_x)}" y="14" font-family="sans-serif" font-size="0.3" fill="#64748b">walls {len(layout.walls)}</text>\n'
    )
    parts.append("</svg>\n")
    return "".join(parts)


def write_layout_svg(output: Path, layout: BuildingLayout, building: BuildingSpec) -> None:
    if output.exists():
        raise FileExistsError(f"refusing to overwrite preview: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_layout_svg(layout, building), encoding="utf-8", newline="\n")

