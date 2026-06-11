#!/usr/bin/env python3
"""
Generate visualizations for the IMRL fleet management README.

Produces two PNG files in docs/visualizations/:
  fig1_layout_graph.png   -- Full LIF graph, color-coded by node/edge type
  fig2_orientations.png   -- World-frame orientation (theta) reference

Run from anywhere:
    python fleet_management/generate_visualizations.py
"""

import json
import math
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parent
LIF_PATH = SCRIPT_DIR / "data" / "input_files" / "lif_file.json"
OUT_DIR = SCRIPT_DIR / "docs" / "visualizations"


# ---------------------------------------------------------------------------
# LIF helpers
# ---------------------------------------------------------------------------

def load_lif():
    with open(LIF_PATH) as f:
        lif = json.load(f)
    layout = lif["layouts"][0]
    nodes = {n["nodeId"]: n["nodePosition"] for n in layout["nodes"]}
    edges = layout["edges"]
    stations = layout["stations"]
    return nodes, edges, stations


def classify_node(node_id, stations):
    for s in stations:
        if node_id in s["interactionNodeIds"]:
            return s["stationDescription"]   # TRANSFER | PROCESS | CHARGING
    return "INTERMEDIATE"


def edge_allowed_types(edge):
    types = set()
    for p in edge["vehicleTypeEdgeProperties"]:
        if "Longitudinal" in p["vehicleTypeId"]:
            types.add("LongC")
        if "Lateral" in p["vehicleTypeId"]:
            types.add("LatC")
    return frozenset(types)


def edge_is_tangential(edge):
    return any(
        p.get("orientationType") == "TANGENTIAL"
        for p in edge["vehicleTypeEdgeProperties"]
    )


# ---------------------------------------------------------------------------
# Figure 1 — Full layout graph
# ---------------------------------------------------------------------------

NODE_STYLE = {
    "TRANSFER":    {"color": "#2ca02c", "size": 16, "label": "TRANSFER station node"},
    "PROCESS":     {"color": "#9467bd", "size": 16, "label": "PROCESS station node"},
    "CHARGING":    {"color": "#f5c842", "size": 16, "label": "CHARGING / dwelling node"},
    "INTERMEDIATE":{"color": "#aec7e8", "size": 14, "label": "Intermediate node"},
}

EDGE_STYLE = {
    frozenset(["LongC"]):          {"color": "#1f77b4", "lw": 2.5, "label": "LongC only"},
    frozenset(["LatC"]):           {"color": "#d62728", "lw": 2.5, "label": "LatC only"},
    frozenset(["LongC", "LatC"]): {"color": "#888888", "lw": 1.8, "label": "Both types"},
}


def plot_layout(nodes, edges, stations):
    fig, ax = plt.subplots(figsize=(13, 8))
    ax.set_aspect("equal")
    ax.set_facecolor("#f4f4f4")
    ax.grid(True, alpha=0.22, linestyle="--", color="gray")

    # ── Edges ──────────────────────────────────────────────────────────────
    for edge in edges:
        sx = nodes[edge["startNodeId"]]["x"]
        sy = nodes[edge["startNodeId"]]["y"]
        ex = nodes[edge["endNodeId"]]["x"]
        ey = nodes[edge["endNodeId"]]["y"]

        et = edge_allowed_types(edge)
        style = EDGE_STYLE.get(et, {"color": "#888888", "lw": 1.8})
        color = style["color"]
        lw = style["lw"]
        ls = "--" if edge_is_tangential(edge) else "-"

        ax.plot([sx, ex], [sy, ey], color=color, lw=lw, ls=ls,
                alpha=0.9, zorder=2, solid_capstyle="round")

        # Edge ID at midpoint
        mx, my = (sx + ex) / 2, (sy + ey) / 2
        ax.text(mx, my, edge["edgeId"], fontsize=5.5, color="#555",
                ha="center", va="center", zorder=4,
                bbox=dict(facecolor="white", edgecolor="none", pad=0.7, alpha=0.75))

    # ── Station center markers ──────────────────────────────────────────────
    for s in stations:
        sp = s.get("stationPosition")
        if sp is None:
            continue
        desc = s["stationDescription"]
        marker = {"TRANSFER": "s", "PROCESS": "D"}.get(desc, "o")
        nc = NODE_STYLE.get(desc, NODE_STYLE["INTERMEDIATE"])["color"]
        ax.plot(sp["x"], sp["y"], marker, markersize=22, color=nc,
                alpha=0.25, zorder=3, markeredgewidth=0)
        ax.text(sp["x"], sp["y"] - 0.28, s["stationId"], fontsize=8,
                ha="center", va="top", color=nc, fontweight="bold", zorder=4)

    # ── Nodes ───────────────────────────────────────────────────────────────
    for nid, pos in nodes.items():
        ntype = classify_node(nid, stations)
        st = NODE_STYLE[ntype]
        ax.plot(pos["x"], pos["y"], "o",
                markersize=st["size"],
                color=st["color"],
                markeredgecolor="white",
                markeredgewidth=1.8,
                zorder=5)
        ax.text(pos["x"], pos["y"] + 0.22, nid,
                fontsize=8.5, ha="center", va="bottom",
                fontweight="bold", zorder=6)

    # ── Legend ───────────────────────────────────────────────────────────────
    handles = []
    for et, st in EDGE_STYLE.items():
        handles.append(mlines.Line2D([], [], color=st["color"], lw=2,
                                     label=f"Edge — {st['label']}"))
    handles.append(mlines.Line2D([], [], color="gray", lw=2, ls="--",
                                  label="Dashed = TANGENTIAL approach edge"))
    handles.append(mlines.Line2D([], [], color="white", lw=0, label=""))  # spacer
    for ntype, st in NODE_STYLE.items():
        handles.append(mpatches.Patch(facecolor=st["color"], edgecolor="white",
                                       lw=1.5, label=f"Node — {st['label']}"))

    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0),
              borderaxespad=0, fontsize=8.5, framealpha=0.93,
              title="Legend", title_fontsize=9)

    ax.set_xlabel("x [m]", fontsize=12)
    ax.set_ylabel("y [m]", fontsize=12)
    ax.set_title("IMRL Test Area — Layout Graph (lif_file.json)",
        fontsize=12, fontweight="bold",
    )
    ax.set_xlim(-0.5, 9.3)
    ax.set_ylim(-0.7, 5.7)
    ax.tick_params(labelsize=10)

    fig.tight_layout()
    _save(fig, "fig1_layout_graph.png")


# ---------------------------------------------------------------------------
# Figure 2 — Orientation reference diagram
# ---------------------------------------------------------------------------

def plot_orientations():
    fig, ax = plt.subplots(figsize=(3.5, 3.5))
    ax.set_aspect("equal")
    ax.set_xlim(-2.3, 2.3)
    ax.set_ylim(-2.3, 2.3)
    ax.axis("off")
    ax.set_facecolor("#fafafa")
    ax.set_title("Vehicle Orientation (θ)", fontsize=10, fontweight="bold")

    configs = [
        (0.0,     "#1f77b4", "θ = 0.0",   "→ +x",  "left",   "center"),
        (1.5708,  "#2ca02c", "θ = 1.57",  "↑ +y",  "center", "bottom"),
        (3.1416,  "#d62728", "θ = 3.14",  "← −x",  "right",  "center"),
        (-1.5708, "#ff7f0e", "θ = −1.57", "↓ −y",  "center", "top"),
    ]

    for theta, color, val_lbl, dir_lbl, ha, va in configs:
        dx = math.cos(theta) * 1.0
        dy = math.sin(theta) * 1.0
        ax.annotate("", xy=(dx, dy), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color=color,
                                   lw=3, mutation_scale=20))
        lx = math.cos(theta) * 1.75
        ly = math.sin(theta) * 1.75
        ax.text(lx, ly, f"{val_lbl}\n{dir_lbl}", fontsize=9,
                color=color, ha=ha, va=va, fontweight="bold",
                bbox=dict(facecolor="white", edgecolor=color, alpha=0.92,
                          boxstyle="round,pad=0.3", lw=1.2))

    # CCW arc + label
    r = 0.48
    angs = np.linspace(0.12, math.pi / 2 - 0.12, 50)
    ax.plot(r * np.cos(angs), r * np.sin(angs), "k-", lw=2)
    tip = angs[-1]
    ax.annotate("",
                xy=(r * math.cos(tip), r * math.sin(tip)),
                xytext=(r * math.cos(tip - 0.18), r * math.sin(tip - 0.18)),
                arrowprops=dict(arrowstyle="-|>", color="black",
                               lw=1.8, mutation_scale=12))
    ax.text(0.2, 0.62, "+θ\n(CCW)", fontsize=8, ha="left", va="bottom",
            style="italic", color="black")
    ax.plot(0, 0, "o", markersize=8, color="#555555", zorder=5)
    ax.text(0, -0.18, "robot center", fontsize=7, ha="center", va="top", color="#555555")

    fig.tight_layout()
    _save(fig, "fig2_orientations.png")


# ---------------------------------------------------------------------------
# Figure 3 — Fine positioning coordinate system
# ---------------------------------------------------------------------------

def plot_fine_pos():
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(14, 6.5))
    fig.suptitle(
        "TRANSFER Station — Fine Positioning Coordinate System",
        fontsize=12, fontweight="bold",
    )

    # ── Left: axis definition ─────────────────────────────────────────────
    ax = ax_left
    ax.set_aspect("equal")
    ax.set_xlim(-1.8, 3.8)
    ax.set_ylim(-2.2, 2.8)
    ax.axis("off")
    ax.set_facecolor("#fafafa")
    ax.set_title("Axis Definition (right-hand coordinate system)", fontsize=11)

    # Thin gray reference lines through origin
    ax.plot([-1.5, 3.5], [0, 0], "-", color="#dddddd", lw=0.8, zorder=1)
    ax.plot([0, 0], [-1.8, 2.4], "-", color="#dddddd", lw=0.8, zorder=1)

    # Station box
    s_rect = mpatches.FancyBboxPatch((2.0, -0.55), 0.9, 1.1,
                                      boxstyle="round,pad=0.08",
                                      facecolor="#2ca02c", edgecolor="darkgreen",
                                      alpha=0.7, zorder=3)
    ax.add_patch(s_rect)
    ax.text(2.45, 0, "STATION\n(center)", fontsize=9, ha="center", va="center",
            fontweight="bold", color="white", zorder=4)

    # Dashed edge from node to station
    ax.plot([0, 2.0], [0, 0], "k:", lw=1.5, alpha=0.3, zorder=2)

    # Interaction node
    ax.plot(0, 0, "o", markersize=14, color="#1f77b4",
            markeredgecolor="white", markeredgewidth=2, zorder=5)
    ax.text(0, -0.28, "Interaction\nNode\n(origin)", fontsize=9,
            ha="center", va="top", color="#1f77b4", fontweight="bold")

    # +x axis (red, toward station)
    ax.annotate("", xy=(3.5, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#d62728",
                               lw=2.5, mutation_scale=18))
    ax.text(3.6, 0, "+x\n(toward station)", fontsize=9,
            ha="left", va="center", color="#d62728", fontweight="bold")

    # +y axis (green, CCW = left when facing station)
    ax.annotate("", xy=(0, 2.4), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#2ca02c",
                               lw=2.5, mutation_scale=18))
    ax.text(0.12, 2.42, "+y\n(left when facing station)",
            fontsize=9, ha="left", va="bottom", color="#2ca02c", fontweight="bold")

    # −y label (no arrow, just text)
    ax.text(-0.12, -1.8, "−y\n(right when facing station)",
            fontsize=8.5, ha="right", va="top", color="#888888")

    # CCW arc (theta convention)
    r_arc = 0.55
    angs = np.linspace(0.0, math.pi * 0.55, 40)
    ax.plot(r_arc * np.cos(angs), r_arc * np.sin(angs), "k-", lw=2)
    tip = angs[-1]
    ax.annotate("",
                xy=(r_arc * math.cos(tip), r_arc * math.sin(tip)),
                xytext=(r_arc * math.cos(tip - 0.2), r_arc * math.sin(tip - 0.2)),
                arrowprops=dict(arrowstyle="-|>", color="black",
                               lw=1.8, mutation_scale=12))
    ax.text(0.62, 0.72, "+θ (CCW)", fontsize=9, ha="left", va="bottom",
            color="black", style="italic")
    ax.text(0.62, -0.15, "θ = 0: nose → +x\n(toward station)",
            fontsize=8.5, ha="left", va="top", color="#d62728", style="italic")

    # ── Right: docking configurations ─────────────────────────────────────
    ax2 = ax_right
    ax2.set_aspect("equal")
    ax2.set_xlim(-1.8, 5.5)
    ax2.set_ylim(-2.4, 3.1)
    ax2.axis("off")
    ax2.set_facecolor("#fafafa")
    ax2.set_title("Docking Configurations", fontsize=11)

    station_x = 2.8

    # Column headers
    ax2.text(-1.72, 2.85, "Vehicle Type", fontsize=9, ha="left", va="center",
             fontweight="bold", color="#444444")
    ax2.text(1.3, 2.85, "Schematic", fontsize=9, ha="center", va="center",
             fontweight="bold", color="#444444")
    ax2.text(4.5, 2.85, "Parameters", fontsize=9, ha="center", va="center",
             fontweight="bold", color="#444444")
    # Header underline
    ax2.plot([-1.8, 5.5], [2.6, 2.6], "-", color="#555555", lw=1.0, zorder=1)
    # Row separators
    ax2.plot([-1.8, 5.5], [0.75, 0.75], ":", color="#aaaaaa", lw=0.9, zorder=1)
    ax2.plot([-1.8, 5.5], [-0.75, -0.75], ":", color="#aaaaaa", lw=0.9, zorder=1)
    # Bottom border
    ax2.plot([-1.8, 5.5], [-2.2, -2.2], "-", color="#555555", lw=0.8, zorder=1)

    configs_r = [
        # (y_center, robot_x, theta, color, v_label, param_label)
        (1.5,  0.20, 0.0,     "#1f77b4",
         "LongC",
         "x = 0.02 m,  y = 0,  θ = 0.0\n(frontal — nose toward station)"),
        (0.0,  0.65, 1.5708,  "#d62728",
         "LatC  (θ = +1.57)",
         "x = 0.08 m,  y = 0,  θ = +1.57\n(lateral — nose toward +y)"),
        (-1.5, 0.65, -1.5708, "#9467bd",
         "LatC  (θ = −1.57)",
         "x = 0.08 m,  y = 0,  θ = −1.57\n(lateral — nose toward −y)"),
    ]

    for y_c, rx, theta, color, v_label, param_label in configs_r:
        # Interaction node
        ax2.plot(0, y_c, "o", markersize=10, color="#1f77b4",
                 markeredgecolor="white", markeredgewidth=1.5, zorder=5)

        # Station box
        s2 = mpatches.FancyBboxPatch((station_x - 0.4, y_c - 0.38), 0.8, 0.76,
                                      boxstyle="round,pad=0.06",
                                      facecolor="#2ca02c", edgecolor="darkgreen",
                                      alpha=0.6, zorder=3)
        ax2.add_patch(s2)
        ax2.text(station_x, y_c, "STA-\nTION", fontsize=7,
                 ha="center", va="center", fontweight="bold", color="white", zorder=4)

        # Dashed edge node → station
        ax2.plot([0, station_x - 0.4], [y_c, y_c], "k:", lw=1.2, alpha=0.3, zorder=2)

        # Robot body
        rw, rh = 0.44, 0.22
        r_box = mpatches.FancyBboxPatch((rx - rw / 2, y_c - rh / 2), rw, rh,
                                         boxstyle="round,pad=0.04",
                                         facecolor=color, edgecolor="white",
                                         alpha=0.9, zorder=6)
        ax2.add_patch(r_box)

        # Heading arrow
        hdx = math.cos(theta) * 0.55
        hdy = math.sin(theta) * 0.55
        ax2.annotate("", xy=(rx + hdx, y_c + hdy), xytext=(rx, y_c),
                     arrowprops=dict(arrowstyle="-|>", color=color,
                                    lw=2.5, mutation_scale=16), zorder=7)

        # Vehicle label (left column)
        ax2.text(-1.72, y_c, v_label, fontsize=9, ha="left", va="center",
                 color=color, fontweight="bold")

        # Param label (right of station)
        ax2.text(station_x + 0.55, y_c, param_label, fontsize=8.5,
                 ha="left", va="center", color=color,
                 bbox=dict(facecolor="white", edgecolor=color,
                           boxstyle="round,pad=0.28", alpha=0.92, lw=1))

    fig.tight_layout()
    _save(fig, "fig3_fine_pos_coordinate_system.png")


# ---------------------------------------------------------------------------
# Util
# ---------------------------------------------------------------------------

def _save(fig, filename):
    path = OUT_DIR / filename
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    nodes, edges, stations = load_lif()

    print("Generating visualizations...")
    plot_layout(nodes, edges, stations)
    plot_orientations()
    plot_fine_pos()
    print(f"\nDone. Output: {OUT_DIR}")


if __name__ == "__main__":
    main()
