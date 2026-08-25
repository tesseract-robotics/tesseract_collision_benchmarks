#!/usr/bin/env python3
"""Plot benchmark CSV into grouped bar charts."""

import argparse
import math
import os

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import pandas as pd

MODE_ORDER = ["FIRST", "CLOSEST", "ALL"]

# Okabe-Ito, minus its yellow, which is too light to read as a fill on a white surface.
# Colours are pinned per manager rather than handed out by position, so a run filtered
# with --manager paints the survivors the same as a full run does.
MANAGER_COLORS = {
    "BulletCastBVHManager": "#0072B2",
    "BulletDiscreteBVHManager": "#56B4E9",
    "BulletDiscreteSimpleManager": "#009E73",
    "CoalCastBVHManager": "#CC79A7",
    "CoalDiscreteBVHManager": "#D55E00",
    "FCLDiscreteBVHManager": "#E69F00",
}


def build_color_map(managers):
    """Colour every manager in the run; unknown ones fall back to the matplotlib cycle."""
    palette = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    color_map = {}
    spare = 0
    for manager in managers:
        if manager in MANAGER_COLORS:
            color_map[manager] = MANAGER_COLORS[manager]
        else:
            color_map[manager] = palette[spare % len(palette)]
            spare += 1
    return color_map


def plot_scenario(df, scenario, output_dir, color_map):
    df_s = df[df["scenario"] == scenario].copy()

    if df_s.empty:
        return

    managers = sorted(df_s["manager"].unique())

    fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharey=False)
    fig.suptitle(f"Checks Per Second — {scenario}")

    for ax, mode in zip(axes, MODE_ORDER):
        df_mode = df_s[df_s["mode"] == mode]
        series = (
            df_mode.groupby("manager")["checks_per_second"]
            .mean()
            .reindex(managers)
        )

        series.plot(kind="bar", ax=ax, color=[color_map[m] for m in series.index])
        ax.set_title("")
        ax.set_xlabel(mode)
        if ax is axes[0]:
            ax.set_ylabel("Checks Per Second")
        else:
            ax.set_ylabel("")
        ax.set_xticks([])
        ax.grid(axis="y", linestyle=":", alpha=0.5)

    legend_handles = [Patch(color=color_map[m], label=m) for m in managers]
    fig.legend(
        handles=legend_handles,
        title="Manager",
        loc="lower center",
        bbox_to_anchor=(0.5, 0.02),
        ncol=len(managers),
        frameon=True,
    )

    scenario_prefix = scenario.split(",", 1)[0].strip()
    filename = scenario_prefix.lower().replace(":", "").replace(" ", "_").replace(".", "")
    output_path = os.path.join(output_dir, f"checks_per_second_{filename}.png")
    fig.subplots_adjust(top=0.9, bottom=0.2, left=0.04, right=0.98)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_all_scenarios(df, output_dir, color_map):
    scenarios = list(df["scenario"].unique())
    if not scenarios:
        return

    managers = sorted(df["manager"].unique())

    fig, axes = plt.subplots(len(scenarios), 3, figsize=(12, 4 * len(scenarios)), sharey=False)
    if len(scenarios) == 1:
        axes = [axes]

    # A row's bars are laid out across the full axis whatever its manager count, so a row with
    # fewer managers would draw wider bars. Scale the width down to keep them equal across rows.
    widest_row = max(df[df["scenario"] == s]["manager"].nunique() for s in scenarios)

    for row_idx, scenario in enumerate(scenarios):
        df_s = df[df["scenario"] == scenario]
        row_managers = sorted(df_s["manager"].unique())
        bar_width = 0.5 * len(row_managers) / widest_row
        row_axes = axes[row_idx]
        for ax, mode in zip(row_axes, MODE_ORDER):
            df_mode = df_s[df_s["mode"] == mode]
            series = (
                df_mode.groupby("manager")["checks_per_second"]
                .mean()
                .reindex(row_managers)
            )

            series.plot(kind="bar", ax=ax, color=[color_map[m] for m in series.index], width=bar_width)
            # Pin the limits to the slots themselves; the default margin scales with the slot
            # count, which would undo the width correction above.
            ax.set_xlim(-0.5, len(row_managers) - 0.5)
            ax.set_title("")
            ax.set_xlabel(mode)
            if ax is row_axes[0]:
                ax.set_ylabel("Checks Per Second")
            else:
                ax.set_ylabel("")
            ax.set_xticks([])
            ax.grid(axis="y", linestyle=":", alpha=0.5)

        row_axes[1].set_title(scenario, pad=12)

    legend_handles = [Patch(color=color_map[m], label=m) for m in managers]
    # Wrap to two rows past three managers: a single-row legend is wider than the plots, and
    # the saved bounding box is tight around every artist, so it would pad the sides.
    legend_cols = math.ceil(len(managers) / 2) if len(managers) > 3 else len(managers)
    legend_rows = math.ceil(len(managers) / legend_cols)
    fig.legend(
        handles=legend_handles,
        title="Manager",
        loc="lower center",
        bbox_to_anchor=(0.5, 0.0),
        ncol=legend_cols,
        frameon=True,
    )

    output_path = os.path.join(output_dir, "checks_per_second_all_scenarios.png")
    # The figure grows with the scenario count, so reserve the legend and titles in inches; a
    # fixed fraction would scale the gap above the legend with the number of scenarios.
    fig_height = 4 * len(scenarios)
    fig.subplots_adjust(
        top=1 - 0.4 / fig_height,
        bottom=(0.45 + 0.3 * legend_rows) / fig_height,
        left=0.05,
        right=0.995,
        hspace=0.3,
    )
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Plot benchmark CSV.")
    parser.add_argument("csv", help="Path to benchmark CSV")
    parser.add_argument("--output-dir", default=".",
                        help="Directory for output images")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    df = pd.read_csv(args.csv)
    color_map = build_color_map(sorted(df["manager"].unique()))

    for scenario in df["scenario"].unique():
        plot_scenario(df, scenario, args.output_dir, color_map)

    plot_all_scenarios(df, args.output_dir, color_map)


if __name__ == "__main__":
    main()
