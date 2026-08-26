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


def duty_cycle_of(df):
    """Return the single duty cycle value in df, or None if every cell is unrecorded.

    A cell is unrecorded either because the column is absent entirely (a pre-duty_cycle-column
    CSV) or because it is present but empty for that row (e.g. such a CSV appended under the
    current 7-column header). Both count as "unrecorded", not as a value to compare against real
    ones, so they are dropped before inspecting what is left.

    Refuses (raises SystemExit) a CSV that mixes duty cycles, including a mix of a recorded value
    with unrecorded rows: averaging checks_per_second across rows produced under different duty
    cycles (or of unknown duty cycle) would silently blend incomparable measurements into one
    meaningless bar.
    """
    if "duty_cycle" not in df.columns:
        return None

    recorded = df["duty_cycle"].dropna()
    if recorded.empty:
        return None

    values = sorted(recorded.unique())
    if len(values) > 1 or len(recorded) != len(df):
        seen = values + (["unrecorded"] if len(recorded) != len(df) else [])
        raise SystemExit(
            f"Refusing to plot: CSV mixes duty cycles {seen}. "
            "Filter the file to a single duty cycle before plotting."
        )
    return values[0]


def duty_cycle_label(duty_cycle):
    """Title fragment for a duty cycle, or 'unrecorded' for a pre-duty_cycle-column CSV."""
    return duty_cycle if duty_cycle is not None else "unrecorded"


def duty_cycle_suffix(duty_cycle):
    """Filename suffix for a duty cycle, or empty for a pre-duty_cycle-column CSV so that
    existing output filenames from before this column existed keep their names."""
    return f"_{duty_cycle}" if duty_cycle is not None else ""


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


def plot_scenario(df, scenario, output_dir, color_map, duty_cycle):
    df_s = df[df["scenario"] == scenario].copy()

    if df_s.empty:
        return

    managers = sorted(df_s["manager"].unique())

    fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharey=False)
    fig.suptitle(f"Checks Per Second — {scenario} (duty cycle: {duty_cycle_label(duty_cycle)})")

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
    output_path = os.path.join(output_dir, f"checks_per_second_{filename}{duty_cycle_suffix(duty_cycle)}.png")
    fig.subplots_adjust(top=0.9, bottom=0.2, left=0.04, right=0.98)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_all_scenarios(df, output_dir, color_map, duty_cycle):
    scenarios = list(df["scenario"].unique())
    if not scenarios:
        return

    managers = sorted(df["manager"].unique())

    # The figure grows with the scenario count, so the suptitle is positioned by a fixed inch
    # offset from the top rather than matplotlib's default fractional y=0.98: a fractional
    # position stays visually fixed only for a fixed figure height, and here the figure height
    # scales with the scenario count while the suptitle's own text height (in inches) does not,
    # so a fractional y drifts toward the top edge — and toward the first row's title, reserved
    # in inches below it — as more scenarios are added. See the matching inch-based reasoning on
    # the top/bottom margins below.
    fig_height = 4 * len(scenarios)
    fig, axes = plt.subplots(len(scenarios), 3, figsize=(12, fig_height), sharey=False)
    fig.suptitle(
        f"Checks Per Second — All Scenarios (duty cycle: {duty_cycle_label(duty_cycle)})",
        y=1 - 0.35 / fig_height,
    )
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

    output_path = os.path.join(
        output_dir, f"checks_per_second_all_scenarios{duty_cycle_suffix(duty_cycle)}.png"
    )
    # The figure grows with the scenario count, so reserve the legend and titles in inches; a
    # fixed fraction would scale the gap above the legend with the number of scenarios. The top
    # margin reserves room for both the figure suptitle and the first row's per-scenario title
    # stacked above it (fig_height computed above, alongside the suptitle's own inch-based y).
    fig.subplots_adjust(
        top=1 - 1.1 / fig_height,
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
    duty_cycle = duty_cycle_of(df)
    color_map = build_color_map(sorted(df["manager"].unique()))

    for scenario in df["scenario"].unique():
        plot_scenario(df, scenario, args.output_dir, color_map, duty_cycle)

    plot_all_scenarios(df, args.output_dir, color_map, duty_cycle)


if __name__ == "__main__":
    main()
