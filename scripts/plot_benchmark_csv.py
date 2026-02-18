#!/usr/bin/env python3
"""Plot benchmark CSV into grouped bar charts."""

import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import pandas as pd

MODE_ORDER = ["FIRST", "CLOSEST", "ALL"]


def plot_scenario(df, scenario, output_dir):
    df_s = df[df["scenario"] == scenario].copy()

    if df_s.empty:
        return

    managers = sorted(df_s["manager"].unique())

    fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharey=False)
    fig.suptitle(f"Checks Per Second — {scenario}")

    palette = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    color_map = {manager: palette[i % len(palette)] for i, manager in enumerate(managers)}

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
    filename = scenario_prefix.lower().replace(" ", "_").replace(".", "")
    output_path = os.path.join(output_dir, f"checks_per_second_{filename}.png")
    fig.subplots_adjust(top=0.9, bottom=0.2, left=0.04, right=0.98)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_all_scenarios(df, output_dir):
    scenarios = list(df["scenario"].unique())
    if not scenarios:
        return

    managers = sorted(df["manager"].unique())
    palette = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    color_map = {manager: palette[i % len(palette)] for i, manager in enumerate(managers)}

    fig, axes = plt.subplots(len(scenarios), 3, figsize=(12, 4 * len(scenarios)), sharey=False)
    if len(scenarios) == 1:
        axes = [axes]

    for row_idx, scenario in enumerate(scenarios):
        df_s = df[df["scenario"] == scenario]
        row_axes = axes[row_idx]
        for ax, mode in zip(row_axes, MODE_ORDER):
            df_mode = df_s[df_s["mode"] == mode]
            series = (
                df_mode.groupby("manager")["checks_per_second"]
                .mean()
                .reindex(managers)
            )

            series.plot(kind="bar", ax=ax, color=[color_map[m] for m in series.index])
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
    fig.legend(
        handles=legend_handles,
        title="Manager",
        loc="lower center",
        bbox_to_anchor=(0.5, 0.03),
        ncol=len(managers),
        frameon=True,
    )

    output_path = os.path.join(output_dir, "checks_per_second_all_scenarios.png")
    fig.subplots_adjust(top=0.96, bottom=0.09, left=0.08, right=0.98, hspace=0.3)
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

    for scenario in df["scenario"].unique():
        plot_scenario(df, scenario, args.output_dir)

    plot_all_scenarios(df, args.output_dir)


if __name__ == "__main__":
    main()
