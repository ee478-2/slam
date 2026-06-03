#!/usr/bin/env python3
"""
Bar chart comparing Option C baseline vs Option C + OdomF2M/BundleAdjustment=1
across the 4 TUM-RGBD pioneer bags.
Output: eval_results/tum_bag/BA_ablation/plots/BA_comparison.png
"""
import numpy as np
import matplotlib.pyplot as plt
import os

# Numbers (m). Sources:
#   baseline = eval_results/tum_bag/option_C/pioneer_*/ape.txt
#   BA       = eval_results/tum_bag/BA_ablation/pioneer_*/ape.txt (measured 2026-05-03)
bags = ["360", "slam", "slam2", "slam3"]

baseline = {
    "max":  [0.637, 4.500, 3.745, 5.188],
    "mean": [0.146, 0.448, 1.497, 0.489],
    "rmse": [0.193, 0.542, 1.648, 0.743],
}
ba = {
    "max":  [0.269, 3.337, 3.769, 5.010],
    "mean": [0.136, 0.455, 1.456, 0.467],
    "rmse": [0.146, 0.543, 1.609, 0.690],
}

x = np.arange(len(bags))
w = 0.38

fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=False)
metrics = ["max", "mean", "rmse"]

for ax, m in zip(axes, metrics):
    b = baseline[m]
    a = ba[m]
    bars1 = ax.bar(x - w/2, b, w, label="Option C (baseline)",
                   color="#888888", edgecolor="black")
    bars2 = ax.bar(x + w/2, a, w, label="+ BundleAdjustment",
                   color="#4a90e2", edgecolor="black")

    # Annotate Δ%
    for i, (bv, av) in enumerate(zip(b, a)):
        delta = (av - bv) / bv * 100 if bv > 0 else 0
        color = "#1f7a1f" if delta < -1 else ("#a02020" if delta > 1 else "#666")
        ax.text(x[i] + w/2, av + max(b + a) * 0.01,
                f"{delta:+.0f}%", ha="center", va="bottom",
                fontsize=9, color=color, fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([f"pioneer_{b}" for b in bags], rotation=10)
    ax.set_ylabel(f"{m} APE (m)")
    ax.set_title(f"APE — {m}")
    ax.grid(axis="y", alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)

fig.suptitle("Option C vs +OdomF2M/BundleAdjustment=1   (TUM-RGBD pioneer bags @ rate=0.3)",
             fontsize=12, y=1.02)
plt.tight_layout()

out = os.path.join(os.path.dirname(__file__), "..", "eval_results",
                   "tum_bag", "BA_ablation", "plots", "BA_comparison.png")
out = os.path.abspath(out)
plt.savefig(out, dpi=140, bbox_inches="tight")
print(f"saved: {out}")
