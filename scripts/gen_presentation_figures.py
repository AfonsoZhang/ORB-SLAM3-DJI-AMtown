#!/usr/bin/env python3
"""Generate presentation figures for ORB-SLAM3 DJI AMtown project."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os

OUT = 'figures/presentation'
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    'figure.dpi': 150,
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
})

# ── 1. k1 Ablation Curve ──
fig, ax = plt.subplots(figsize=(8, 5))
k1_vals   = [-0.121, -0.056, -0.053, -0.045, -0.035]
ate_vals  = [110.7,   5.25,   2.39,   6.45,   22.1]
colors = ['#d62728' if a > 20 else '#ff7f0e' if a > 5 else '#2ca02c' for a in ate_vals]

ax.plot(k1_vals, ate_vals, 'o-', color='#1f77b4', linewidth=2, markersize=10, zorder=3)
for i, (k, a) in enumerate(zip(k1_vals, ate_vals)):
    ax.scatter(k, a, c=colors[i], s=120, zorder=4, edgecolors='black', linewidths=0.8)
    offset = (0, 12) if a > 10 else (0, -18)
    ax.annotate(f'{a:.1f}m', (k, a), textcoords='offset points',
                xytext=offset, ha='center', fontsize=11, fontweight='bold')

ax.axvline(x=-0.053, color='green', linestyle='--', alpha=0.5, label='Optimal k1=-0.053')
crash_k1 = -0.070
ax.axvline(x=crash_k1, color='red', linestyle=':', alpha=0.5)
ax.annotate('tracking\nfailure', (crash_k1, 60), ha='center', color='red', fontsize=10)

ax.set_xlabel('k1 (first-order radial distortion)')
ax.set_ylabel('ATE RMSE (m)')
ax.set_title('k1 Ablation: ATE varies 46× with k1 alone')
ax.set_yscale('log')
ax.set_yticks([2, 5, 10, 20, 50, 100])
ax.get_yaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{OUT}/k1_ablation.png')
plt.close()
print('Saved k1_ablation.png')


# ── 2. Calibration Sensitivity ──
fig, ax = plt.subplots(figsize=(8, 5))
configs = [
    'HKisland intrinsics\n(wrong fx/fy/cx/cy)',
    'Self-calibrated\n(k1=-0.121)',
    'HK_GNSS official\n(k1=-0.056)',
    'HK_GNSS + HKisland dist.\n(k1=-0.053)'
]
ates = [215.0, 113.4, 6.1, 2.31]
bar_colors = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4']

bars = ax.bar(range(len(configs)), ates, color=bar_colors, edgecolor='black', linewidth=0.8, width=0.6)
for bar, ate in zip(bars, ates):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 3,
            f'{ate:.1f}m', ha='center', va='bottom', fontweight='bold', fontsize=12)

ax.set_xticks(range(len(configs)))
ax.set_xticklabels(configs, fontsize=9)
ax.set_ylabel('ATE RMSE (m)')
ax.set_title('Calibration Impact on Trajectory Accuracy')
ax.set_ylim(0, 240)
ax.grid(axis='y', alpha=0.3)
fig.tight_layout()
fig.savefig(f'{OUT}/calibration_sensitivity.png')
plt.close()
print('Saved calibration_sensitivity.png')


# ── 3. Dual GT Comparison ──
fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))

metrics_labels = ['ATE RMSE', 'ATE Mean', 'ATE Std']
sfm_vals  = [2.310, 2.112, 0.937]
rtk_vals  = [2.647, 2.457, 0.984]

x = np.arange(len(metrics_labels))
w = 0.35
axes[0].bar(x - w/2, sfm_vals, w, label='SfM GT', color='#2ca02c', edgecolor='black', linewidth=0.5)
axes[0].bar(x + w/2, rtk_vals, w, label='RTK GPS GT', color='#1f77b4', edgecolor='black', linewidth=0.5)
for i in range(len(metrics_labels)):
    axes[0].text(x[i] - w/2, sfm_vals[i] + 0.05, f'{sfm_vals[i]:.2f}', ha='center', fontsize=9)
    axes[0].text(x[i] + w/2, rtk_vals[i] + 0.05, f'{rtk_vals[i]:.2f}', ha='center', fontsize=9)
axes[0].set_xticks(x)
axes[0].set_xticklabels(metrics_labels)
axes[0].set_ylabel('Error (m)')
axes[0].set_title('ATE Comparison')
axes[0].legend()
axes[0].grid(axis='y', alpha=0.3)

rpe_labels = ['RPE Trans\n(10m)', 'Completeness\n(%)']
sfm_rpe = [0.103, 100.0]
rtk_rpe = [20.58, 98.7]
x2 = np.arange(len(rpe_labels))
axes[1].bar(x2 - w/2, sfm_rpe, w, label='SfM GT', color='#2ca02c', edgecolor='black', linewidth=0.5)
axes[1].bar(x2 + w/2, rtk_rpe, w, label='RTK GPS GT', color='#1f77b4', edgecolor='black', linewidth=0.5)
for i in range(len(rpe_labels)):
    axes[1].text(x2[i] - w/2, sfm_rpe[i] + 0.3, f'{sfm_rpe[i]:.1f}', ha='center', fontsize=9)
    axes[1].text(x2[i] + w/2, rtk_rpe[i] + 0.3, f'{rtk_rpe[i]:.1f}', ha='center', fontsize=9)
axes[1].set_xticks(x2)
axes[1].set_xticklabels(rpe_labels)
axes[1].set_ylabel('Value')
axes[1].set_title('RPE & Completeness')
axes[1].legend()
axes[1].grid(axis='y', alpha=0.3)

fig.suptitle('Dual Ground Truth Validation (SfM vs RTK GPS)', fontsize=14, fontweight='bold')
fig.tight_layout()
fig.savefig(f'{OUT}/dual_gt.png')
plt.close()
print('Saved dual_gt.png')


# ── 4. Non-Determinism (6 runs) ──
fig, ax = plt.subplots(figsize=(7, 4.5))
runs = ['Run 0\n(best)', 'Run 1', 'Run 2', 'Run 3', 'Run 4', 'Run 5']
run_ates = [2.310, 2.470, 2.841, 3.527, 2.357, 2.318]
bar_colors_nd = ['#2ca02c'] + ['#1f77b4'] * 5
bars = ax.bar(range(len(runs)), run_ates, color=bar_colors_nd, edgecolor='black', linewidth=0.8, width=0.6)
for bar, ate in zip(bars, run_ates):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.03,
            f'{ate:.2f}m', ha='center', va='bottom', fontsize=10, fontweight='bold')

mean_ate = np.mean(run_ates)
std_ate = np.std(run_ates)
ax.axhline(y=mean_ate, color='red', linestyle='--', alpha=0.7, label=f'Mean={mean_ate:.2f}m (std={std_ate:.2f})')
ax.set_xticks(range(len(runs)))
ax.set_xticklabels(runs, fontsize=10)
ax.set_ylabel('ATE RMSE (m)')
ax.set_title('Non-Determinism: 6 Identical Runs')
ax.legend(fontsize=10)
ax.set_ylim(0, 4.2)
ax.grid(axis='y', alpha=0.3)
fig.tight_layout()
fig.savefig(f'{OUT}/nondeterminism.png')
plt.close()
print('Saved nondeterminism.png')


# ── 5. ORB Tuning (all with self-calibrated intrinsics) ──
fig, ax = plt.subplots(figsize=(8, 5))
exp_names = ['Baseline\nnF=4000 nL=12', 'Fewer feat.\nnF=3000 nL=8', 'Finer pyramid\nsFactor=1.1', 'More feat.\nnF=6000', 'Full res\n2448×2048']
exp_ates = [113.4, 128.5, 118.3, 176.8, 121.0]
bar_colors_orb = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
bars = ax.bar(range(len(exp_names)), exp_ates, color=bar_colors_orb, edgecolor='black', linewidth=0.8, width=0.6)
for bar, ate in zip(bars, exp_ates):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
            f'{ate:.1f}m', ha='center', va='bottom', fontsize=10, fontweight='bold')
ax.set_xticks(range(len(exp_names)))
ax.set_xticklabels(exp_names, fontsize=9)
ax.set_ylabel('ATE RMSE (m)')
ax.set_title('ORB Parameter Tuning (all with self-calibrated intrinsics)')
ax.set_ylim(0, 200)
ax.grid(axis='y', alpha=0.3)
fig.tight_layout()
fig.savefig(f'{OUT}/orb_tuning.png')
plt.close()
print('Saved orb_tuning.png')


# ── 6. Results Summary (AMtown02 + TUM-VI) ──
fig, ax = plt.subplots(figsize=(7, 4))
ax.axis('off')
table_data = [
    ['', 'AMtown02\nMono VO', 'TUM-VI\nMono-Inertial'],
    ['ATE RMSE', '2.31 m', '0.011 m'],
    ['Completeness', '100%', '97.9%'],
    ['Loop Closure', '✓', 'N/A'],
    ['Scale', '2.15 (mono)', '0.999 (metric)'],
    ['Config', 'HK_GNSS + HKisland\ndistortion', 'TUM-VI.yaml\n(standard)'],
]
table = ax.table(cellText=table_data, loc='center', cellLoc='center')
table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1.2, 1.8)
for (row, col), cell in table.get_celld().items():
    if row == 0:
        cell.set_facecolor('#4472C4')
        cell.set_text_props(color='white', fontweight='bold')
    elif col == 0:
        cell.set_facecolor('#D6E4F0')
        cell.set_text_props(fontweight='bold')
    cell.set_edgecolor('#888888')
ax.set_title('Final Results Summary', fontsize=14, fontweight='bold', pad=20)
fig.tight_layout()
fig.savefig(f'{OUT}/results_summary.png')
plt.close()
print('Saved results_summary.png')


print('\nAll presentation figures generated.')
