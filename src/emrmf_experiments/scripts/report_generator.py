#!/usr/bin/env python3

import pandas as pd
import numpy as np
import scipy.stats as stats
import os
import argparse

def compute_ci(data, confidence=0.95):
    n = len(data)
    if n < 2:
        return 0.0
    _, se = np.mean(data), stats.sem(data)
    h = se * stats.t.ppf((1 + confidence) / 2., n-1)
    return h

def format_cell(mean, std, ci):
    return f"{mean:.3f} \u00B1 {std:.3f} (CI: \u00B1{ci:.3f})"

def format_cell_latex(mean, std, ci):
    return f"${mean:.3f} \\pm {std:.3f}$"

def generate_reports(input_csv='/tmp/emrmf_experiments/raw_results.csv', output_dir='/tmp/emrmf_experiments'):
    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found.")
        return

    df = pd.read_csv(input_csv)

    # Group by configuration
    group_cols = ['ablation_mode', 'p', 'gamma', 'delay', 'packet_loss']
    grouped = df.groupby(group_cols)

    metrics = ['pose_rmse', 'map_alignment_rmse', 'fusion_time', 'theta_mean', 'theta_std']

    summary_data = []
    markdown_lines = ["| Mode | p | gamma | Delay | Loss | Pose RMSE | Map RMSE | Fusion Time (s) | Theta Mean | Runs |",
                      "|---|---|---|---|---|---|---|---|---|---|"]

    latex_lines = [
        "\\begin{table}[h]",
        "\\centering",
        "\\caption{EMRMF Experimental Results}",
        "\\begin{tabular}{l c c c c c c c c}",
        "\\hline",
        "Mode & $p$ & $\\gamma$ & Delay (s) & Loss & Pose RMSE & Map RMSE & Fusion Time (s) & $\\bar{\\theta}$ \\\\",
        "\\hline"
    ]

    for name, group in grouped:
        mode, p, gamma, delay, loss = name
        n_runs = len(group)

        row_dict = {
            'ablation_mode': mode, 'p': p, 'gamma': gamma, 'delay': delay, 'packet_loss': loss, 'runs': n_runs
        }

        md_row = [str(mode), str(p), str(gamma), str(delay), f"{loss*100:.0f}%"]
        tex_row = [str(mode).replace("_", "\\_"), str(p), str(gamma), str(delay), f"{loss*100:.0f}\\%"]

        for metric in metrics:
            data = group[metric].dropna().values
            if len(data) > 0:
                mean = np.mean(data)
                std = np.std(data)
                ci = compute_ci(data)
            else:
                mean, std, ci = 0.0, 0.0, 0.0

            row_dict[f'{metric}_mean'] = mean
            row_dict[f'{metric}_std'] = std
            row_dict[f'{metric}_ci95'] = ci

            if metric != 'theta_std': # Exclude from simple tables to save space
                md_row.append(format_cell(mean, std, ci))
                tex_row.append(format_cell_latex(mean, std, ci))

        summary_data.append(row_dict)
        md_row.append(str(n_runs))
        markdown_lines.append("| " + " | ".join(md_row) + " |")
        latex_lines.append(" & ".join(tex_row) + " \\\\")

    latex_lines.append("\\hline")
    latex_lines.append("\\end{tabular}")
    latex_lines.append("\\end{table}")

    # Save CSV
    summary_df = pd.DataFrame(summary_data)
    summary_csv = os.path.join(output_dir, 'summary_results.csv')
    summary_df.to_csv(summary_csv, index=False)

    # Save Markdown
    md_file = os.path.join(output_dir, 'summary_results.md')
    with open(md_file, 'w') as f:
        f.write("\n".join(markdown_lines))

    # Save LaTeX
    tex_file = os.path.join(output_dir, 'summary_results.tex')
    with open(tex_file, 'w') as f:
        f.write("\n".join(latex_lines))

    print(f"Reports generated in {output_dir}:")
    print(f" - {summary_csv}")
    print(f" - {md_file}")
    print(f" - {tex_file}")

def main():
    parser = argparse.ArgumentParser(description="Generate EMRMF experiment reports.")
    parser.add_argument('--input', default='/tmp/emrmf_experiments/raw_results.csv', help='Input raw results CSV file')
    parser.add_argument('--output', default='/tmp/emrmf_experiments', help='Output directory')
    args = parser.parse_args()
    generate_reports(args.input, args.output)

if __name__ == '__main__':
    main()
