from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
from scipy.integrate import solve_ivp


ROOT = Path(__file__).resolve().parents[3]
FIG_DIR = ROOT / "figures" / "04_robust_control"
GEN_DIR = ROOT / "generated" / "04_robust_control"


def uncertainty_matrix(rho_1: float, rho_2: float) -> np.ndarray:
    return np.array([[0.0, 1.0], [1.2 + rho_1, 0.3 + rho_2]], dtype=float)


def disturbance(t: np.ndarray | float) -> np.ndarray | float:
    return 0.8 * np.exp(-0.3 * t) * np.sin(2.4 * t)


def estimate_hinf_norm(
    a_cl: np.ndarray,
    d_mat: np.ndarray,
    e_mat: np.ndarray,
    frequencies: np.ndarray,
) -> float:
    max_sigma = 0.0
    identity = np.eye(a_cl.shape[0], dtype=complex)
    for omega in frequencies:
        transfer = e_mat @ np.linalg.inv(1j * omega * identity - a_cl) @ d_mat
        sigma = np.linalg.norm(transfer, ord=2)
        max_sigma = max(max_sigma, float(sigma))
    return max_sigma


def simulate_case(
    a_cl: np.ndarray,
    k_gain: np.ndarray,
    horizon: float = 15.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_eval = np.linspace(0.0, horizon, 1501)

    def dynamics(t: float, x: np.ndarray) -> np.ndarray:
        return a_cl @ x + np.array([0.0, disturbance(t)], dtype=float)

    solution = solve_ivp(
        dynamics,
        (0.0, horizon),
        np.zeros(2, dtype=float),
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
    )
    control = (k_gain @ solution.y).ravel()
    return solution.t, solution.y, control


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    GEN_DIR.mkdir(parents=True, exist_ok=True)

    b_mat = np.array([[0.0], [1.0]], dtype=float)
    d_mat = np.array([[0.0], [1.0]], dtype=float)
    e_mat = np.diag([1.0, 0.4]).astype(float)
    q_mat = np.diag([6.0, 2.0]).astype(float)
    r_mat = np.array([[1.0]], dtype=float)

    a_nominal = uncertainty_matrix(0.0, 0.0)
    p_mat = linalg.solve_continuous_are(a_nominal, b_mat, q_mat, r_mat)
    k_gain = -np.linalg.solve(r_mat, b_mat.T @ p_mat)

    rho_1_grid = np.linspace(-0.6, 0.6, 81)
    rho_2_grid = np.linspace(-0.25, 0.25, 81)
    open_loop_alpha = np.zeros((rho_2_grid.size, rho_1_grid.size), dtype=float)
    closed_loop_alpha = np.zeros_like(open_loop_alpha)

    for i, rho_2 in enumerate(rho_2_grid):
        for j, rho_1 in enumerate(rho_1_grid):
            a_mat = uncertainty_matrix(rho_1, rho_2)
            a_cl = a_mat + b_mat @ k_gain
            open_loop_alpha[i, j] = np.max(np.linalg.eigvals(a_mat).real)
            closed_loop_alpha[i, j] = np.max(np.linalg.eigvals(a_cl).real)

    vertex_points = [
        ("V1", -0.6, -0.25),
        ("V2", -0.6, 0.25),
        ("V3", 0.6, -0.25),
        ("V4", 0.6, 0.25),
        ("Nominal", 0.0, 0.0),
    ]
    frequency_grid = np.logspace(-3, 3, 4000)
    vertex_metrics: list[dict[str, float | str]] = []
    for label, rho_1, rho_2 in vertex_points:
        a_mat = uncertainty_matrix(rho_1, rho_2)
        a_cl = a_mat + b_mat @ k_gain
        lyapunov_residual = a_cl.T @ p_mat + p_mat @ a_cl
        vertex_metrics.append(
            {
                "label": label,
                "rho_1": rho_1,
                "rho_2": rho_2,
                "open_loop_alpha": float(np.max(np.linalg.eigvals(a_mat).real)),
                "closed_loop_alpha": float(np.max(np.linalg.eigvals(a_cl).real)),
                "lyapunov_residual_alpha": float(
                    np.max(np.linalg.eigvals(lyapunov_residual).real)
                ),
                "hinf_estimate": estimate_hinf_norm(a_cl, d_mat, e_mat, frequency_grid),
            }
        )

    representative_cases = [
        ("rho1=-0.6, rho2=-0.25", -0.6, -0.25),
        ("rho1=0.0, rho2=0.0", 0.0, 0.0),
        ("rho1=0.6, rho2=0.25", 0.6, 0.25),
    ]
    response_data = []
    for label, rho_1, rho_2 in representative_cases:
        a_cl = uncertainty_matrix(rho_1, rho_2) + b_mat @ k_gain
        time, states, control = simulate_case(a_cl, k_gain)
        response_data.append(
            {
                "label": label,
                "rho_1": rho_1,
                "rho_2": rho_2,
                "time": time,
                "states": states,
                "control": control,
                "state_peak": float(np.max(np.abs(states))),
                "control_peak": float(np.max(np.abs(control))),
            }
        )

    summary = {
        "A_nominal": a_nominal.tolist(),
        "B": b_mat.tolist(),
        "D": d_mat.tolist(),
        "E": e_mat.tolist(),
        "K": k_gain.tolist(),
        "P": p_mat.tolist(),
        "uncertainty_box": {"rho_1": [-0.6, 0.6], "rho_2": [-0.25, 0.25]},
        "worst_closed_loop_alpha_on_grid": float(np.max(closed_loop_alpha)),
        "best_closed_loop_alpha_on_grid": float(np.min(closed_loop_alpha)),
        "worst_open_loop_alpha_on_grid": float(np.max(open_loop_alpha)),
        "best_open_loop_alpha_on_grid": float(np.min(open_loop_alpha)),
        "vertex_metrics": vertex_metrics,
        "representative_peaks": [
            {
                "label": item["label"],
                "state_peak": item["state_peak"],
                "control_peak": item["control_peak"],
            }
            for item in response_data
        ],
    }

    with (GEN_DIR / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    with (GEN_DIR / "vertex_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(vertex_metrics[0].keys()))
        writer.writeheader()
        writer.writerows(vertex_metrics)

    plt.rcParams.update(
        {
            "figure.figsize": (12, 4.8),
            "axes.grid": True,
            "grid.alpha": 0.25,
            "font.size": 11,
        }
    )

    fig, axes = plt.subplots(1, 2, constrained_layout=True)
    extent = [rho_1_grid[0], rho_1_grid[-1], rho_2_grid[0], rho_2_grid[-1]]
    image_1 = axes[0].imshow(
        open_loop_alpha,
        origin="lower",
        aspect="auto",
        extent=extent,
        cmap="magma",
    )
    axes[0].set_title("Open-loop spectral abscissa")
    axes[0].set_xlabel(r"$\rho_1$")
    axes[0].set_ylabel(r"$\rho_2$")
    fig.colorbar(image_1, ax=axes[0], shrink=0.85)

    image_2 = axes[1].imshow(
        closed_loop_alpha,
        origin="lower",
        aspect="auto",
        extent=extent,
        cmap="viridis",
    )
    axes[1].set_title("Closed-loop spectral abscissa")
    axes[1].set_xlabel(r"$\rho_1$")
    axes[1].set_ylabel(r"$\rho_2$")
    fig.colorbar(image_2, ax=axes[1], shrink=0.85)
    fig.savefig(FIG_DIR / "spectral_abscissa_scan.png", dpi=220)
    plt.close(fig)

    labels = [item["label"] for item in vertex_metrics]
    hinf_values = [float(item["hinf_estimate"]) for item in vertex_metrics]
    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    ax.bar(labels, hinf_values, color=["#456990", "#4d9078", "#f4d35e", "#ee964b", "#7d5ba6"])
    ax.axhline(0.5, color="#bc412b", linestyle="--", linewidth=1.6, label="gamma = 0.5")
    ax.set_ylabel("Estimated gain")
    ax.set_title(r"Estimated $H_\infty$ gain at interval vertices")
    ax.legend()
    fig.savefig(FIG_DIR / "vertex_hinf_estimates.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10.5, 6.4), sharex=True, constrained_layout=True)
    for item in response_data:
        axes[0].plot(item["time"], item["states"][0], label=f"{item['label']} : x1")
        axes[1].plot(item["time"], item["states"][1], label=f"{item['label']} : x2")
    axes[0].set_ylabel("x1(t)")
    axes[1].set_ylabel("x2(t)")
    axes[1].set_xlabel("Time (s)")
    axes[0].set_title("Closed-loop states under decaying disturbance")
    axes[0].legend(ncol=3, fontsize=8)
    axes[1].legend(ncol=3, fontsize=8)
    fig.savefig(FIG_DIR / "state_response_under_disturbance.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.5, 4.6), constrained_layout=True)
    for item in response_data:
        ax.plot(item["time"], item["control"], linewidth=1.8, label=item["label"])
    sample_time = response_data[0]["time"]
    ax.plot(sample_time, disturbance(sample_time), color="#222222", linestyle="--", linewidth=1.2, label="w(t)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Signal value")
    ax.set_title("Control input and disturbance")
    ax.legend(ncol=4, fontsize=8)
    fig.savefig(FIG_DIR / "control_response_under_disturbance.png", dpi=220)
    plt.close(fig)

    print("Robust control reproduction completed.")
    print(f"K = {k_gain}")
    print(f"Worst closed-loop spectral abscissa on grid: {np.max(closed_loop_alpha):.6f}")
    print(f"Worst estimated vertex Hinf gain: {max(hinf_values):.6f}")


if __name__ == "__main__":
    main()
