from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eigvals, expm, solve_continuous_lyapunov


REPO_ROOT = Path(__file__).resolve().parents[3]
OUTPUT_DIR = REPO_ROOT / "generated" / "02_lti_stability"
FIGURE_DIR = REPO_ROOT / "figures" / "02_lti_stability"

A = np.array([[0.0, 1.0], [-1.0, -1.0]], dtype=float)
Q = np.eye(2)
T_END = 20.0
NUM_SAMPLES = 1000
REFERENCE_X0 = np.array([1.0, -1.0], dtype=float)


def simulate_trajectory(x0: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    states = np.empty((t_grid.size, x0.size), dtype=float)
    for index, t in enumerate(t_grid):
        states[index] = expm(A * t) @ x0
    return states


def solve_lyapunov_matrix() -> np.ndarray:
    # scipy solves A X + X A^T = Q. Using A^T and Q = -I gives:
    # A^T P + P A = -I.
    return solve_continuous_lyapunov(A.T, -Q)


def save_state_plot(t_grid: np.ndarray, states: np.ndarray, destination: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

    labels = [r"$x_1(t)$", r"$x_2(t)$"]
    colors = ["#0b5fff", "#d1495b"]

    for axis, column, label, color in zip(axes, range(states.shape[1]), labels, colors):
        axis.plot(t_grid, states[:, column], color=color, linewidth=2)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("t")
    fig.suptitle("State Trajectories for A = [[0, 1], [-1, -1]] and x(0) = [1, -1]^T")
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_phase_portrait(destination: Path) -> None:
    grid = np.linspace(-2.0, 2.0, 21)
    x1, x2 = np.meshgrid(grid, grid)
    u = A[0, 0] * x1 + A[0, 1] * x2
    v = A[1, 0] * x1 + A[1, 1] * x2

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)
    initial_conditions = [
        np.array([1.0, -1.0]),
        np.array([1.2, 0.8]),
        np.array([-1.0, 1.0]),
        np.array([-1.5, -0.5]),
        np.array([0.5, -1.5]),
    ]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.streamplot(x1, x2, u, v, density=1.0, color="#b8c4d6", linewidth=0.9, arrowsize=1)

    for x0 in initial_conditions:
        states = simulate_trajectory(x0, t_grid)
        ax.plot(states[:, 0], states[:, 1], linewidth=2)
        ax.scatter(states[0, 0], states[0, 1], s=18)

    ax.scatter(0.0, 0.0, color="black", s=28, label="equilibrium")
    ax.set_xlabel(r"$x_1$")
    ax.set_ylabel(r"$x_2$")
    ax.set_title("Phase Portrait of the LTI System")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def write_report(destination: Path, lyapunov_matrix: np.ndarray, states: np.ndarray) -> None:
    eigenvalues = eigvals(A)
    p_eigenvalues = np.linalg.eigvalsh(lyapunov_matrix)
    residual = A.T @ lyapunov_matrix + lyapunov_matrix @ A + Q

    report = {
        "system_matrix": A.tolist(),
        "system_eigenvalues": [
            {"real": float(value.real), "imag": float(value.imag)} for value in eigenvalues
        ],
        "reference_initial_condition": REFERENCE_X0.tolist(),
        "lyapunov_equation": "A^T P + P A = -I",
        "lyapunov_matrix": lyapunov_matrix.tolist(),
        "lyapunov_matrix_eigenvalues": p_eigenvalues.tolist(),
        "lyapunov_residual_frobenius_norm": float(np.linalg.norm(residual, ord="fro")),
        "final_state_at_t_end": states[-1].tolist(),
    }

    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)
    lyapunov_matrix = solve_lyapunov_matrix()
    states = simulate_trajectory(REFERENCE_X0, t_grid)

    save_state_plot(t_grid, states, FIGURE_DIR / "state_trajectories.png")
    save_phase_portrait(FIGURE_DIR / "phase_portrait.png")
    write_report(OUTPUT_DIR / "lyapunov_report.json", lyapunov_matrix, states)

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
