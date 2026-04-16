from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm


REPO_ROOT = Path(__file__).resolve().parents[3]
FIGURE_DIR = REPO_ROOT / "figures" / "03_controllability_observability"
OUTPUT_DIR = REPO_ROOT / "generated" / "03_controllability_observability"

A = np.array([[0.0, 1.0], [-2.0, -1.0]], dtype=float)
B = np.array([[0.0], [1.0]], dtype=float)
C = np.array([[1.0, 0.3]], dtype=float)
X0_STEER = np.array([0.2, -0.8], dtype=float)
XF_STEER = np.array([1.0, 0.0], dtype=float)
X0_RECON = np.array([0.8, -1.1], dtype=float)
T_HORIZON = 5.0
NUM_SAMPLES = 1200
RECONSTRUCTION_DT = 0.6


def controllability_matrix() -> np.ndarray:
    return np.hstack([B, A @ B])


def pbh_rank(eigenvalue: complex, matrix: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(matrix.astype(complex) + 0.0j))


def minimum_energy_control(t_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dt = t_grid[1] - t_grid[0]
    phi_horizon = expm(A * T_HORIZON)
    gramian = np.zeros((2, 2), dtype=float)

    for tau in t_grid:
        factor = expm(A * (T_HORIZON - tau)) @ B
        gramian += (factor @ factor.T) * dt

    target_gap = XF_STEER - phi_horizon @ X0_STEER
    gramian_inv = np.linalg.inv(gramian)
    control = np.array(
        [
            float((B.T @ expm(A.T * (T_HORIZON - tau)) @ gramian_inv @ target_gap).item())
            for tau in t_grid
        ],
        dtype=float,
    )
    return gramian, control


def simulate_steering(t_grid: np.ndarray, control: np.ndarray) -> np.ndarray:
    def dynamics(t: float, x: np.ndarray) -> np.ndarray:
        u = float(np.interp(t, t_grid, control))
        return A @ x + B[:, 0] * u

    solution = solve_ivp(
        dynamics,
        (float(t_grid[0]), float(t_grid[-1])),
        X0_STEER,
        t_eval=t_grid,
        rtol=1e-8,
        atol=1e-10,
    )
    return solution.y.T


def reconstruct_initial_state() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ad = expm(A * RECONSTRUCTION_DT)
    observability = np.vstack([C, C @ ad])
    outputs = np.array(
        [
            float((C @ X0_RECON.reshape(-1, 1)).item()),
            float((C @ (ad @ X0_RECON).reshape(-1, 1)).item()),
        ],
        dtype=float,
    )
    estimate = np.linalg.solve(observability, outputs)
    return observability, outputs, estimate


def simulate_zero_input(initial_state: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    return np.array([expm(A * t) @ initial_state for t in t_grid], dtype=float)


def save_steering_plot(t_grid: np.ndarray, states: np.ndarray, control: np.ndarray) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.2), sharex=True)

    axes[0].plot(t_grid, states[:, 0], linewidth=2.0, color="#0b5fff", label=r"$x_1(t)$")
    axes[0].plot(
        t_grid,
        states[:, 1],
        linewidth=2.0,
        color="#d1495b",
        linestyle=(0, (6, 4)),
        label=r"$x_2(t)$",
    )
    axes[0].scatter([t_grid[0], t_grid[-1]], [X0_STEER[0], XF_STEER[0]], color="#1f1f1f", s=18)
    axes[0].set_ylabel("states")
    axes[0].set_title("Minimum-energy state steering")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t_grid, control, linewidth=2.0, color="#2a9d8f")
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("u(t)")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "minimum_energy_steering.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_reconstruction_plot(t_grid: np.ndarray, truth: np.ndarray, estimate: np.ndarray) -> None:
    error_norm = np.linalg.norm(truth - estimate, axis=1)

    fig, axes = plt.subplots(3, 1, figsize=(8.4, 7.0), sharex=True)
    axes[0].plot(t_grid, truth[:, 0], linewidth=2.0, color="#0b5fff", label=r"True $x_1$")
    axes[0].plot(
        t_grid,
        estimate[:, 0],
        linewidth=2.0,
        color="#0b5fff",
        linestyle=(0, (6, 4)),
        label=r"Reconstructed $x_1$",
    )
    axes[0].set_ylabel(r"$x_1$")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t_grid, truth[:, 1], linewidth=2.0, color="#d1495b", label=r"True $x_2$")
    axes[1].plot(
        t_grid,
        estimate[:, 1],
        linewidth=2.0,
        color="#d1495b",
        linestyle=(0, (6, 4)),
        label=r"Reconstructed $x_2$",
    )
    axes[1].set_ylabel(r"$x_2$")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].semilogy(t_grid, np.maximum(error_norm, 1.0e-14), linewidth=2.0, color="#2a9d8f")
    axes[2].set_xlabel("t (s)")
    axes[2].set_ylabel(r"$\|x-\hat x\|_2$")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "state_reconstruction_comparison.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    t_grid = np.linspace(0.0, T_HORIZON, NUM_SAMPLES)
    gramian, control = minimum_energy_control(t_grid)
    steering_states = simulate_steering(t_grid, control)

    observability, output_samples, estimated_x0 = reconstruct_initial_state()
    truth_states = simulate_zero_input(X0_RECON, t_grid)
    estimated_states = simulate_zero_input(estimated_x0, t_grid)

    eigenvalues = np.linalg.eigvals(A)
    controllability = controllability_matrix()
    controllability_rank = int(np.linalg.matrix_rank(controllability))
    observability_rank = int(np.linalg.matrix_rank(observability))
    pbh_controllability = [
        {
            "eigenvalue_real": float(val.real),
            "eigenvalue_imag": float(val.imag),
            "rank": pbh_rank(val, np.hstack([val * np.eye(2) - A, B])),
        }
        for val in eigenvalues
    ]
    pbh_observability = [
        {
            "eigenvalue_real": float(val.real),
            "eigenvalue_imag": float(val.imag),
            "rank": pbh_rank(val, np.vstack([val * np.eye(2) - A, C])),
        }
        for val in eigenvalues
    ]

    save_steering_plot(t_grid, steering_states, control)
    save_reconstruction_plot(t_grid, truth_states, estimated_states)

    report = {
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "output_matrix_C": C.tolist(),
        "controllability_matrix": controllability.tolist(),
        "controllability_rank": controllability_rank,
        "observability_matrix_from_two_samples": observability.tolist(),
        "observability_rank": observability_rank,
        "pbh_controllability_ranks": pbh_controllability,
        "pbh_observability_ranks": pbh_observability,
        "stabilizable": controllability_rank == 2,
        "detectable": observability_rank == 2,
        "minimum_energy_gramian": gramian.tolist(),
        "steering_initial_state": X0_STEER.tolist(),
        "steering_target_state": XF_STEER.tolist(),
        "steering_terminal_error_norm": float(np.linalg.norm(steering_states[-1] - XF_STEER)),
        "reconstruction_reference_state": X0_RECON.tolist(),
        "reconstruction_output_samples": output_samples.tolist(),
        "reconstructed_initial_state": estimated_x0.tolist(),
        "reconstruction_error_norm": float(np.linalg.norm(estimated_x0 - X0_RECON)),
    }
    (OUTPUT_DIR / "controllability_observability_report.json").write_text(
        json.dumps(report, indent=2),
        encoding="utf-8",
    )

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
