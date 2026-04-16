from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles


REPO_ROOT = Path(__file__).resolve().parents[3]
FIGURE_DIR = REPO_ROOT / "figures" / "05_observer_and_separation"
OUTPUT_DIR = REPO_ROOT / "generated" / "05_observer_and_separation"

A = np.array([[0.0, 1.0], [1.2, 0.3]], dtype=float)
B = np.array([[0.0], [1.0]], dtype=float)
C = np.array([[1.0, 0.0]], dtype=float)
X0 = np.array([1.2, -0.8], dtype=float)
XHAT0 = np.array([0.0, 0.0], dtype=float)
CONTROLLER_POLES = np.array([-1.4, -2.2], dtype=float)
OBSERVER_POLES = np.array([-4.8, -5.6], dtype=float)
T_END = 8.0
NUM_SAMPLES = 1200


def settling_time(t_grid: np.ndarray, values: np.ndarray, threshold: float) -> float:
    suffix_max = np.maximum.accumulate(values[::-1])[::-1]
    indices = np.where(suffix_max <= threshold)[0]
    return float(t_grid[indices[0]]) if indices.size else float("nan")


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    controller_gain = place_poles(A, B, CONTROLLER_POLES).gain_matrix
    K = -controller_gain
    L = place_poles(A.T, C.T, OBSERVER_POLES).gain_matrix.T
    acl = A + B @ K
    observer_error_matrix = A - L @ C

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)

    def augmented_dynamics(_t: float, z: np.ndarray) -> np.ndarray:
        x = z[:2]
        x_hat = z[2:]
        u = float((K @ x_hat).item())
        y = float((C @ x).item())
        x_dot = A @ x + B[:, 0] * u
        x_hat_dot = A @ x_hat + B[:, 0] * u + L[:, 0] * (y - float((C @ x_hat).item()))
        return np.hstack([x_dot, x_hat_dot])

    solution = solve_ivp(
        augmented_dynamics,
        (0.0, T_END),
        np.hstack([X0, XHAT0]),
        t_eval=t_grid,
        rtol=1e-8,
        atol=1e-10,
    )

    states = solution.y[:2].T
    estimates = solution.y[2:].T
    error = estimates - states
    error_norm = np.linalg.norm(error, axis=1)
    control = (estimates @ K.T).ravel()
    combined_matrix = np.block(
        [
            [A, B @ K],
            [L @ C, A + B @ K - L @ C],
        ]
    )

    fig, axes = plt.subplots(3, 1, figsize=(8.4, 7.0), sharex=True)
    axes[0].plot(t_grid, states[:, 0], linewidth=2.0, color="#0b5fff", label=r"True $x_1$")
    axes[0].plot(
        t_grid,
        estimates[:, 0],
        linewidth=2.0,
        color="#0b5fff",
        linestyle=(0, (6, 4)),
        label=r"Estimated $\hat x_1$",
    )
    axes[0].set_ylabel(r"$x_1$")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t_grid, states[:, 1], linewidth=2.0, color="#d1495b", label=r"True $x_2$")
    axes[1].plot(
        t_grid,
        estimates[:, 1],
        linewidth=2.0,
        color="#d1495b",
        linestyle=(0, (6, 4)),
        label=r"Estimated $\hat x_2$",
    )
    axes[1].set_ylabel(r"$x_2$")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].semilogy(t_grid, np.maximum(error_norm, 1.0e-14), linewidth=2.0, color="#2a9d8f")
    axes[2].set_xlabel("t (s)")
    axes[2].set_ylabel(r"$\|e(t)\|_2$")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "observer_state_estimates.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.0), sharex=True)
    axes[0].plot(t_grid, states[:, 0], linewidth=2.0, color="#0b5fff", label=r"$x_1(t)$")
    axes[0].plot(
        t_grid,
        states[:, 1],
        linewidth=2.0,
        color="#d1495b",
        linestyle=(0, (6, 4)),
        label=r"$x_2(t)$",
    )
    axes[0].set_ylabel("states")
    axes[0].set_title("Observer-based closed-loop response")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t_grid, control, linewidth=2.0, color="#2a9d8f")
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("u(t)")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "observer_closed_loop_response.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    report = {
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "output_matrix_C": C.tolist(),
        "state_feedback_gain_K": K.tolist(),
        "observer_gain_L": L.tolist(),
        "controller_poles": CONTROLLER_POLES.tolist(),
        "observer_poles": OBSERVER_POLES.tolist(),
        "state_feedback_closed_loop_eigenvalues": [
            {"real": float(val.real), "imag": float(val.imag)} for val in np.linalg.eigvals(acl)
        ],
        "observer_error_eigenvalues": [
            {"real": float(val.real), "imag": float(val.imag)} for val in np.linalg.eigvals(observer_error_matrix)
        ],
        "combined_augmented_eigenvalues": [
            {"real": float(val.real), "imag": float(val.imag)} for val in np.linalg.eigvals(combined_matrix)
        ],
        "initial_state": X0.tolist(),
        "initial_estimate": XHAT0.tolist(),
        "max_estimation_error_norm": float(np.max(error_norm)),
        "final_estimation_error_norm": float(error_norm[-1]),
        "estimation_error_settling_time_norm_le_1e_3": settling_time(t_grid, error_norm, 1.0e-3),
        "control_peak_abs": float(np.max(np.abs(control))),
    }
    (OUTPUT_DIR / "observer_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
