from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm, solve_continuous_are


REPO_ROOT = Path(__file__).resolve().parents[3]
FIGURE_DIR = REPO_ROOT / "figures" / "06_optimal_control"
OUTPUT_DIR = REPO_ROOT / "generated" / "06_optimal_control"

A = np.array([[0.0, 1.0], [1.0, 0.2]], dtype=float)
B = np.array([[0.0], [1.0]], dtype=float)
X0 = np.array([1.0, -1.5], dtype=float)
T_END = 10.0
NUM_SAMPLES = 1200

WEIGHT_CASES = {
    "state_priority": {
        "Q": np.diag([12.0, 2.0]).astype(float),
        "R": np.array([[0.25]], dtype=float),
        "color": "#0b5fff",
        "label": "State-priority",
    },
    "control_priority": {
        "Q": np.diag([3.0, 1.0]).astype(float),
        "R": np.array([[1.5]], dtype=float),
        "color": "#d1495b",
        "label": "Control-priority",
    },
}


def simulate(acl: np.ndarray, k_gain: np.ndarray, t_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    states = np.array([expm(acl * t) @ X0 for t in t_grid], dtype=float)
    control = (states @ k_gain.T).ravel()
    return states, control


def quadratic_cost(states: np.ndarray, control: np.ndarray, q_mat: np.ndarray, r_mat: np.ndarray, t_grid: np.ndarray) -> float:
    integrand = np.array(
        [
            state @ q_mat @ state + float(r_mat[0, 0] * control_value**2)
            for state, control_value in zip(states, control)
        ],
        dtype=float,
    )
    return float(np.trapezoid(integrand, t_grid))


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)
    results: dict[str, dict[str, object]] = {}

    fig_states, axes_states = plt.subplots(2, 1, figsize=(8.4, 6.4), sharex=True)
    fig_control, ax_control = plt.subplots(figsize=(8.4, 4.8))

    for key, case in WEIGHT_CASES.items():
        q_mat = case["Q"]
        r_mat = case["R"]
        color = str(case["color"])
        label = str(case["label"])

        p_mat = solve_continuous_are(A, B, q_mat, r_mat)
        k_gain = -np.linalg.solve(r_mat, B.T @ p_mat)
        acl = A + B @ k_gain
        states, control = simulate(acl, k_gain, t_grid)
        cost = quadratic_cost(states, control, q_mat, r_mat, t_grid)

        axes_states[0].plot(t_grid, states[:, 0], linewidth=2.0, color=color, label=f"{label} $x_1$")
        axes_states[1].plot(
            t_grid,
            states[:, 1],
            linewidth=2.0,
            color=color,
            linestyle=(0, (6, 4)) if key == "control_priority" else "-",
            label=f"{label} $x_2$",
        )
        ax_control.plot(t_grid, control, linewidth=2.0, color=color, label=label)

        results[key] = {
            "Q": q_mat.tolist(),
            "R": r_mat.tolist(),
            "P": p_mat.tolist(),
            "K": k_gain.tolist(),
            "closed_loop_eigenvalues": [
                {"real": float(val.real), "imag": float(val.imag)} for val in np.linalg.eigvals(acl)
            ],
            "state_peak_abs": float(np.max(np.abs(states))),
            "control_peak_abs": float(np.max(np.abs(control))),
            "quadratic_cost": cost,
            "final_state": states[-1].tolist(),
        }

    axes_states[0].set_ylabel(r"$x_1(t)$")
    axes_states[0].set_title("LQR state responses under different weight selections")
    axes_states[0].grid(True, alpha=0.3)
    axes_states[0].legend(loc="upper right")
    axes_states[1].set_ylabel(r"$x_2(t)$")
    axes_states[1].set_xlabel("t (s)")
    axes_states[1].grid(True, alpha=0.3)
    axes_states[1].legend(loc="upper right")
    fig_states.tight_layout()
    fig_states.savefig(FIGURE_DIR / "lqr_weight_tradeoff_states.png", dpi=160, bbox_inches="tight")
    plt.close(fig_states)

    ax_control.set_xlabel("t (s)")
    ax_control.set_ylabel("u(t)")
    ax_control.set_title("LQR control effort trade-off")
    ax_control.grid(True, alpha=0.3)
    ax_control.legend(loc="upper right")
    fig_control.tight_layout()
    fig_control.savefig(FIGURE_DIR / "lqr_weight_tradeoff_control.png", dpi=160, bbox_inches="tight")
    plt.close(fig_control)

    report = {
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "initial_state": X0.tolist(),
        "cases": results,
    }
    (OUTPUT_DIR / "lqr_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
