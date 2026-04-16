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
FIGURE_DIR = REPO_ROOT / "figures" / "01_state_space_modeling"
OUTPUT_DIR = REPO_ROOT / "generated" / "01_state_space_modeling"

MASS = 1.0
DAMPING = 0.6
STIFFNESS = 2.0

A = np.array([[0.0, 1.0], [-STIFFNESS / MASS, -DAMPING / MASS]], dtype=float)
B = np.array([[0.0], [1.0 / MASS]], dtype=float)
C = np.array([[1.0, 0.0]], dtype=float)
D = np.array([[0.0]], dtype=float)
X0 = np.array([0.8, -0.3], dtype=float)
T_END = 12.0
NUM_SAMPLES = 1200
UNIT_INPUT = 1.0


def simulate(initial_state: np.ndarray, input_signal, t_grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    solution = solve_ivp(
        lambda t, x: A @ x + B[:, 0] * float(input_signal(t)),
        (float(t_grid[0]), float(t_grid[-1])),
        initial_state,
        t_eval=t_grid,
        rtol=1e-8,
        atol=1e-10,
    )
    states = solution.y.T
    controls = np.array([float(input_signal(t)) for t in t_grid], dtype=float)
    outputs = (states @ C.T).ravel() + D[0, 0] * controls
    return states, outputs


def save_state_output_plot(t_grid: np.ndarray, states: np.ndarray, outputs: np.ndarray) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.4), sharex=True)

    axes[0].plot(t_grid, states[:, 0], linewidth=2.2, color="#0b5fff", label=r"$x_1(t)=q(t)$")
    axes[0].plot(
        t_grid,
        states[:, 1],
        linewidth=2.2,
        color="#d1495b",
        linestyle=(0, (6, 4)),
        label=r"$x_2(t)=\dot q(t)$",
    )
    axes[0].set_ylabel("states")
    axes[0].set_title("State-space response of the mass-spring-damper model")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(t_grid, outputs, linewidth=2.2, color="#2a9d8f", label=r"$y(t)=q(t)$")
    axes[1].axhline(0.0, color="black", linewidth=0.6)
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("output")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "state_output_response.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_forced_response_plot(
    t_grid: np.ndarray,
    zero_output: np.ndarray,
    forced_output: np.ndarray,
    equilibrium_output: float,
) -> None:
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    ax.plot(t_grid, zero_output, linewidth=2.0, color="#4c6ef5", label="Zero input")
    ax.plot(t_grid, forced_output, linewidth=2.0, color="#e76f51", label="Unit input")
    ax.axhline(equilibrium_output, color="#1f1f1f", linestyle="--", linewidth=1.1, label="Unit-input equilibrium")
    ax.set_xlabel("t (s)")
    ax.set_ylabel(r"$y(t)$")
    ax.set_title("Forced-response comparison in state-space form")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "forced_response_comparison.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)
    free_states, free_outputs = simulate(X0, lambda _t: 0.0, t_grid)
    zero_states, zero_outputs = simulate(np.zeros(2, dtype=float), lambda _t: 0.0, t_grid)
    forced_states, forced_outputs = simulate(np.zeros(2, dtype=float), lambda _t: UNIT_INPUT, t_grid)

    equilibrium_state = -np.linalg.solve(A, B[:, 0] * UNIT_INPUT)
    equilibrium_output = float((C @ equilibrium_state.reshape(-1, 1)).item())
    transition_at_1s = expm(A)
    eigenvalues = np.linalg.eigvals(A)

    save_state_output_plot(t_grid, free_states, free_outputs)
    save_forced_response_plot(t_grid, zero_outputs, forced_outputs, equilibrium_output)

    report = {
        "physical_parameters": {
            "mass": MASS,
            "damping": DAMPING,
            "stiffness": STIFFNESS,
        },
        "state_space_model": {
            "A": A.tolist(),
            "B": B.tolist(),
            "C": C.tolist(),
            "D": D.tolist(),
        },
        "reference_initial_state": X0.tolist(),
        "unit_input_equilibrium_state": equilibrium_state.tolist(),
        "unit_input_equilibrium_output": equilibrium_output,
        "transition_matrix_at_1s": transition_at_1s.tolist(),
        "eigenvalues": [{"real": float(val.real), "imag": float(val.imag)} for val in eigenvalues],
        "free_response_final_state": free_states[-1].tolist(),
        "free_response_final_output": float(free_outputs[-1]),
        "zero_input_final_output": float(zero_outputs[-1]),
        "forced_response_final_state": forced_states[-1].tolist(),
        "forced_response_final_output": float(forced_outputs[-1]),
    }
    (OUTPUT_DIR / "model_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
