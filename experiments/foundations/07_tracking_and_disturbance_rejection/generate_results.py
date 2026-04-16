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
FIGURE_DIR = REPO_ROOT / "figures" / "07_tracking_and_disturbance_rejection"
OUTPUT_DIR = REPO_ROOT / "generated" / "07_tracking_and_disturbance_rejection"

A = np.array([[0.0, 1.0], [1.0, 0.2]], dtype=float)
B = np.array([[0.0], [1.0]], dtype=float)
C = np.array([[1.0, 0.0]], dtype=float)
T_END = 12.0
NUM_SAMPLES = 1500
REFERENCE = 1.0
DISTURBANCE_MAGNITUDE = 0.35
DISTURBANCE_START = 4.0


def disturbance(t: float) -> float:
    return DISTURBANCE_MAGNITUDE if t >= DISTURBANCE_START else 0.0


def settling_time(t_grid: np.ndarray, output: np.ndarray, reference: float, band: float = 0.02) -> float:
    error = np.abs(output - reference)
    threshold = band * max(abs(reference), 1.0)
    suffix_max = np.maximum.accumulate(error[::-1])[::-1]
    indices = np.where(suffix_max <= threshold)[0]
    return float(t_grid[indices[0]]) if indices.size else float("nan")


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    base_gain = place_poles(A, B, np.array([-1.2, -1.9], dtype=float)).gain_matrix
    k_state = -base_gain
    reference_gain = float((-1.0 / (C @ np.linalg.solve(A + B @ k_state, B))).item())

    a_aug = np.block([[A, np.zeros((2, 1))], [-C, np.zeros((1, 1))]])
    b_aug = np.vstack([B, [[0.0]]])
    augmented_gain = place_poles(a_aug, b_aug, np.array([-1.2, -1.9, -2.6], dtype=float)).gain_matrix
    kx_integral = -augmented_gain[0, :2]
    ki_integral = float(-augmented_gain[0, 2])

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)

    def state_feedback_dynamics(t: float, x: np.ndarray) -> np.ndarray:
        u = float((k_state @ x).item() + reference_gain * REFERENCE)
        return A @ x + B[:, 0] * (u + disturbance(t))

    def integral_servo_dynamics(t: float, z: np.ndarray) -> np.ndarray:
        x = z[:2]
        xi = z[2]
        u = float((kx_integral @ x).item() + ki_integral * xi)
        x_dot = A @ x + B[:, 0] * (u + disturbance(t))
        xi_dot = REFERENCE - float((C @ x).item())
        return np.array([x_dot[0], x_dot[1], xi_dot], dtype=float)

    state_feedback_solution = solve_ivp(
        state_feedback_dynamics,
        (0.0, T_END),
        np.zeros(2, dtype=float),
        t_eval=t_grid,
        rtol=1e-8,
        atol=1e-10,
    )
    integral_solution = solve_ivp(
        integral_servo_dynamics,
        (0.0, T_END),
        np.zeros(3, dtype=float),
        t_eval=t_grid,
        rtol=1e-8,
        atol=1e-10,
    )

    x_state_feedback = state_feedback_solution.y.T
    x_integral = integral_solution.y[:2].T
    xi_integral = integral_solution.y[2]

    y_state_feedback = (x_state_feedback @ C.T).ravel()
    y_integral = (x_integral @ C.T).ravel()
    u_state_feedback = (x_state_feedback @ k_state.T).ravel() + reference_gain * REFERENCE
    u_integral = (x_integral @ kx_integral.reshape(-1, 1)).ravel() + ki_integral * xi_integral

    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    ax.plot(t_grid, y_state_feedback, linewidth=2.0, color="#d1495b", label="State feedback")
    ax.plot(t_grid, y_integral, linewidth=2.0, color="#0b5fff", label="Integral servo")
    ax.axhline(REFERENCE, color="#1f1f1f", linestyle="--", linewidth=1.0, label="Reference")
    ax.axvline(DISTURBANCE_START, color="#6c757d", linestyle=":", linewidth=1.0, label="Disturbance starts")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("y(t)")
    ax.set_title("Tracking response with constant input disturbance")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "tracking_response_under_disturbance.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    ax.plot(t_grid, u_state_feedback, linewidth=2.0, color="#d1495b", label="State feedback")
    ax.plot(t_grid, u_integral, linewidth=2.0, color="#0b5fff", label="Integral servo")
    ax.axvline(DISTURBANCE_START, color="#6c757d", linestyle=":", linewidth=1.0)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("u(t)")
    ax.set_title("Control input under reference tracking")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "tracking_control_input.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    report = {
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "output_matrix_C": C.tolist(),
        "reference": REFERENCE,
        "disturbance": {
            "magnitude": DISTURBANCE_MAGNITUDE,
            "start_time": DISTURBANCE_START,
        },
        "state_feedback": {
            "K": k_state.tolist(),
            "reference_gain": reference_gain,
            "steady_state_error": float(abs(y_state_feedback[-1] - REFERENCE)),
            "overshoot": float(max(np.max(y_state_feedback) - REFERENCE, 0.0)),
            "settling_time_2pct": settling_time(t_grid, y_state_feedback, REFERENCE),
            "control_peak_abs": float(np.max(np.abs(u_state_feedback))),
        },
        "integral_servo": {
            "Kx": kx_integral.tolist(),
            "Ki": ki_integral,
            "steady_state_error": float(abs(y_integral[-1] - REFERENCE)),
            "overshoot": float(max(np.max(y_integral) - REFERENCE, 0.0)),
            "settling_time_2pct": settling_time(t_grid, y_integral, REFERENCE),
            "control_peak_abs": float(np.max(np.abs(u_integral))),
        },
    }
    (OUTPUT_DIR / "tracking_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
