from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm


REPO_ROOT = Path(__file__).resolve().parents[3]
FIGURE_DIR = REPO_ROOT / "figures" / "04_lti_control"
OUTPUT_DIR = REPO_ROOT / "generated" / "04_lti_control"

A = np.array([[0.12, 0.08], [-0.18, -0.02]], dtype=float)
B = np.eye(2)
C = np.diag([1.0, 0.8])
X0 = np.array([1.0, 2.0], dtype=float)
T_END = 20.0
NUM_SAMPLES = 1000

STATE_FEEDBACK_GAINS = {
    "theorem_2": np.array([[-2.12, 17.92], [-17.82, -2.28]], dtype=float),
    "theorem_3": np.array([[-1.92, 4.42], [-5.82, -1.58]], dtype=float),
}

OUTPUT_FEEDBACK_GAINS = {
    "theorem_5": np.array([[-2.32, 21.775], [-16.82, -2.475]], dtype=float),
    "theorem_6": np.array([[-2.92, 0.65], [-1.62, -4.35]], dtype=float),
    "theorem_7": np.array([[-2.52, 1.65], [-2.42, -3.725]], dtype=float),
}

FIGURE_NAMES = {
    "open_loop": "open_loop_state_response.png",
    "theorem_2": "theorem_2_state_feedback_response.png",
    "theorem_3": "theorem_3_state_feedback_response.png",
    "theorem_5": "theorem_5_output_feedback_response.png",
    "theorem_6": "theorem_6_output_feedback_response.png",
    "theorem_7": "theorem_7_output_feedback_response.png",
}


def simulate(matrix: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    return np.array([expm(matrix * t) @ X0 for t in t_grid], dtype=float)


def save_response_plot(t_grid: np.ndarray, states: np.ndarray, title: str, destination: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(t_grid, states[:, 0], linewidth=2.4, color="#0b5fff", label=r"$x_1(t)$")
    ax.plot(
        t_grid,
        states[:, 1],
        linewidth=2.4,
        color="#d1495b",
        linestyle=(0, (6, 5)),
        label=r"$x_2(t)$",
    )
    ax.set_xlabel("t")
    ax.set_ylabel("state")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(destination, dpi=160, bbox_inches="tight")
    plt.close(fig)


def write_report(report: dict[str, object], destination: Path) -> None:
    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    t_grid = np.linspace(0.0, T_END, NUM_SAMPLES)
    report: dict[str, object] = {
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "output_matrix_C": C.tolist(),
        "initial_state": X0.tolist(),
        "state_feedback_gains": {key: value.tolist() for key, value in STATE_FEEDBACK_GAINS.items()},
        "output_feedback_gains": {key: value.tolist() for key, value in OUTPUT_FEEDBACK_GAINS.items()},
        "responses": {},
    }

    open_loop_states = simulate(A, t_grid)
    save_response_plot(
        t_grid,
        open_loop_states,
        "Open-Loop State Response",
        FIGURE_DIR / FIGURE_NAMES["open_loop"],
    )
    report["responses"]["open_loop"] = {
        "closed_loop_matrix": A.tolist(),
        "final_state": open_loop_states[-1].tolist(),
        "eigenvalues": np.linalg.eigvals(A).tolist(),
    }

    for key, gain in STATE_FEEDBACK_GAINS.items():
        acl = A + B @ gain
        states = simulate(acl, t_grid)
        save_response_plot(
            t_grid,
            states,
            f"State-Feedback Response ({key.replace('_', ' ').title()})",
            FIGURE_DIR / FIGURE_NAMES[key],
        )
        report["responses"][key] = {
            "closed_loop_matrix": acl.tolist(),
            "gain": gain.tolist(),
            "final_state": states[-1].tolist(),
            "eigenvalues": np.linalg.eigvals(acl).tolist(),
        }

    for key, gain in OUTPUT_FEEDBACK_GAINS.items():
        acl = A + B @ gain @ C
        states = simulate(acl, t_grid)
        save_response_plot(
            t_grid,
            states,
            f"Output-Feedback Response ({key.replace('_', ' ').title()})",
            FIGURE_DIR / FIGURE_NAMES[key],
        )
        report["responses"][key] = {
            "closed_loop_matrix": acl.tolist(),
            "gain": gain.tolist(),
            "final_state": states[-1].tolist(),
            "eigenvalues": np.linalg.eigvals(acl).tolist(),
        }

    # Convert complex eigenvalues into JSON-friendly objects.
    for response in report["responses"].values():
        response["eigenvalues"] = [
            {"real": float(value.real), "imag": float(value.imag)} for value in response["eigenvalues"]
        ]

    write_report(report, OUTPUT_DIR / "control_report.json")
    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
