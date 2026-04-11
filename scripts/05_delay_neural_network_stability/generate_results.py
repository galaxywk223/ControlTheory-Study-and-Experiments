from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
FIG_DIR = ROOT / "figures" / "05_delay_neural_network_stability"
GEN_DIR = ROOT / "generated" / "05_delay_neural_network_stability"


def simulate_delay_network(
    a_mat: np.ndarray,
    w_mat: np.ndarray,
    tau: float,
    history: np.ndarray,
    dt: float = 0.002,
    horizon: float = 20.0,
) -> tuple[np.ndarray, np.ndarray]:
    delay_steps = max(0, int(round(tau / dt)))
    total_steps = int(round(horizon / dt))

    if delay_steps == 0:
        time = np.linspace(0.0, horizon, total_steps + 1)
        states = np.zeros((2, total_steps + 1), dtype=float)
        states[:, 0] = history
        for k in range(total_steps):
            delayed_state = states[:, k]
            derivative = a_mat @ states[:, k] + w_mat @ np.tanh(delayed_state)
            states[:, k + 1] = states[:, k] + dt * derivative
        return time, states

    tau_eff = delay_steps * dt
    time = np.linspace(-tau_eff, horizon, total_steps + delay_steps + 1)
    states = np.zeros((2, time.size), dtype=float)
    states[:, : delay_steps + 1] = history[:, None]

    for k in range(delay_steps, time.size - 1):
        delayed_state = states[:, k - delay_steps]
        derivative = a_mat @ states[:, k] + w_mat @ np.tanh(delayed_state)
        states[:, k + 1] = states[:, k] + dt * derivative

    return time[delay_steps:], states[:, delay_steps:]


def settling_time(time: np.ndarray, states: np.ndarray, threshold: float = 1e-2) -> float:
    norm_series = np.linalg.norm(states, axis=0)
    suffix_max = np.maximum.accumulate(norm_series[::-1])[::-1]
    indices = np.where(suffix_max <= threshold)[0]
    if indices.size == 0:
        return float("nan")
    return float(time[indices[0]])


def theorem_1_matrix(a_mat: np.ndarray, w_mat: np.ndarray, p_mat: np.ndarray, q_mat: np.ndarray) -> np.ndarray:
    return np.block([[a_mat.T @ p_mat + p_mat @ a_mat + q_mat, p_mat @ w_mat], [(p_mat @ w_mat).T, -q_mat]])


def theorem_2_matrix(a_mat: np.ndarray, w_mat: np.ndarray, p_mat: np.ndarray, q_mat: np.ndarray, l_mat: np.ndarray) -> np.ndarray:
    return np.block(
        [
            [a_mat.T @ p_mat + p_mat @ a_mat + l_mat @ q_mat @ l_mat, p_mat @ w_mat],
            [(p_mat @ w_mat).T, -q_mat],
        ]
    )


def theorem_3_matrix(
    a_mat: np.ndarray,
    w_mat: np.ndarray,
    p_mat: np.ndarray,
    q_mat: np.ndarray,
    r_mat: np.ndarray,
    u_mat: np.ndarray,
    l_mat: np.ndarray,
) -> np.ndarray:
    return np.block(
        [
            [a_mat.T @ p_mat + p_mat @ a_mat, r_mat @ a_mat + u_mat @ l_mat, p_mat @ w_mat],
            [(r_mat @ a_mat + u_mat @ l_mat).T, q_mat - (u_mat + u_mat.T), r_mat @ w_mat],
            [(p_mat @ w_mat).T, (r_mat @ w_mat).T, -q_mat],
        ]
    )


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    GEN_DIR.mkdir(parents=True, exist_ok=True)

    a_mat = -np.diag([1.0, 0.9]).astype(float)
    w_mat = np.array([[0.35, -0.28], [0.22, 0.31]], dtype=float)
    l_mat = np.eye(2, dtype=float)
    history = np.array([1.2, -1.0], dtype=float)

    p1 = np.diag([1.47220303, 3.34527307])
    q1 = np.diag([1.66769356, 1.72755548])
    p2 = np.diag([0.64768388, 3.40401467])
    q2 = np.diag([0.49623577, 3.05849385])
    p3 = np.diag([2.50148372, 2.19018502])
    q3 = np.diag([2.59316350, 1.49824932])
    r3 = np.diag([1.69318792, 0.66533231])
    u3 = np.diag([1.89660380, 2.26405287])

    theorem_margins = {
        "theorem_1": float(np.max(np.linalg.eigvals(theorem_1_matrix(a_mat, w_mat, p1, q1)).real)),
        "theorem_2": float(np.max(np.linalg.eigvals(theorem_2_matrix(a_mat, w_mat, p2, q2, l_mat)).real)),
        "theorem_3": float(np.max(np.linalg.eigvals(theorem_3_matrix(a_mat, w_mat, p3, q3, r3, u3, l_mat)).real)),
    }

    representative_taus = [0.2, 1.0, 2.0]
    representative_results = []
    for tau in representative_taus:
        time, states = simulate_delay_network(a_mat, w_mat, tau, history)
        representative_results.append(
            {
                "tau": tau,
                "time": time,
                "states": states,
                "peak": float(np.max(np.abs(states))),
                "settling_time": settling_time(time, states),
                "final_state": states[:, -1].tolist(),
            }
        )

    tau_grid = np.linspace(0.0, 2.2, 12)
    scan_rows = []
    for tau in tau_grid:
        time, states = simulate_delay_network(a_mat, w_mat, float(tau), history)
        scan_rows.append(
            {
                "tau": float(tau),
                "settling_time": settling_time(time, states),
                "final_norm": float(np.linalg.norm(states[:, -1])),
            }
        )

    summary = {
        "A": a_mat.tolist(),
        "W": w_mat.tolist(),
        "history": history.tolist(),
        "activation": "tanh",
        "theorem_1": {"P": p1.tolist(), "Q": q1.tolist(), "margin": theorem_margins["theorem_1"]},
        "theorem_2": {"P": p2.tolist(), "Q": q2.tolist(), "margin": theorem_margins["theorem_2"]},
        "theorem_3": {
            "P": p3.tolist(),
            "Q": q3.tolist(),
            "R": r3.tolist(),
            "U": u3.tolist(),
            "margin": theorem_margins["theorem_3"],
        },
        "representative_cases": [
            {
                "tau": item["tau"],
                "peak": item["peak"],
                "settling_time": item["settling_time"],
                "final_state": item["final_state"],
            }
            for item in representative_results
        ],
        "settling_scan": scan_rows,
    }

    with (GEN_DIR / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    with (GEN_DIR / "settling_scan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["tau", "settling_time", "final_norm"])
        writer.writeheader()
        writer.writerows(scan_rows)

    plt.rcParams.update(
        {
            "figure.figsize": (10, 5),
            "axes.grid": True,
            "grid.alpha": 0.25,
            "font.size": 11,
        }
    )

    fig, ax = plt.subplots(figsize=(8.6, 4.8), constrained_layout=True)
    labels = ["Theorem 1", "Theorem 2", "Theorem 3"]
    margins = [theorem_margins["theorem_1"], theorem_margins["theorem_2"], theorem_margins["theorem_3"]]
    ax.bar(labels, margins, color=["#456990", "#49a078", "#ee964b"])
    ax.axhline(0.0, color="#222222", linewidth=1.1)
    ax.set_ylabel("Largest eigenvalue")
    ax.set_title("LMI certificate margins")
    fig.savefig(FIG_DIR / "lmi_certificate_margins.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10.4, 6.2), sharex=True, constrained_layout=True)
    for item in representative_results:
        axes[0].plot(item["time"], item["states"][0], linewidth=1.8, label=fr"$\tau={item['tau']:.1f}$")
        axes[1].plot(item["time"], item["states"][1], linewidth=1.8, label=fr"$\tau={item['tau']:.1f}$")
    axes[0].set_ylabel("x1(t)")
    axes[1].set_ylabel("x2(t)")
    axes[1].set_xlabel("Time (s)")
    axes[0].set_title("State trajectories under different delays")
    axes[0].legend(ncol=3)
    axes[1].legend(ncol=3)
    fig.savefig(FIG_DIR / "state_trajectories_by_delay.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.8, 4.8), constrained_layout=True)
    ax.plot([row["tau"] for row in scan_rows], [row["settling_time"] for row in scan_rows], marker="o", linewidth=1.8)
    ax.set_xlabel(r"$\tau$")
    ax.set_ylabel("Settling time to ||x(t)|| <= 1e-2 (s)")
    ax.set_title("Delay versus settling time")
    fig.savefig(FIG_DIR / "delay_settling_time_scan.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.8, 5.2), constrained_layout=True)
    tau_focus = 2.0
    time, states = simulate_delay_network(a_mat, w_mat, tau_focus, history)
    ax.plot(states[0], states[1], color="#7d5ba6", linewidth=1.8)
    ax.scatter(states[0, 0], states[1, 0], color="#bc412b", s=36, label="start")
    ax.scatter(states[0, -1], states[1, -1], color="#2b6cb0", s=36, label="end")
    ax.set_xlabel("x1(t)")
    ax.set_ylabel("x2(t)")
    ax.set_title(r"Phase trajectory for $\tau=2.0$")
    ax.legend()
    fig.savefig(FIG_DIR / "phase_portrait_tau_2_0.png", dpi=220)
    plt.close(fig)

    print("Delay neural network reproduction completed.")
    print(f"Theorem 1 margin: {theorem_margins['theorem_1']:.6f}")
    print(f"Theorem 2 margin: {theorem_margins['theorem_2']:.6f}")
    print(f"Theorem 3 margin: {theorem_margins['theorem_3']:.6f}")


if __name__ == "__main__":
    main()
