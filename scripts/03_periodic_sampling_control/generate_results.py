from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eigvals, expm


REPO_ROOT = Path(__file__).resolve().parents[2]
FIGURE_DIR = REPO_ROOT / "figures" / "03_periodic_sampling"
OUTPUT_DIR = REPO_ROOT / "generated" / "03_periodic_sampling"

A = np.array([[-0.479908, -3.81625], [5.1546, 14.4723]], dtype=float)
B = np.diag([5.8705212, 15.50107]).astype(float)
K = np.array([[-0.3399, -0.0263], [-0.0628, -1.2527]], dtype=float)
ALPHA = 0.05
H = 0.125
X0 = np.array([1.0, -1.0], dtype=float)
T_END = 6.0
POINTS_PER_INTERVAL = 40


def zoh_discretize(tau: float) -> tuple[np.ndarray, np.ndarray]:
    n = A.shape[0]
    m = B.shape[1]
    augmented = np.zeros((n + m, n + m), dtype=float)
    augmented[:n, :n] = A
    augmented[:n, n:] = B
    transition = expm(augmented * tau)
    ad = transition[:n, :n]
    bd = transition[:n, n:]
    return ad, bd


def simulate() -> dict[str, np.ndarray]:
    num_intervals = int(round(T_END / H))
    sample_times = np.linspace(0.0, num_intervals * H, num_intervals + 1)
    sampled_states = np.zeros((num_intervals + 1, 2), dtype=float)
    sampled_states[0] = X0
    sampled_controls = np.zeros((num_intervals, 2), dtype=float)

    continuous_time_segments: list[np.ndarray] = []
    continuous_state_segments: list[np.ndarray] = []

    local_tau = np.linspace(0.0, H, POINTS_PER_INTERVAL, endpoint=False)
    ad_tau_list = []
    bd_tau_list = []
    for tau in local_tau:
        ad_tau, bd_tau = zoh_discretize(float(tau))
        ad_tau_list.append(ad_tau)
        bd_tau_list.append(bd_tau)

    ad_h, bd_h = zoh_discretize(H)

    for k in range(num_intervals):
        xk = sampled_states[k]
        uk = K @ xk
        sampled_controls[k] = uk

        t0 = sample_times[k]
        local_times = t0 + local_tau
        states = np.array([ad @ xk + bd @ uk for ad, bd in zip(ad_tau_list, bd_tau_list)], dtype=float)
        continuous_time_segments.append(local_times)
        continuous_state_segments.append(states)

        sampled_states[k + 1] = ad_h @ xk + bd_h @ uk

    continuous_times = np.concatenate(continuous_time_segments + [sample_times[-1:]])
    continuous_states = np.vstack(continuous_state_segments + [sampled_states[-1:]])

    return {
        "sample_times": sample_times,
        "sampled_states": sampled_states,
        "sampled_controls": sampled_controls,
        "continuous_times": continuous_times,
        "continuous_states": continuous_states,
        "control_times": sample_times,
        "control_values": np.vstack([sampled_controls, sampled_controls[-1]]),
        "ad_h": ad_h,
        "bd_h": bd_h,
    }


def save_state_plot(data: dict[str, np.ndarray], spectral_radius: float) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(data["continuous_times"], data["continuous_states"][:, 0], linewidth=1.8, color="#1f77b4", label=r"$x_1(t)$")
    ax.plot(data["continuous_times"], data["continuous_states"][:, 1], linewidth=1.8, color="#d2691e", label=r"$x_2(t)$")
    ax.scatter(data["sample_times"], data["sampled_states"][:, 0], s=14, color="#d9a520", label=r"$x_1[k]$")
    ax.scatter(data["sample_times"], data["sampled_states"][:, 1], s=12, color="#7b2cbf", label=r"$x_2[k]$")
    ax.axhline(0.0, color="black", linewidth=0.6)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("states")
    ax.set_title(f"Continuous and sampled states (h={H:.3f} s, rho={spectral_radius:.6f})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "continuous_sampled_states.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def save_control_plot(data: dict[str, np.ndarray]) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.step(data["control_times"], data["control_values"][:, 0], where="post", linewidth=1.8, color="#1f77b4", label=r"$u_1(t)$")
    ax.step(data["control_times"], data["control_values"][:, 1], where="post", linewidth=1.8, color="#d2691e", label=r"$u_2(t)$")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("u(t)")
    ax.set_title("ZOH control signals")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "zoh_control_signals.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    data = simulate()
    discrete_closed_loop = data["ad_h"] + data["bd_h"] @ K
    discrete_eigs = eigvals(discrete_closed_loop)
    spectral_radius = float(np.max(np.abs(discrete_eigs)))

    save_state_plot(data, spectral_radius)
    save_control_plot(data)

    report = {
        "alpha": ALPHA,
        "sampling_period": H,
        "system_matrix_A": A.tolist(),
        "input_matrix_B": B.tolist(),
        "feedback_gain_K": K.tolist(),
        "initial_state": X0.tolist(),
        "discrete_closed_loop_matrix": discrete_closed_loop.tolist(),
        "discrete_closed_loop_eigenvalues": [
            {"real": float(val.real), "imag": float(val.imag)} for val in discrete_eigs
        ],
        "spectral_radius": spectral_radius,
        "final_sampled_state": data["sampled_states"][-1].tolist(),
        "final_control": data["sampled_controls"][-1].tolist(),
    }
    (OUTPUT_DIR / "periodic_sampling_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Saved report to: {OUTPUT_DIR}")
    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
