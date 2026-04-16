from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
FIG_DIR = ROOT / "figures" / "11_chaotic_sync_and_image_encryption"
GEN_DIR = ROOT / "generated" / "11_chaotic_sync_and_image_encryption"
SOURCE_IMAGE = FIG_DIR / "plaintext_input_image.jpg"


def simulate_driver(
    horizon: float,
    dt: float = 0.001,
    tau: float = 1.0,
    history: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    if history is None:
        history = np.array([1.7, 2.5], dtype=float)

    a_mat = -np.eye(2, dtype=float)
    b_mat = np.array([[2.0, -0.1], [-5.0, 2.0]], dtype=float)
    bd_mat = np.array([[-1.5, -0.1], [-0.2, -1.5]], dtype=float)

    delay_steps = int(round(tau / dt))
    total_steps = int(round(horizon / dt))
    time = np.linspace(-tau, horizon, total_steps + delay_steps + 1)
    states = np.zeros((2, time.size), dtype=float)
    states[:, : delay_steps + 1] = history[:, None]

    for k in range(delay_steps, time.size - 1):
        delayed_state = states[:, k - delay_steps]
        derivative = (
            a_mat @ states[:, k]
            + b_mat @ np.tanh(states[:, k])
            + bd_mat @ np.tanh(delayed_state)
        )
        states[:, k + 1] = states[:, k] + dt * derivative

    return time[delay_steps:], states[:, delay_steps:]


def simulate_sync(
    horizon: float = 25.0,
    dt: float = 0.001,
    tau: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    a_mat = -np.eye(2, dtype=float)
    b_mat = np.array([[2.0, -0.1], [-5.0, 2.0]], dtype=float)
    bd_mat = np.array([[-1.5, -0.1], [-0.2, -1.5]], dtype=float)
    k_gain = np.array([[32.8175, 47.7361], [-133.1490, -79.0097]], dtype=float)

    x_history = np.array([1.7, 2.5], dtype=float)
    xhat_history = np.array([1.0, 2.0], dtype=float)

    delay_steps = int(round(tau / dt))
    total_steps = int(round(horizon / dt))
    time = np.linspace(-tau, horizon, total_steps + delay_steps + 1)
    drive = np.zeros((2, time.size), dtype=float)
    response = np.zeros((2, time.size), dtype=float)
    control = np.zeros((2, time.size), dtype=float)
    drive[:, : delay_steps + 1] = x_history[:, None]
    response[:, : delay_steps + 1] = xhat_history[:, None]

    for k in range(delay_steps, time.size - 1):
        drive_delay = drive[:, k - delay_steps]
        response_delay = response[:, k - delay_steps]
        error = response[:, k] - drive[:, k]
        u_val = k_gain @ error
        control[:, k] = u_val

        drive_derivative = (
            a_mat @ drive[:, k]
            + b_mat @ np.tanh(drive[:, k])
            + bd_mat @ np.tanh(drive_delay)
        )
        response_derivative = (
            a_mat @ response[:, k]
            + b_mat @ np.tanh(response[:, k])
            + bd_mat @ np.tanh(response_delay)
            + u_val
        )
        drive[:, k + 1] = drive[:, k] + dt * drive_derivative
        response[:, k + 1] = response[:, k] + dt * response_derivative

    valid = slice(delay_steps, None)
    return time[valid], drive[:, valid], response[:, valid], control[:, valid]


def settling_time(time: np.ndarray, error: np.ndarray, threshold: float) -> float:
    norm_series = np.linalg.norm(error, axis=0)
    suffix_max = np.maximum.accumulate(norm_series[::-1])[::-1]
    indices = np.where(suffix_max <= threshold)[0]
    if indices.size == 0:
        return float("nan")
    return float(time[indices[0]])


def chaotic_sequence(seed: float, length: int, burn_in: int = 1000) -> np.ndarray:
    value = float(np.clip(seed, 1e-6, 1.0 - 1e-6))
    for _ in range(burn_in):
        value = (3.99 * value * (1.0 - value) + 0.27 * np.sin(np.pi * value)) % 1.0
    output = np.empty(length, dtype=float)
    for i in range(length):
        value = (3.99 * value * (1.0 - value) + 0.27 * np.sin(np.pi * value)) % 1.0
        output[i] = value
    return output


def derive_image_seeds(time: np.ndarray, drive: np.ndarray) -> list[float]:
    samples = []
    for target in (12.0, 16.0, 20.0):
        index = int(np.argmin(np.abs(time - target)))
        samples.append(drive[:, index])
    seed_1 = (abs(samples[0][0]) * 1.0e4 + abs(samples[1][1]) * 1.0e3) % 1.0
    seed_2 = (abs(samples[1][0]) * 1.0e4 + abs(samples[2][1]) * 1.0e3 + 0.314159) % 1.0
    seed_3 = (abs(samples[2][0] - samples[0][1]) * 1.0e4 + 0.271828) % 1.0
    return [float(np.clip(seed, 1e-6, 1.0 - 1e-6)) for seed in (seed_1, seed_2, seed_3)]


def encrypt_image(image: np.ndarray, seeds: list[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    height, width, channels = image.shape
    row_order = np.argsort(chaotic_sequence(seeds[0], height), kind="mergesort")
    col_order = np.argsort(chaotic_sequence(seeds[1], width), kind="mergesort")
    permuted = image[row_order, :, :][:, col_order, :]

    key_length = height * width * channels
    key_stream = (
        np.floor(256.0 * chaotic_sequence(seeds[2], key_length)).astype(np.uint16)
        + (np.arange(key_length, dtype=np.uint16) % 256)
    ) % 256
    key_image = key_stream.astype(np.uint8).reshape(height, width, channels)
    cipher = np.bitwise_xor(permuted, key_image)
    recovered = np.bitwise_xor(cipher, key_image)

    inverse_row = np.empty_like(row_order)
    inverse_col = np.empty_like(col_order)
    inverse_row[row_order] = np.arange(height)
    inverse_col[col_order] = np.arange(width)
    decrypted = recovered[inverse_row, :, :][:, inverse_col, :]
    return cipher, decrypted, row_order, col_order


def rgb_to_gray(image: np.ndarray) -> np.ndarray:
    gray = 0.299 * image[:, :, 0] + 0.587 * image[:, :, 1] + 0.114 * image[:, :, 2]
    return np.round(gray).astype(np.uint8)


def adjacency_correlation(gray: np.ndarray, mode: str) -> float:
    gray_f = gray.astype(float)
    if mode == "horizontal":
        left = gray_f[:, :-1].ravel()
        right = gray_f[:, 1:].ravel()
    else:
        left = gray_f[:-1, :].ravel()
        right = gray_f[1:, :].ravel()
    return float(np.corrcoef(left, right)[0, 1])


def sampled_pairs(gray: np.ndarray, mode: str, samples: int = 4000) -> tuple[np.ndarray, np.ndarray]:
    gray_f = gray.astype(float)
    if mode == "horizontal":
        left = gray_f[:, :-1].ravel()
        right = gray_f[:, 1:].ravel()
    else:
        left = gray_f[:-1, :].ravel()
        right = gray_f[1:, :].ravel()
    indices = np.linspace(0, left.size - 1, min(samples, left.size), dtype=int)
    return left[indices], right[indices]


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    GEN_DIR.mkdir(parents=True, exist_ok=True)

    time_driver, driver_only = simulate_driver(80.0)
    time_sync, drive, response, control = simulate_sync()
    error = response - drive

    seeds = derive_image_seeds(time_sync, drive)

    image = np.array(Image.open(SOURCE_IMAGE).convert("RGB"), dtype=np.uint8)
    cipher, decrypted, row_order, col_order = encrypt_image(image, seeds)
    if not np.array_equal(image, decrypted):
        raise RuntimeError("Decryption did not recover the original image.")

    gray_original = rgb_to_gray(image)
    gray_cipher = rgb_to_gray(cipher)

    correlation_metrics = [
        ("original", "horizontal", adjacency_correlation(gray_original, "horizontal")),
        ("original", "vertical", adjacency_correlation(gray_original, "vertical")),
        ("encrypted", "horizontal", adjacency_correlation(gray_cipher, "horizontal")),
        ("encrypted", "vertical", adjacency_correlation(gray_cipher, "vertical")),
    ]

    summary = {
        "sync_system": {
            "A": [[-1.0, 0.0], [0.0, -1.0]],
            "B": [[2.0, -0.1], [-5.0, 2.0]],
            "B_delay": [[-1.5, -0.1], [-0.2, -1.5]],
            "K": [[32.8175, 47.7361], [-133.1490, -79.0097]],
            "tau": 1.0,
            "drive_history": [1.7, 2.5],
            "response_history": [1.0, 2.0],
        },
        "sync_metrics": {
            "settling_time_norm_le_1e-2": settling_time(time_sync, error, 1.0e-2),
            "settling_time_norm_le_1e-4": settling_time(time_sync, error, 1.0e-4),
            "max_error_norm": float(np.max(np.linalg.norm(error, axis=0))),
            "max_control_abs": [float(val) for val in np.max(np.abs(control), axis=1)],
            "final_error": [float(val) for val in error[:, -1]],
        },
        "image_encryption": {
            "source_image": str(SOURCE_IMAGE.relative_to(ROOT)).replace("\\", "/"),
            "seeds": seeds,
            "image_shape": list(image.shape),
            "decryption_exact": True,
            "correlations": [
                {"image": item[0], "direction": item[1], "value": item[2]}
                for item in correlation_metrics
            ],
            "row_perm_checksum": int(np.sum(row_order[:20])),
            "col_perm_checksum": int(np.sum(col_order[:20])),
        },
    }

    with (GEN_DIR / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    with (GEN_DIR / "correlation_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", "direction", "value"])
        writer.writerows(correlation_metrics)

    plt.rcParams.update(
        {
            "figure.figsize": (10, 5),
            "axes.grid": True,
            "grid.alpha": 0.25,
            "font.size": 11,
        }
    )

    fig, ax = plt.subplots(figsize=(6.2, 5.8), constrained_layout=True)
    mask = time_driver >= 20.0
    ax.plot(driver_only[0, mask], driver_only[1, mask], color="#1f5aa6", linewidth=0.9)
    ax.set_xlabel("x1(t)")
    ax.set_ylabel("x2(t)")
    ax.set_title("Drive-system phase portrait")
    fig.savefig(FIG_DIR / "drive_phase_portrait.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10.6, 6.2), sharex=True, constrained_layout=True)
    axes[0].plot(time_sync, error[0], linewidth=1.8, label="e1(t)")
    axes[0].plot(time_sync, error[1], linewidth=1.8, label="e2(t)")
    axes[0].set_ylabel("Error state")
    axes[0].set_title("Synchronization error trajectories")
    axes[0].legend()

    error_norm = np.linalg.norm(error, axis=0)
    axes[1].semilogy(time_sync, np.maximum(error_norm, 1.0e-12), color="#bc412b", linewidth=1.8)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("||e(t)||2")
    fig.savefig(FIG_DIR / "synchronization_error.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(9.2, 8.0), constrained_layout=True)
    axes[0, 0].imshow(image)
    axes[0, 0].set_title("Original image")
    axes[0, 1].imshow(image[:, :, 0], cmap="Reds")
    axes[0, 1].set_title("Red channel")
    axes[1, 0].imshow(image[:, :, 1], cmap="Greens")
    axes[1, 0].set_title("Green channel")
    axes[1, 1].imshow(image[:, :, 2], cmap="Blues")
    axes[1, 1].set_title("Blue channel")
    for axis in axes.ravel():
        axis.set_xticks([])
        axis.set_yticks([])
        axis.grid(False)
    fig.savefig(FIG_DIR / "plaintext_rgb_channels.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(12.0, 4.4), constrained_layout=True)
    axes[0].imshow(image)
    axes[0].set_title("Original")
    axes[1].imshow(cipher)
    axes[1].set_title("Encrypted")
    axes[2].imshow(decrypted)
    axes[2].set_title("Decrypted")
    for axis in axes:
        axis.set_xticks([])
        axis.set_yticks([])
        axis.grid(False)
    fig.savefig(FIG_DIR / "encryption_pipeline.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(11.6, 6.6), constrained_layout=True)
    channel_names = ["Red", "Green", "Blue"]
    channel_colors = ["#d1495b", "#2f9e44", "#3b82f6"]
    for idx in range(3):
        axes[0, idx].hist(image[:, :, idx].ravel(), bins=256, color=channel_colors[idx], alpha=0.9)
        axes[0, idx].set_title(f"Original {channel_names[idx]}")
        axes[1, idx].hist(cipher[:, :, idx].ravel(), bins=256, color=channel_colors[idx], alpha=0.9)
        axes[1, idx].set_title(f"Encrypted {channel_names[idx]}")
    for axis in axes.ravel():
        axis.set_xlim(0, 255)
        axis.grid(True, alpha=0.2)
    fig.savefig(FIG_DIR / "rgb_histograms_before_after_encryption.png", dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(9.4, 8.6), constrained_layout=True)
    scatter_specs = [
        ("original", "horizontal", axes[0, 0], "#1f5aa6"),
        ("original", "vertical", axes[0, 1], "#2f9e44"),
        ("encrypted", "horizontal", axes[1, 0], "#bc412b"),
        ("encrypted", "vertical", axes[1, 1], "#7d5ba6"),
    ]
    for image_name, direction, axis, color in scatter_specs:
        source = gray_original if image_name == "original" else gray_cipher
        left, right = sampled_pairs(source, direction)
        axis.scatter(left, right, s=4, alpha=0.22, color=color)
        axis.set_xlim(0, 255)
        axis.set_ylim(0, 255)
        axis.set_xlabel("Pixel i")
        axis.set_ylabel("Pixel i+1")
        axis.set_title(f"{image_name.capitalize()} {direction}")
    fig.savefig(FIG_DIR / "adjacent_pixel_correlation.png", dpi=220)
    plt.close(fig)

    print("Chaotic synchronization and image encryption reproduction completed.")
    print(f"Seeds: {seeds}")
    print(f"Sync settling time ||e||<=1e-2: {summary['sync_metrics']['settling_time_norm_le_1e-2']:.3f}s")
    print(f"Encrypted horizontal correlation: {correlation_metrics[2][2]:.6f}")


if __name__ == "__main__":
    main()
