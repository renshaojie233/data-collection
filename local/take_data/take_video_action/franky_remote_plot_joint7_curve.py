import math
import time

from franky_remote_client import FrankyRemoteClient


def main() -> None:
    amp_deg = 0.1
    amp_gripper = 0.5
    freq_hz = 0.2
    rate_hz = 15.0
    duration_s = 5.0
    gripper_freq_hz = 0.2
    gripper_update_hz = 15.0

    client = FrankyRemoteClient()

    state = client.get_state()
    base = state[6]
    gripper_cmd = state[7]
    gripper_sample = state[7]

    amp = math.radians(amp_deg)
    period = 1.0 / rate_hz
    steps = int(duration_s * rate_hz)
    gripper_every = max(1, int(round(rate_hz / gripper_update_hz)))
    start = time.time()
    next_t = start

    times = []
    actual = []
    target = []
    g_actual = []
    g_target = []

    for step in range(steps):
        now = time.time()
        t = now - start
        include_gripper = (step % gripper_every) == 0
        state = client.get_state(include_gripper=include_gripper)
        if include_gripper:
            gripper_sample = state[7]
        target_state = state[:]
        target_state[6] = base + amp * math.sin(2.0 * math.pi * freq_hz * t)
        if include_gripper:
            gripper_cmd = 0.5 + amp_gripper * math.sin(
                2.0 * math.pi * gripper_freq_hz * t
            )
        target_state[7] = gripper_cmd
        client.send_action(target_state, wait=False)

        times.append(t)
        actual.append(state[6])
        target.append(target_state[6])
        g_actual.append(gripper_sample)
        g_target.append(gripper_cmd)

        next_t += period
        sleep_s = next_t - time.time()
        if sleep_s > 0:
            time.sleep(sleep_s)

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required") from exc

    to_deg = 180.0 / math.pi
    actual_deg = [v * to_deg for v in actual]
    target_deg = [v * to_deg for v in target]

    fig = plt.figure(figsize=(7, 6))
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot(times, actual_deg, label="actual")
    ax1.plot(times, target_deg, label="target", linestyle="--")
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("joint 7 (deg)")
    ax1.set_title(
        "Joint 7 vs time (amp={0} deg, freq={1} Hz, duration={2} s)".format(
            amp_deg, freq_hz, duration_s
        )
    )
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(times, g_actual, label="actual")
    ax2.plot(times, g_target, label="target", linestyle="--")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("gripper (0=open, 1=close)")
    ax2.set_title(
        "Gripper vs time (amp=0.5, freq={0} Hz, duration={1} s)".format(
            gripper_freq_hz, duration_s
        )
    )
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    out_path = "/home/ubuntu/take_data/take_video_action/joint7_curve_remote.png"
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print("saved:", out_path)


if __name__ == "__main__":
    main()
