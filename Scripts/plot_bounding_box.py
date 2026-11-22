"""Edit log_paths and selected_aspects below, then run this file. Requires matplotlib."""

import math
import re
from pathlib import Path

import matplotlib.pyplot as plt


project_dir = Path(__file__).resolve().parent.parent
# Replace with [project_dir / "Boats.mov_BoundingBox.log"] to select one log.
#
log_paths = sorted(project_dir.glob("*_BoundingBox.log"))

# Available aspects: centroid, probabilities, validity, box_size.
selected_aspects = [
    "centroid",
    "probabilities",
    "validity",
    # "box_size",
]


def read_log(log_path):
    records = []
    with Path(log_path).open(encoding="utf-8") as log_file:
        for line in log_file:
            if not line.strip():
                continue
            points = {
                name: tuple(float(value) for value in coordinates.split(","))
                for name, coordinates in re.findall(r"(\w+): \(([^)]+)\)", line)
            }
            probabilities = re.search(r"IMM mu: \{([^}]+)\}", line).group(1)
            records.append({
                "frame": int(re.search(r"Frame Number (\d+):", line).group(1)),
                "centroid": points["centroid"],
                "probabilities": [float(value) for value in probabilities.split(",")],
                "validity": re.search(r"valid: (true|false)", line).group(1) == "true",
                "box_size": (
                    math.dist(points["ul"], points["ur"]),
                    math.dist(points["ul"], points["ll"]),
                ),
            })
    return records


def plot_log(log_path, aspects):
    records = read_log(log_path)
    frames = [record["frame"] for record in records]
    figure, axes = plt.subplots(
        len(aspects), 1, sharex=True, squeeze=False,
        figsize=(12, 3 * len(aspects)), layout="constrained",
    )
    figure.suptitle(Path(log_path).name)

    for axis, aspect in zip(axes[:, 0], aspects):
        if aspect == "centroid":
            axis.plot(frames, [record["centroid"][0] for record in records], label="x")
            axis.plot(frames, [record["centroid"][1] for record in records], label="y")
            axis.set_ylabel("Position (pixels)")
            axis.legend()
        elif aspect == "probabilities":

            axis.plot(
                frames,
                [record["probabilities"][0] for record in records],
                label=f"CA Model",
            )

            axis.plot(
                frames,
                [record["probabilities"][1] for record in records],
                label=f"CV Model",
            )

            axis.set_ylabel("IMM model probability")
            axis.set_ylim(0, 1)
            axis.legend()
        elif aspect == "validity":
            axis.step(frames, [record["validity"] for record in records], where="post")
            axis.set_yticks([0, 1], ["Invalid", "Valid"])
            axis.set_ylim(-0.1, 1.1)
            axis.set_ylabel("Track validity")
        elif aspect == "box_size":
            axis.plot(frames, [record["box_size"][0] for record in records], label="width")
            axis.plot(frames, [record["box_size"][1] for record in records], label="height")
            axis.set_ylabel("Size (pixels)")
            axis.legend()
        else:
            raise ValueError(f"Unknown aspect: {aspect}")

        axis.set_title(aspect.replace("_", " ").capitalize())
        axis.grid(True, alpha=0.3)

    axes[-1, 0].set_xlabel("Frame number")
    return figure


def main():
    for log_path in log_paths:
        plot_log(log_path, selected_aspects)
    plt.show()


if __name__ == "__main__":
    main()
