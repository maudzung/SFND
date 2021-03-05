from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    f = open("./nis_data.txt", "r")
    lines = f.readlines()
    nis_data_dict = defaultdict(list)
    for line in lines[2:]:
        nis_type, nis_value = line[:-1].split(" ")
        nis_data_dict[nis_type].append(float(nis_value))

    n_types = len(nis_data_dict)
    fig, axes = plt.subplots(nrows=1, ncols=n_types, figsize=(10, 5))
    axes = axes.ravel()

    plt_idx = 0
    nis_thresh_dict = {
        "nis_radar": 7.815,
        "nis_laser": 5.991
    }
    for nis_type, nis_vals in nis_data_dict.items():
        nis_thresh = nis_thresh_dict[nis_type]

        nis_vals_np = np.array(nis_vals)
        nis_val_under_thres = nis_vals_np[nis_vals_np < nis_thresh]

        nis_under_thresh_percent = len(nis_val_under_thres) / len(nis_vals) * 100.

        # Plot nis data
        axes[plt_idx].plot(nis_vals, label="NIS")

        # Plot a straight line
        x_line = [0, len(nis_vals) - 1]
        y_line = [nis_thresh, nis_thresh]

        axes[plt_idx].plot(x_line, y_line, label="95%")

        # Set title
        axes[plt_idx].set_title("{} - under thresh: {:.1f}%".format(nis_type, nis_under_thresh_percent))

        plt_idx += 1

    fig.suptitle("Check consistent with NIS")
    plt.savefig("./nis_check_consistance.png")
    plt.show()

