# To convert binary point cloud to numpy.

import numpy as np
import glob

file_dir = "/home/pratham/Desktop/documents/Tasks/Ati_motors/cv_assignment/"      # Source directory. Where the original ".npy" files are.
save_dir = "/home/pratham/Desktop/documents/Tasks/Ati_motors/pc_save/"            # Destination Directory. Where you want to save the output files.

clouds = glob.glob(file_dir + "*.npy")

for i in clouds:
    data = np.fromfile(i, dtype=np.float32).reshape(-1, 9)
    save_path = save_dir + i.split("/")[-1]

    np.save(save_path, data)
