# %% Imports

import stereo
from matplotlib import pyplot as plt
import numpy as np

#%%

# main_data_dir = "/Users/cjt/Academics/COURSES/MEAM 620/EuRoc Data Set/MachineHall01/"

main_data_dir = "../dataset/MachineHall01_reduced/"
dataset = stereo.StereoDataSet(main_data_dir)

#%%

index = 200
stereo_pair_1 = dataset.process_stereo_pair(index)
stereo_pair_2 = dataset.process_stereo_pair(index+1)

#%% Display results

fig = plt.figure()
stereo_pair_1.display_unrectified_images()
fig.suptitle('Stereo Pair 1 - Unrectified')
plt.show()

#%% Display matches

plt.figure()
stereo_pair_1.display_matches()
plt.title('Stereo Pair 1')
plt.show()

plt.figure()
stereo_pair_2.display_matches()
plt.title('Stereo Pair 2')
plt.show()

#%% Display temporal correspondences

temporal_match = stereo.TemporalMatch(stereo_pair_1, stereo_pair_2)

#%%
plt.figure()
temporal_match.display_matches()
plt.title('Temporal Matches')
plt.show()

