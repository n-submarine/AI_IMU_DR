import pickle
import torch

# Load the pickle file
with open('./data_mobinha/2024-07-16-21-47-10.p', 'rb') as f:
    data = pickle.load(f)

# Print the length of each tensor
print(f"Length of ang_gt: {len(data['ang_gt'])}")
print(f"Length of p_gt: {len(data['p_gt'])}")
print(f"Length of t: {len(data['t'])}")
print(f"Length of v_gt: {len(data['v_gt'])}")
print(f"Length of u: {len(data['u'])}")
