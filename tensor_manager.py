import os
import pickle
import torch
import numpy as np

def reduce_gt_data(t_target, t_source, data_source):
    indices_to_keep = np.searchsorted(t_source, t_target)
    reduced_data = data_source[indices_to_keep]
    return torch.tensor(reduced_data, dtype=torch.float32)

def match_length(data):
    t_target = data['t'].numpy()
    t_source = np.linspace(t_target[0], t_target[-1], len(data['ang_gt']))

    data['ang_gt'] = reduce_gt_data(t_target, t_source, data['ang_gt'].numpy())
    data['p_gt'] = reduce_gt_data(t_target, t_source, data['p_gt'].numpy())
    data['v_gt'] = reduce_gt_data(t_target, t_source, data['v_gt'].numpy())

    return data

def save_reduced_data(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    reduced_data = match_length(data)
    with open(file_path, 'wb') as f:
        pickle.dump(reduced_data, f)

def reduce_all_pickles(folder_path):
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.p'):
            file_path = os.path.join(folder_path, file_name)
            save_reduced_data(file_path)

folder_path = './data_mobinha' 

reduce_all_pickles(folder_path)
