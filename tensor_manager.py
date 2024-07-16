import os
import pickle
import torch

def reduce_to_match_length(data):
    target_length = len(data['ang_gt'])

    indices_to_keep = torch.linspace(0, len(data['t']) - 1, target_length).long()
    print(indices_to_keep)

    data['t'] = data['t'][indices_to_keep]
    data['u'] = data['u'][indices_to_keep]

    return data

def save_reduced_data(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    reduced_data = reduce_to_match_length(data)
    with open(file_path, 'wb') as f:
        pickle.dump(reduced_data, f)

def revise_all_pickles(folder_path):
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.p'):
            file_path = os.path.join(folder_path, file_name)
            save_reduced_data(file_path)

folder_path = './data_mobinha' 

revise_all_pickles(folder_path)
