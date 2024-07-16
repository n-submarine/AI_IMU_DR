import pickle
import torch

def reduce_to_match_length(data):
    target_length = len(data['ang_gt'])

    indices_to_keep = torch.linspace(0, len(data['t']) - 1, target_length).long()
    print(indices_to_keep)

    data['t'] = data['t'][indices_to_keep]
    data['u'] = data['u'][indices_to_keep]

    return data

def save_reduced_data(input_file, output_file):
    with open(input_file, 'rb') as f:
        data = pickle.load(f)
    reduced_data = reduce_to_match_length(data)
    with open(output_file, 'wb') as f:
        pickle.dump(reduced_data, f)

save_reduced_data('./songdo_230923.p', './data_mobinha/songdo_230923_revised.p')
save_reduced_data('./underground_output.p', './data_mobinha/underground_output_revised.p')
