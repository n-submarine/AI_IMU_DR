import pickle
import os

file_name = '2024-07-16-21-44-01'
# pickle 파일 열기
with open(f'./data_mobinha/{file_name}.p', 'rb') as file:
    data = pickle.load(file)

output_dir = './txts'
os.makedirs(output_dir, exist_ok=True)

output_file_path = os.path.join(output_dir, f'{file_name}.txt')

with open(output_file_path, 'w') as f:
    f.write(str(data))
    # for key, value in data.items():
    #     f.write(f"{key}: {value.tolist() if hasattr(value, 'tolist') else value}\n\n")

print(f"Data has been written to {output_file_path}")