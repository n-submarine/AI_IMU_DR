import pickle

# pickle 파일 열기
with open('./data_mobinha/underground_output.p', 'rb') as file:
    data = pickle.load(file)

print(data)
