import pickle
import torch

with open('data/Q_model/data.pkl', 'rb') as f:  # caminho para o pkl dentro do ZIP extraído
    state_dict = pickle.load(f)

torch.save(state_dict, 'data/Q_model_clean.pth')  # agora é um .pth puro
