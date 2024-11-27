# rl_agent.py

import torch
import torch.nn as nn
import torch.optim as optim
import os

class RLAgent(nn.Module):
    def __init__(self, input_size, output_size, lr=1e-3):
        super(RLAgent, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(128, 64)
        self.relu2 = nn.ReLU()
        self.output_layer = nn.Linear(64, output_size)

        self.optimizer = optim.Adam(self.parameters(), lr=lr)
        self.criterion = nn.MSELoss()

    def forward(self, x):
        x = self.relu1(self.fc1(x))
        x = self.relu2(self.fc2(x))
        return self.output_layer(x)

    def save_model(self, path):
        torch.save(self.state_dict(), path)

    def load_model(self, path):
        if os.path.exists(path):
            self.load_state_dict(torch.load(path))
            self.eval()
        else:
            print(f"Model file not found at {path}. Starting with a new model.")
