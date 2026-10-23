"""Deterministic present-observation-only chunk; no simulator access."""
import numpy as np

class MockChunkPolicy:
    def __init__(self):
        self.origin = None

    def predict(self, observation, request_tick, horizon):
        if self.origin is None:
            self.origin = observation['eef10'].copy()
        out = np.tile(self.origin,(horizon,1))
        tick = request_tick+np.arange(horizon)
        out[:,0] += .015*np.sin(tick/30*2)
        return out
