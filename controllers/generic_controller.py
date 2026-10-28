"""Independent actuator-command adapters; no RFT or simulator dependency."""

import numpy as np


class ActuatorCommandController:
    """Hold an existing actuator command, optionally adding named-index sine motion.

    This is an integration demonstration, not an automatic walking policy.
    The runner resolves model-specific actuator names to indices.
    """

    def __init__(self, initial_command, *, indices=(), amplitude=0.0, frequency=0.5):
        self.initial = np.asarray(initial_command, dtype=float).copy()
        self.indices = np.asarray(indices, dtype=int)
        self.amplitude = float(amplitude)
        self.frequency = float(frequency)

    def reset(self):
        pass

    def update(self, observation, time):
        command = self.initial.copy()
        command[self.indices] += self.amplitude * np.sin(2*np.pi*self.frequency*time)
        return command
