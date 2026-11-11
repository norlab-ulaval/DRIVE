# Drive Module

The drive module (`drive.py`) contains the main control loop for the DRIVE protocol. It manages the high-level state machine that governs data collection, geofence creation and safety monitoring during autonomous drive sessions. It uses a robot interface, a command sampling strategy, and a dataset recorder to execute and log drive sessions. The `Drive` class 'run' method should be called in a main loop to execute the appropriate actions based on the current state.

## State Machine

Here is the state machine of the protocol:

![Drive protocol workflow](images/drive_protocol.png)

Each state in this diagram maps to a state class. For example, "Running" is implemented in the `RunningState` class.
