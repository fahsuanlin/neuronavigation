The project of imaging-guided neuro-modulation

# Navigation by camera

Use a camera to track the relative position between the brain and a device in real time.

## Setup of the camera
We are using NDI Polaris camera (VEGA ST) to perform the real-time tracking. Some related info:

- PlusSever: a server on a computer (Windows system for now) to communicate with the camera
-- Task: use CMake to enable Mac computers for this communcation

# Real-time communication wih the EEG system
The NeuroOne Tesla (Bittium Oy, Oulu, Finland) has the real-time communcation function via UDP. 

- [Phaseimate](https://github.com/bnplab/phastimate): the toolbox from Tuebingen/CAMH for oscillatory phase estimate
- [A state-space model for phase estimate](https://elifesciences.org/articles/68803). The codes are [here](https://github.com/tne-lab/phase-calculator).
-- Task: Use Matlab Realtime to establish the communcation between the EEG system for real-time oscillatory phase estimate
