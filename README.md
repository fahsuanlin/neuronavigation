# Navigating neuromodulation devices via real-time spatiotemporal guidance

The project of imaging-guided neuro-modulation

## Navigation by camera

Use a camera to track the relative position between the brain and a device in real time. We are using NDI Polaris camera (VEGA ST) to perform the real-time tracking. Some related info:

- Task 1: use CMake to enable Mac computers for this communcation. [This info](https://github.com/PlusToolkit/PlusBuild) is potentially useful.
- Task 2: build a program in Matlab for real-time tracking of of camera markers.

### Related resources
- [PlusSever](https://github.com/PlusToolkit/PlusLib): a server on a computer (Windows system for now) to communicate with the camera
- [Our setup](https://github.com/fahsuanlin/labmanual/wiki/27.-Connec-to-Polaris-Vega-camera) so far (via Dr. Aapo Nummenmma's help).

## Real-time communication wih the EEG system
The NeurOne Tesla (Bittium Oy, Oulu, Finland) has the real-time communcation function via UDP. 

- Task 1: establish the communicaiton between a computer with Matlab and the EEG system via UDP protocol.
- Task 2: establshh the real-time oscillatory phase estimate algorithm.

### Related resources
- [User manual of NeurOne](https://github.com/fahsuanlin/neuronavigation/blob/main/doc/NeurOne%20User%20Manual.pdf). See Appendix 11 for details.
- [Phaseimate](https://github.com/bnplab/phastimate): the toolbox from Tuebingen/CAMH for oscillatory phase estimate
- [A state-space model for phase estimate](https://elifesciences.org/articles/68803). The codes are [here](https://github.com/tne-lab/phase-calculator).
-- Task: Use Matlab Realtime to establish the communcation between the EEG system for real-time oscillatory phase estimate
