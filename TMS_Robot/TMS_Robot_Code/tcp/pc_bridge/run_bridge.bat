@echo off
rem TMS Robot Bridge: Mac (Polaris terminal) <-> LabVIEW robot. Double-click to start.
rem Edit the line below: --robot sim (built-in simulator) or --robot 172.22.11.2:6340 (myRIO VI TCP loop).
cd /d "%~dp0"
start "TMS Robot Bridge" pythonw tms_bridge.py --robot 172.22.11.2:6340
