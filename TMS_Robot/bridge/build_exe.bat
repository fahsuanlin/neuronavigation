@echo off
rem Build a single tms_bridge.exe (no Python needed on the target PC). Output: dist\tms_bridge.exe
cd /d "%~dp0"
py -m pip install --upgrade pyinstaller
py -m PyInstaller --onefile --windowed --name tms_bridge tms_bridge.py
rem For GET DOC next to the .exe: copy ..\README.md, ..\doc\PROTOCOL.md and ..\doc\WINDOWS_BRIDGE.md into dist\docs\.
pause
