@echo off
rem Build a single tms_bridge.exe (no Python needed on the target PC). Output: dist\tms_bridge.exe
cd /d "%~dp0"
py -m pip install --upgrade pyinstaller
py -m PyInstaller --onefile --windowed --name tms_bridge tms_bridge.py
rem Copy BRIEF.md, PROTOCOL.md, GUIDE.md into dist\docs\ so GET DOC works next to the .exe.
pause
