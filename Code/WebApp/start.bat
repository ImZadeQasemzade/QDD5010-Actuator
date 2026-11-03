@echo off
rem Serves the QDD console on http://localhost:8765 and opens it. Close this window to stop the server.
rem Uses FreeCAD's bundled Python, because the "python" on PATH is only the Microsoft Store stub on this PC.
start "" http://localhost:8765
"C:\Program Files\FreeCAD 1.1\bin\python.exe" -m http.server 8765 --bind 127.0.0.1 --directory "%~dp0."
