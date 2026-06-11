"""A deliberately unmissable test window: proves the container -> WSLg -> Windows desktop pipe.

Run from PowerShell (Docker Desktop WSLg passthrough):
    docker run --rm -d --name wslg_test -e DISPLAY=:0 \
      -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix \
      -v "$PWD/docker:/ci" algae-dt:dev python3 /ci/wslg_test_window.py
If THIS window shows but the twin's windows don't, the problem is app-side; if neither shows,
the WSLg RDP surface (msrdc) is broken -> `wsl --shutdown` + restart Docker Desktop.
Closes itself after 10 minutes so it can never linger headless.
"""
import sys

from PyQt5 import QtCore, QtWidgets

app = QtWidgets.QApplication(sys.argv)
w = QtWidgets.QLabel("WSLg TEST WINDOW\n\nIf you can read this, the display pipe works.\nClose me.")
w.setWindowTitle("WSLg TEST — algae-dt")
w.setAlignment(QtCore.Qt.AlignCenter)
w.setStyleSheet("background:#1f9d3a; color:white; font-size:22px; font-weight:bold; padding:30px;")
w.resize(640, 300)
w.show()
QtCore.QTimer.singleShot(600_000, app.quit)   # self-destruct after 10 min
sys.exit(app.exec_())
