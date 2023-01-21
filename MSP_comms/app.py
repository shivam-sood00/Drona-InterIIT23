# pycalc.py

"""DronaApp is a simple calculator built with Python and PyQt."""

import sys
import threading, plutoComms, time
from functools import partial
from PyQt5.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

ERROR_MSG = "ERROR"
WINDOW_SIZE = 235
DISPLAY_HEIGHT = 35
BUTTON_SIZE = 40


class AppWindow(QMainWindow):
    """DronaApp's main window (GUI or view)."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DronaApp")
        self.setFixedSize(WINDOW_SIZE, WINDOW_SIZE)
        self.generalLayout = QVBoxLayout()
        centralWidget = QWidget(self)
        centralWidget.setLayout(self.generalLayout)
        self.setCentralWidget(centralWidget)
        self._createDisplay()
        self._createButtons()

        self.comms = plutoComms.COMMS(debug=False)
        # toggle = False
        self.readThread = threading.Thread(target=self.comms.read)
        self.writeThread = threading.Thread(target=self.comms.write)
        self.writeThread.start()
        self.readThread.start()

        global KEY_MAPPED_FUNCTION
        KEY_MAPPED_FUNCTION = {
            "q": "comms.takeOff()",
            ",": "comms.arm()",
            ".": "comms.disArm()",
            "/": "comms.boxArm()",
            "e": "comms.land()",
            "w": "comms.increaseHeight()",
            "s": "comms.decreaseHeight()",
            "x": "comms.lowThrottle()",
            "a": "comms.leftYaw()",
            "d": "comms.rightYaw()",
            "u": "comms.forward()",
            "j": "comms.backward()",
            "h": "comms.left()",
            "k": "comms.right()",
            "b": "comms.backFlip()",
            "r": "comms.reset()",
        }

    def _createDisplay(self):
        self.display = QLineEdit()
        self.display.setFixedHeight(DISPLAY_HEIGHT)
        # self.display.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.display.setReadOnly(True)
        # self.generalLayout.addWidget(self.display)

    def _createButtons(self):
        self.buttonMap = {}
        buttonsLayout = QGridLayout()
        keyBoard = [
            # ["7", "8", "9", "/", "C"],
            # ["4", "5", "6", "*", "("],
            # ["1", "2", "3", "-", ")"],
            # ["0", "00", ".", "+", "="],
            ["q", ",", ".", "/", "e"],
            ["w", "s", "x", "a", "d"],
            ["u", "j", "h", "k", "b"],
            # ["r", "j", "h", "k", "b"],
        ]

        for row, keys in enumerate(keyBoard):
            for col, key in enumerate(keys):
                self.buttonMap[key] = QPushButton(key)
                self.buttonMap[key].setFixedSize(BUTTON_SIZE, BUTTON_SIZE)
                buttonsLayout.addWidget(self.buttonMap[key], row, col)

        self.generalLayout.addLayout(buttonsLayout)

    def setDisplayText(self, text):
        """Set the display's text."""
        # print(text)
        # print(KEY_MAPPED_FUNCTION[text])
        eval("self." + KEY_MAPPED_FUNCTION[text])
        text = ""
        self.display.setText(text)
        self.display.setFocus()

    def displayText(self):
        """Get the display's text."""
        return self.display.text()

    def clearDisplay(self):
        """Clear the display."""
        self.setDisplayText("")

    def __del__(self):
        print("destructor calling")
        self.comms.disconnect()
        self.writeThread.join()
        self.readThread.join()


def evaluateExpression(expression):
    """Evaluate an expression (Model)."""
    try:
        result = str(eval(expression, {}, {}))
    except Exception:
        result = ERROR_MSG
    return result


class DronaApp:
    """DronaApp's controller class."""

    def __init__(self, model, view):
        self._evaluate = model
        self._view = view
        self._connectSignalsAndSlots()

    def _calculateResult(self):
        result = self._evaluate(expression=self._view.displayText())
        self._view.setDisplayText(result)

    def _buildExpression(self, subExpression):
        if self._view.displayText() == ERROR_MSG:
            self._view.clearDisplay()
        expression = self._view.displayText() + subExpression
        self._view.setDisplayText(expression)

    def _connectSignalsAndSlots(self):
        for keySymbol, button in self._view.buttonMap.items():
            if keySymbol not in {"=", "C"}:
                button.clicked.connect(partial(self._buildExpression, keySymbol))
        # self._view.buttonMap["="].clicked.connect(self._calculateResult)
        # self._view.display.returnPressed.connect(self._calculateResult)
        # self._view.buttonMap["C"].clicked.connect(self._view.clearDisplay)


def main():
    """DronaApp's main function."""

    dronaApp = QApplication([])
    dronaAppWindow = AppWindow()
    dronaAppWindow.show()
    DronaApp(model=evaluateExpression, view=dronaAppWindow)

    # comms.disconnect()
    # writeThread.join()
    # readThread.join()

    sys.exit(dronaApp.exec())


if __name__ == "__main__":
    main()
