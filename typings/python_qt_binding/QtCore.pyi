from typing import Callable

from PyQt5.QtCore import *  # type: ignore

class pyqtSignal:
    def __init__(self, *types, name: str = ...) -> None: ...
    def emit(self, *args): ...
    def connect(self, receiver: Callable) -> None: ...

