import requests
from qtpy import QtCore 

from . import GroundingDINOAPIWrapper

class InferenceThread(QtCore.QThread):
    inferenceFinished = QtCore.Signal(dict)

    def __init__(self, prompts):
        super().__init__()
        self.gdino = GroundingDINOAPIWrapper("1253043b11a9c1e4c4da6d886a2ea847")
        self.prompts = prompts
        
    def run(self):
        try:
            data = self.gdino.inference(self.prompts)
            self.inferenceFinished.emit(data)
        except requests.exceptions.RequestException as e:
            self.inferenceFinished.emit({"error": str(e)})