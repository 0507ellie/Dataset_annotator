import requests
from qtpy import QtCore 

from . import GroundingDINOCloud, GroundingDINOLocal

swinb = "groundingdino_swinb_cogcoor"
swint = "groundingdino_swint_ogc"
onnx_file = f"./models/groundingdino_swint_ogc_quant.onnx"
model_type = swinb if swinb in onnx_file else swint
configs = {
    "model_path": onnx_file,
    "model_type": model_type,
    "device": "cpu",
    "box_threshold": 0.3,
    "text_threshold": 0.25,
    "input_width": 1200,
    "input_height": 800,
} # TODO: thres maybe use slider
# gdino = GroundingDINOLocal(configs)
gdino = GroundingDINOCloud("1253043b11a9c1e4c4da6d886a2ea847")
        
class InferenceThread(QtCore.QThread):
    inferenceFinished = QtCore.Signal(dict)

    def __init__(self, prompts):
        super().__init__()
        self.prompts = prompts
        
    def run(self):
        try:
            data = gdino.inference(self.prompts)
            self.inferenceFinished.emit(data)
        except requests.exceptions.RequestException as e:
            self.inferenceFinished.emit({"error": str(e)})
            
            
        