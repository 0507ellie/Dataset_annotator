import requests
import os
import urllib.request
from urllib.parse import urlparse
from qtpy import QtCore, QtWidgets
from pathlib import Path

from . import GroundingDINOCloud, GroundingDINOLocal
from modules.labeling.libs.loadingWidget import LoadingExtension
model_dir = Path(__file__).resolve().parents[2] / "models"

swinb = "groundingdino_swinb_cogcoor"
swint = "groundingdino_swint_ogc"
onnx_file = f"{model_dir}/groundingdino_swint_ogc_quant.onnx"
model_type = swinb if swinb in onnx_file else swint
configs = {
    "model_path": onnx_file,
    "model_type": model_type,
    "device": "cpu",
    "box_threshold": 0.3,
    "text_threshold": 0.25,
    "input_width": 1200,
    "input_height": 800,
} 
local_gdino = GroundingDINOLocal(configs)
cloud_gdino = GroundingDINOCloud("1253043b11a9c1e4c4da6d886a2ea847")
        
class InferenceThread(QtCore.QThread):
    inferenceFinished = QtCore.Signal(dict)
    downloadPercent = 0
    
    def __init__(self, mode, prompts, parent):
        super().__init__()
        self.mode = mode
        self.prompts = prompts
        self.parent = parent
        
        # TODO: temp
        if self.mode == "Offline":
            if not onnx_file or not os.path.isfile(onnx_file):
                download_url = "https://github.com/CVHub520/X-AnyLabeling/releases/download/v1.0.0/groundingdino_swint_ogc_quant.onnx"
                Path(onnx_file).parent.mkdir(parents=True, exist_ok=True)
                loading = LoadingExtension(self.parent)
                loading.startLoading('Downloading ...')
                try:
                    # Download and show progress
                    def _progress(count, block_size, total_size):
                        percent = int(count * block_size * 100 / total_size)
                        if InferenceThread.downloadPercent != percent:
                            InferenceThread.downloadPercent = percent
                            QtWidgets.QApplication.processEvents()
                        
                        loading.setProgress(percent)

                    urllib.request.urlretrieve(
                        download_url, onnx_file, reporthook=_progress
                    )
                    loading.loadingFinished()
                    InferenceThread.downloadPercent = 0
                    local_gdino.__init__(configs)
                except Exception as e:  # noqa
                    loading.loadingFinished()
                    
        self.gdino = local_gdino if self.mode=="Offline" else cloud_gdino
        
    def run(self):
        try:
            data = self.gdino.inference(self.prompts)
            self.inferenceFinished.emit(data)
        except requests.exceptions.RequestException as e:
            self.inferenceFinished.emit({"error": str(e)})
            
            
        