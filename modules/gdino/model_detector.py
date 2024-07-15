import time
import requests
import urllib.request
from urllib.parse import urlparse
from qtpy import QtCore, QtWidgets
from pathlib import Path

from . import GroundingDINOCloud, GroundingDINOLocal
from modules.labeling.libs.loadingWidget import LoadingExtension
MODEL_DIR = Path.cwd() / "models"

swinb = "groundingdino_swinb_cogcoor"
swint = "groundingdino_swint_ogc"

onnx_file = str(MODEL_DIR / f"{swinb}_quant.onnx")
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

class GenericWorker(QtCore.QObject):
    finished = QtCore.Signal()

    def __init__(self, func, *args, **kwargs):
        super().__init__()
        self.func = func
        self.args = args
        self.kwargs = kwargs
       
    @QtCore.Slot()
    def run(self):
        self.func(*self.args, **self.kwargs)
        self.finished.emit()

class CreateThread(QtCore.QThread):
    message = QtCore.Signal(str)
    createLoaded = QtCore.Signal(int)
    createFinished = QtCore.Signal(object)

    def __init__(self, mode, parent):
        super().__init__()
        self.mode = mode
        self.parent = parent

        self.loading = LoadingExtension(parent)
        if not onnx_file or not Path(onnx_file).is_file():
            self.loading.startLoading('Downloading ...')
        else:
            self.loading.startLoading('Init model graph ...')
        QtWidgets.QApplication.processEvents()

    def _loading(self, value):
        self.loading.setProgress(value)

    def run(self):
        if self.mode == "Offline":
            if not onnx_file or not Path(onnx_file).is_file():
                download_url = f"https://github.com/CVHub520/X-AnyLabeling/releases/download/v1.0.0/{swinb}_quant.onnx"
                if len(download_url) > 40:
                    ellipsis_download_url = (
                        download_url[:20] + "..." + download_url[-20:]
                    )
                else :
                    ellipsis_download_url =  download_url
                Path(onnx_file).parent.mkdir(parents=True, exist_ok=True)

                try:
                    self.message.emit(f"Downloading [ {ellipsis_download_url} ] to [ {onnx_file} ]")

                    # Download and show progress
                    def _progress(count, block_size, total_size):
                        percent = int(count * block_size * 100 / total_size)
                        self.createLoaded.emit(percent)

                    urllib.request.urlretrieve(
                        download_url, onnx_file, reporthook=_progress
                    )
                except Exception as e:  # noqa
                    self.loading.loadingFinished()
                    self.message.emit(f"Could not download {ellipsis_download_url}: {e}")
            try:

                # TODO: 卡住bug
                if InferenceThread.local_gdino.config == None:
                    gdino = GroundingDINOLocal(configs)
                    self.createFinished.emit(gdino)

            except Exception as e:
                self.message.emit(f"Could not initialize [ {onnx_file} ] model.")
                self.loading.loadingFinished()
                self.quit()
                return 
            
        self.message.emit(f"Initialize AI engine success.")
        self.loading.loadingFinished()
        self.quit()

class InferenceThread(QtCore.QThread):
    message = QtCore.Signal(str)
    inferenceFinished = QtCore.Signal(dict)
    local_gdino = GroundingDINOLocal()
    cloud_gdino = GroundingDINOCloud("1253043b11a9c1e4c4da6d886a2ea847")  

    def __init__(self, mode, prompts, parent):
        super().__init__()
        self.mode = mode
        self.prompts = prompts
        self.loading = LoadingExtension(parent)
        self.loading.startLoading("Waiting for infer ...")

        self.gdino = self.local_gdino if self.mode == "Offline" else self.cloud_gdino 

    @staticmethod
    def _loading(model):
        InferenceThread.local_gdino = model

    def run(self):
        try:
            start_time = time.time()
            data = self.gdino.inference(self.prompts)
            end_time = time.time()
            self.message.emit("Auto-model inference time: {:.3f}s".format(end_time - start_time))
            self.inferenceFinished.emit(data)
        except requests.exceptions.RequestException as e:
            self.inferenceFinished.emit({"error": str(e)})
        self.loading.loadingFinished()    
        self.quit()   
        