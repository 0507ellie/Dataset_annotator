import time
import requests
import urllib.request
from urllib.parse import urlparse
from qtpy import QtCore, QtWidgets
from pathlib import Path

from . import GroundingDINOCloud, GroundingDINOLocal, GroundingDINOSAM2Local
from ..labeling.libs.loadingWidget import LoadingExtension
MODEL_DIR = Path.cwd() / "models"

swinb = "groundingdino_swinb_cogcoor"
swint = "groundingdino_swint_ogc"
sam2 = "tiny" # "large" / "small" / "tiny"

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
    
    # Add SAM2
    "encoder_model_path": str( MODEL_DIR / f"sam2.1_hiera_{sam2}.encoder.onnx"),
    "decoder_model_path": str( MODEL_DIR / f"sam2.1_hiera_{sam2}.decoder.onnx"),
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

    pre_shape_type = None
    def __init__(self, mode, shape_type, parent):
        super().__init__()
        self.mode = mode
        self.shape_type = shape_type
        self.parent = parent

        self.loading = LoadingExtension(parent)
        # TODO: 
        if not onnx_file or not Path(onnx_file).is_file():
            self.loading.startLoading('Downloading base model ...')
        else:
            self.loading.startLoading('Init model graph ...')
        QtWidgets.QApplication.processEvents()

    def _loading(self, value):
        self.loading.setProgress(value)

    def run(self):
        if self.mode == "Offline":
            download_urls = [
                (
                    configs["model_path"],
                    f"https://github.com/CVHub520/X-AnyLabeling/releases/download/v1.0.0/{swinb}_quant.onnx"
                    if not onnx_file or not Path(configs["model_path"]).is_file() else ""
                )
            ]
            if self.shape_type == "polygon":
                self.message.emit("Switching to polygon mode")
                download_urls.extend([
                    (
                        configs["encoder_model_path"],
                        f"https://github.com/CVHub520/X-AnyLabeling/releases/download/v2.5.0/sam2.1_hiera_{sam2}.encoder.onnx"
                        if not Path(configs["encoder_model_path"]).is_file() else ""
                    ),
                    (
                        configs["decoder_model_path"],
                        f"https://github.com/CVHub520/X-AnyLabeling/releases/download/v2.5.0/sam2.1_hiera_{sam2}.decoder.onnx"
                        if not Path(configs["decoder_model_path"]).is_file() else ""
                    ),
                ])
            
            def _download(save_path, url):
                try:
                    if len(url) > 40:
                        ellipsis_download_url = (
                            url[:20] + "..." + url[-20:]
                        )
                    else :
                        ellipsis_download_url =  url
                    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
                    
                    self.message.emit(f"Downloading [ {ellipsis_download_url} ] to [ {save_path} ]")

                    # Download and show progress
                    def _progress(count, block_size, total_size):
                        percent = int(count * block_size * 100 / total_size)
                        self.createLoaded.emit(percent)

                    urllib.request.urlretrieve(
                        url, save_path, reporthook=_progress
                    )
                except Exception as e:  # noqa
                    self.message.emit(f"Could not download {ellipsis_download_url}: {e}")
                    
            try:
                for idx, (save_path, download_url) in enumerate(download_urls):
                    if download_url != "":
                        self.loading.loader.text = f"Downloading ({idx+1}/{len(download_urls)}) ..."
                        _download(save_path, download_url)
            except Exception as e:
                self.message.emit(f"Error during downloading: {e}")
                self.loading.loadingFinished()

            try:

                # TODO: 卡住bug
                if InferenceThread.local_gdino.config is None or CreateThread.pre_shape_type != self.shape_type:
                    if self.shape_type == "polygon":
                        gdino = GroundingDINOSAM2Local(configs)
                    else:
                        gdino = GroundingDINOLocal(configs)

                    CreateThread.pre_shape_type = self.shape_type
                    self.createFinished.emit(gdino)
                    self.message.emit("AI engine initialized successfully.")


            except Exception as e:
                self.message.emit(f"Error: Could not initialize model [{onnx_file}]: {e}")
                self.loading.loadingFinished()
                self.quit()
                return 
    
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
        