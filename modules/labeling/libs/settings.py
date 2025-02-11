import json
import os
from pathlib import Path
from PyQt5.QtCore import QSize, QPoint, QByteArray
from PyQt5.QtGui import QColor

from modules.labeling.libs.labelFile import LabelFileFormat

class Settings(object):
    def __init__(self):
        home = Path.cwd()
        self.data = {}
        self.path = str(home / '.labelingToolSettings.json')

    def __setitem__(self, key, value):
        self.data[key] = value

    def __getitem__(self, key):
        return self.data[key]

    def get(self, key, default=None):
        return self.data.get(key, default)

    def save(self):
        if self.path:
            try:
                serialized_data = self._serialize(self.data)
                with open(self.path, 'w') as f:
                    json.dump(serialized_data, f, indent=4)
                return True
            except Exception as e:
                print(f'Saving settings failed: {e}')
        return False

    def load(self):
        try:
            if os.path.exists(self.path):
                with open(self.path, 'r') as f:
                    serialized_data = json.load(f)
                    self.data = self._deserialize(serialized_data)
                return True
        except Exception as e:
            print(f'Loading settings failed: {e}')
        return False

    def reset(self):
        if os.path.exists(self.path):
            os.remove(self.path)
            print(f'Removed settings file {self.path}')
        self.data = {}
        self.path = None

    def _serialize(self, data):
        """Convert PyQt5 objects to serializable format"""
        if isinstance(data, (QSize, QPoint, QByteArray, QColor, LabelFileFormat)):
            if isinstance(data, QSize):
                return {'__type__': 'QSize', 'width': data.width(), 'height': data.height()}
            elif isinstance(data, QPoint):
                return {'__type__': 'QPoint', 'x': data.x(), 'y': data.y()}
            elif isinstance(data, QByteArray):
                return {'__type__': 'QByteArray', 'data': data.toHex().data().decode()}  # Fix QByteArray serialization
            elif isinstance(data, QColor):
                return {'__type__': 'QColor', 'rgba': data.rgba()}
            elif isinstance(data, LabelFileFormat):
                return {'__type__': 'LabelFileFormat', 'value': data.name}  # Serialize enum as string
        elif isinstance(data, dict):
            return {key: self._serialize(value) for key, value in data.items()}
        elif isinstance(data, (list, tuple)):
            return [self._serialize(item) for item in data]
        else:
            return data

    def _deserialize(self, data):
        """Convert serialized data back to PyQt5 objects"""
        if isinstance(data, dict):
            if '__type__' in data:
                if data['__type__'] == 'QSize':
                    return QSize(data['width'], data['height'])
                elif data['__type__'] == 'QPoint':
                    return QPoint(data['x'], data['y'])
                elif data['__type__'] == 'QByteArray':
                    return QByteArray.fromHex(bytes(data['data'], 'utf-8'))
                elif data['__type__'] == 'QColor':
                    return QColor(data['rgba'])
                elif data['__type__'] == 'LabelFileFormat':  # Deserialize enum from string
                    return LabelFileFormat[data['value']]
            else:
                return {key: self._deserialize(value) for key, value in data.items()}
        elif isinstance(data, (list, tuple)):
            return [self._deserialize(item) for item in data]
        else:
            return data