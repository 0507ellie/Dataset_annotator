import numpy as np
from typing import *

from . import converter
from sensor_msgs.msg import Image, CompressedImage


def imgify(
    message: Union[Image, CompressedImage],
    color_space: Optional[str] = None,
) -> np.ndarray:
    """Convert ROS message to OpenCV image.

    Args:
        message: CompressedImage or Image message.
        color_space: Color space of output image. Default is None.

    Returns:
        OpenCV image as a numpy array.

    Raises:
        ValueError: If the input message type is not supported.
    """
    if converter.is_compressed(message):
        return converter.compressed_imgmsg_to_cvimage(message, color_space)
    elif isinstance(message, Image):
        return converter.imgmsg_to_cvimage(message, color_space)
    else:
        raise ValueError(f"Unsupported message type: {type(message)}")

def msgify(
    message_type: Union[Image, CompressedImage], 
    img_obj: np.ndarray, 
    quality: int = 100
) -> Union[Image, CompressedImage]:
    """Convert OpenCV image to ROS message.

    Args:
        message_type: ROS message type, either Image or CompressedImage.
        img_obj: OpenCV image object (numpy array).
        quality: Compression quality for CompressedImage. Default is 100.

    Returns:
        ROS Image or CompressedImage message.

    Raises:
        ValueError: If the input message_type is not supported or quality is invalid.
    """
    if not (0 <= quality <= 100):
        raise ValueError(f"Invalid quality value: {quality}. Must be between 0 and 100.")

    if issubclass(message_type, CompressedImage):
        if img_obj.dtype == np.uint16:
            dst_format = 'png'
        else:
            dst_format = 'jpg'
        return converter.cvimage_to_compressed_imgmsg(img_obj, dst_format=dst_format, quality=quality)
    elif issubclass(message_type, Image):
        return converter.cvimage_to_imgmsg(img_obj)
    else:
        raise ValueError(f"Unsupported message type: {message_type}")