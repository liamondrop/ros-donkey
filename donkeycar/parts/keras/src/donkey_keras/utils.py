import cv2
import numpy as np


def image_deserialize(data):
    """
    Deserialize a compressed image buffer to a numpy uint8 array
    """
    buf = np.ndarray(shape=(1, len(data)), dtype=np.uint8, buffer=data)
    return cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
