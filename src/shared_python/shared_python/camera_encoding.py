import cv2
import numpy as np

JPEG_ENCODE = (
  ('.jpg', (
      int(cv2.IMWRITE_JPEG_QUALITY), 90
    )),
  (cv2.IMREAD_UNCHANGED,)
)


def encode_img(img: np.ndarray, parameters=JPEG_ENCODE) -> bytes | None:
  params = parameters[0]
  res, buff = cv2.imencode(params[0], img, params[1])
  return buff if res else None

def decode_img(buff: bytes, parameters=JPEG_ENCODE) -> np.ndarray:
  params = parameters[1]
  return cv2.imdecode(
    np.frombuffer(buff, dtype=np.uint8),
    params[0]
  )