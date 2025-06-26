# segurito_exploration_control/people_detector.py
#
# – Sin cv_bridge
# – Roboflow Inference v2   (http://localhost:9001/infer/object_detection)
# – BoundingBox2D.center.position.{x,y}

import base64
import rclpy, requests
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D

BASE_URL   = "http://localhost:9001"
API_KEY    = "3lSt0CGh7u6e2zLG9R3q"
PROJECT_ID = "people-detection-o4rdr"
MODEL_VER  = 9
TASK       = "object_detection"


class PeopleDetector(Node):
    def __init__(self):
        super().__init__("people_detector")

        self.pub = self.create_publisher(Detection2DArray,
                                         "/people_detections", 10)

        self.create_subscription(CompressedImage,
                                 "/image_raw/compressed",
                                 self.cb_compressed, 10)

    def cb_compressed(self, msg: CompressedImage):
        jpeg_b64 = base64.b64encode(msg.data).decode()
        self.call_inference(jpeg_b64, msg.header)

    def call_inference(self, jpeg_b64: str, header):
        payload = {
            "model_id": f"{PROJECT_ID}/{MODEL_VER}",
            "image":   {"type": "base64", "value": jpeg_b64},
            "api_key": API_KEY,
        }

        try:
            res = requests.post(f"{BASE_URL}/infer/{TASK}",
                                json=payload, timeout=2)
            res.raise_for_status()
            preds = res.json().get("predictions", [])
        except Exception as e:
            self.get_logger().warn(f"Roboflow err: {e}")
            return

        out = Detection2DArray()
        out.header = header

        for p in preds:
            box = BoundingBox2D()
            box.center.position.x = float(p["x"])
            box.center.position.y = float(p["y"])
            box.center.theta      = 0.0
            box.size_x            = float(p["width"])
            box.size_y            = float(p["height"])

            out.detections.append(Detection2D(bbox=box))

        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PeopleDetector())


if __name__ == "__main__":
    main()
