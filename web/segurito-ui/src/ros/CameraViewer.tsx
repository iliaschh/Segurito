/* CameraViewer.tsx */
import { useEffect, useRef } from "react";
import { useRos } from "./RosContext";

declare global {
  interface Window { ROSLIB: any; }
}
declare const ROSLIB: any;

type Props = {
  topic?: string;          // ej. "/image_raw/compressed"
  width?: number | string; // ej. 640 | "100%"
  height?: number | string;// ej. 480 | "100%"
};

export default function CameraViewer({
  topic = "/image_raw/compressed",
  width = 1280,
  height = 720,
}: Props) {
  const ros = useRos();
  const img = useRef<HTMLImageElement>(null);
  const sub = useRef<any>(null);

  useEffect(() => {
    if (!ros) return;

    const cameraSub = new window.ROSLIB.Topic({
      ros,
      name: topic,
      messageType: "sensor_msgs/CompressedImage",
      throttle_rate: 0,
      queue_length : 1,
    });
    sub.current = cameraSub;

    cameraSub.subscribe((msg: any) => {
      if (img.current) {
        img.current.src = "data:image/jpeg;base64," + msg.data;
      }
    });

    return () => sub.current?.unsubscribe();
  }, [ros, topic]);

  return (
    <img
      ref={img}
      alt="camera"
      style={{
        width,
        height,
        objectFit: "cover",
        border: "2px solid #fff",
        boxShadow: "0 0 6px rgba(0,0,0,.5)",
      }}
    />
  );
}