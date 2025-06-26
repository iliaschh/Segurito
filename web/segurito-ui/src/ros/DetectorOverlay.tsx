import { useEffect, useRef } from "react";
import { useRos } from "./RosContext";

declare global { interface Window { ROSLIB: any; } }

const SRC_W = 640;   
const SRC_H = 480;

export default function DetectionOverlay() {
  const ros = useRos();
  const canvas = useRef<HTMLCanvasElement>(null);


  useEffect(() => {
    const video = document.querySelector("img[alt='camera']");
    if (!video || !canvas.current) return;
    const sync = () => {
      const { width, height } = video.getBoundingClientRect();
      canvas.current!.width  = width;
      canvas.current!.height = height;
    };
    sync();
    window.addEventListener("resize", sync);
    return () => window.removeEventListener("resize", sync);
  }, []);


  useEffect(() => {
    if (!ros) return;

    const sub = new window.ROSLIB.Topic({
      ros,
      name: "/people_detections",
      messageType: "vision_msgs/Detection2DArray",
    });

    sub.subscribe((msg: any) => {
      const ctx = canvas.current?.getContext("2d");
      if (!ctx || !canvas.current) return;

      const CW = canvas.current.width;
      const CH = canvas.current.height;

      ctx.clearRect(0, 0, CW, CH);

      msg.detections.forEach((d: any) => {
        if (!canvas.current) return;

        
        

        const ctx = canvas.current.getContext("2d")!;
        const CW = canvas.current.width;
        const CH = canvas.current.height;

        const sX  = CW / SRC_W;              
        const sY  = CH / SRC_H;

        const b   = d.bbox;

        const w   = b.size_x * sX;
        const h   = b.size_y * sY;
        const x   = (b.center.position.x - b.size_x / 2) * sX;
        const y   = (b.center.position.y - b.size_y / 2) * sY;

        ctx.strokeStyle = "#0f0";
        ctx.lineWidth   = 2;
        ctx.strokeRect(x, y, w, h);
        console.log("x: ", x, ", y: ", y, ", w: ", w, ",h: ", h);
        console.log("CW: ", CW, "CH: ", CH);
      });

    });



    return () => sub.unsubscribe();
  }, [ros]);

  return (
    <canvas
      ref={canvas}
      style={{ position: "absolute", inset: 0, pointerEvents: "none" }}
    />
  );
}
