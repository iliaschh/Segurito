import { useEffect, useRef } from "react";
import { useRos } from "./RosContext";
declare const ROSLIB: any;

export default function Teleop() {
  const ros = useRos();
  const cmdRef   = useRef<any>(null);
  const lastRef  = useRef({ lin: 0, ang: 0 });   

  useEffect(() => {
    if (!ros) return;

    cmdRef.current = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
      queue_size: 1,
    });

    const publish = (lin = 0, ang = 0) => {
      lastRef.current = { lin, ang };
      cmdRef.current.publish({
        linear:  { x: lin, y: 0, z: 0 },
        angular: { x: 0,   y: 0, z: ang },
      });
    };

    const LIN = 0.15;   
    const ANG = 1.0;    

    const keyDown = (e: KeyboardEvent) => {
      if (e.repeat) return;
      let lin = 0, ang = 0;

      switch (e.key.toLowerCase()) {
        case "arrowup":
        case "w": lin =  LIN; break;
        case "arrowdown":
        case "s": lin = -LIN; break;
        case "arrowleft":
        case "a": ang =  ANG; break;
        case "arrowright":
        case "d": ang = -ANG; break;
        case " ": publish(0, 0); return;
        default: return;
      }

      const { lin: pLin, ang: pAng } = lastRef.current;
      if (lin && pLin && Math.sign(lin) !== Math.sign(pLin)) publish(0, 0);
      if (ang && pAng && Math.sign(ang) !== Math.sign(pAng)) publish(0, 0);

      publish(lin, ang);
    };

    const keyUp = () => publish(0, 0);

    window.addEventListener("keydown", keyDown);
    window.addEventListener("keyup",   keyUp);

    return () => {
      window.removeEventListener("keydown", keyDown);
      window.removeEventListener("keyup",   keyUp);
      cmdRef.current?.unadvertise?.();
    };
  }, [ros]);

  return null;
}
