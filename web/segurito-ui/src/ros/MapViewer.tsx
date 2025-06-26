import { useEffect, useRef } from "react";
import { useRos } from "./RosContext";

declare global {
  interface Window {
    ROS2D: any;
    ROSLIB: any;
  }
}

export default function MapViewer() {
  const ros = useRos();
  const divRef       = useRef<HTMLDivElement>(null);
  const viewerRef    = useRef<any>(null);
  const rootRef      = useRef<any>(null);
  const gridRef      = useRef<any>(null);
  const mapSubRef    = useRef<any>(null);

  const robotRef     = useRef<any>(null);
  const poseSubRef   = useRef<any>(null);

  useEffect(() => {
    if (!ros || !divRef.current) return;

    const viewer = new window.ROS2D.Viewer({
      divID: divRef.current.id,
      width: 640,
      height: 480,
    });
    viewerRef.current = viewer;
    rootRef.current   = viewer.scene;

    const mapSub = new window.ROSLIB.Topic({
      ros,
      name: "/map",
      messageType: "nav_msgs/OccupancyGrid",
      compression: "cbor",
      throttle_rate: 0,
      queue_length: 1,
    });
    mapSubRef.current = mapSub;

    mapSub.subscribe((msg: any) => {
      if (gridRef.current) rootRef.current.removeChild(gridRef.current);

      const grid = new window.ROS2D.OccupancyGrid({ message: msg });
      gridRef.current = grid;
      rootRef.current.addChild(grid);

      const { width, height, resolution, origin } = msg.info;
      viewer.scaleToDimensions(width * resolution, height * resolution);
      viewer.shift(origin.position.x, origin.position.y);
    });

    const robotArrow = new window.ROS2D.NavigationArrow({ size: 0.5 });
    robotArrow.visible = false;
    rootRef.current.addChild(robotArrow);
    robotRef.current = robotArrow;

    const poseSub = new window.ROSLIB.Topic({
      ros,
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });
    poseSubRef.current = poseSub;

    poseSub.subscribe((msg: any) => {
      const { position, orientation: q } = msg.pose.pose;

      const yaw =
        (Math.atan2(
          2 * (q.w * q.z + q.x * q.y),
          1 - 2 * (q.y * q.y + q.z * q.z)
        ) *
          -180) /
        Math.PI;

      robotArrow.x = position.x;
      robotArrow.y = -position.y;    
      robotArrow.rotation = yaw;
      robotArrow.visible = true;
    });

    return () => {
      mapSubRef.current?.unsubscribe();
      poseSubRef.current?.unsubscribe();
      rootRef.current?.removeAllChildren();
      mapSubRef.current = gridRef.current = robotRef.current = viewerRef.current = null;
    };
  }, [ros]);

  return (
    <div
      id="map"
      ref={divRef}
      style={{ border: "1px solid #ccc", width: "100%", height: 480 }}
    />
  );
}
