declare const ROSLIB: any;        

import { createContext, useContext } from "react";

export const ros = new ROSLIB.Ros({
  url: `ws://${location.hostname}:9090`,
});

ros.on('connection', () => {
  console.log('[RosContext] conectado a rosbridge');
  (window as any).ros = ros;        // debug para que se pueda usar en la consola
});

export const RosCtx = createContext(ros);
export const useRos = () => useContext(RosCtx);
