import { useEffect, useRef, useState } from "react";
import { Snackbar, Alert } from "@mui/material";
import { useRos } from "./RosContext";
import alertSound from "../assets/alert.mp3";

declare global { interface Window { ROSLIB:any } }

export default function PersonAlert() {
  const ros  = useRos();
  const [open, setOpen] = useState(false);
  const audioRef = useRef<HTMLAudioElement>(null);

  const triggerAlert = () => {
    setOpen(true);
    audioRef.current?.play().catch(() => {});
    setTimeout(() => setOpen(false), 3000);
  };

  useEffect(() => {
    if (!ros) return;
    const topic = new window.ROSLIB.Topic({
      ros,
      name: "/person_alert",
      messageType: "std_msgs/String",
    });
    topic.subscribe(triggerAlert);
    return () => topic.unsubscribe();
  }, [ros]);

  return (
    <>
      <audio ref={audioRef} src={alertSound} preload="auto" />

      <Snackbar
        open={open}
        anchorOrigin={{ vertical: "top", horizontal: "center" }}  
        sx={{ width: "100%" }}                                  
      >
        <Alert
          severity="warning"
          sx={{
            width: "100%",                
            justifyContent: "center",      
            fontWeight: 600              
          }}
        >
          ¡Persona detectada!
        </Alert>
      </Snackbar>
    </>
  );
}
