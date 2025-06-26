import { useEffect, useState, useCallback } from "react";
import {
  Grid,
  Card,
  CardMedia,
  CardActionArea,
  Typography,
} from "@mui/material";
import { useRos } from "../ros/RosContext";

declare global {
  interface Window {
    ROSLIB: any;
  }
}

export default function VideoGallery() {
  const ros = useRos();
  const [clips, setClips] = useState<string[]>([]);
  const [playing, setPlaying] = useState<string | null>(null); 

  const fetchClips = useCallback(() => {
    if (!ros) return;
    const svc = new window.ROSLIB.Service({
      ros,
      name: "/videos/list",
      serviceType: "segurito_interfaces/srv/ListVideos",
    });

    svc.callService(
      {},
      (res: any) => {
        setClips(res.basenames);
      },
      (err: any) => console.error("Error servicio:", err)
    );
  }, [ros]);

  useEffect(() => {
    if (!ros) return;

    if (ros.isConnected) {
      fetchClips();
    } else {
      const onConn = () => {
        ros.off("connection", onConn);
        fetchClips();
      };
      ros.on("connection", onConn);
      return () => ros.off("connection", onConn);
    }
  }, [ros, fetchClips]);


  const host = window.location.hostname;
  const getVideoURL = (name: string) => `http://${host}:8002/${name}.mp4`;
  const getThumbURL = (name: string) => `http://${host}:8002/${name}.jpg`; 

  return (
    <Grid container spacing={2} p={2}>
      {clips.map((name) => {
        const videoURL = getVideoURL(name);
        const thumbURL = getThumbURL(name);

        const isPlaying = playing === name;

        return (
          <Grid item xs={12} sm={6} md={4} key={name}>
            <Card>
              <CardActionArea
                onClick={() =>
                  setPlaying((prev) => (prev === name ? null : name))
                }
              >
                {isPlaying ? (
                  <CardMedia
                    component="video"
                    src={videoURL}
                    controls          
                    autoPlay       
                    poster={thumbURL} 
                    sx={{ height: 180, objectFit: "cover" }}
                  />
                ) : (
                 // miniatura
                  <CardMedia
                    component="img"
                    image={thumbURL}
                    alt={name}
                    sx={{ height: 180, objectFit: "cover" }}
                  />
                )}
              </CardActionArea>
              <Typography align="center" p={1}>
                {name}
              </Typography>
            </Card>
          </Grid>
        );
      })}

      {!clips.length && (
        <Typography variant="h6" sx={{ m: 4, opacity: 0.6 }}>
          (No hay vídeos todavía)
        </Typography>
      )}
    </Grid>
  );
}
