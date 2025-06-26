import { useEffect, useState } from "react";
import { Grid, Card, CardMedia, CardActionArea, Typography } from "@mui/material";
import { useRos } from "../ros/RosContext";

declare global { interface Window { ROSLIB: any } }
type Props = { onSelect: (page: string) => void }; 

export default function MapGallery({ onSelect }: Props) {
  const ros  = useRos();
  const [maps, setMaps] = useState<string[]>([]);

  //  lista de mapas
  useEffect(() => {
    if (!ros) return;
    const svc = new window.ROSLIB.Service({
      ros,
      name: "/maps/list",
      serviceType: "segurito_interfaces/srv/ListMaps",
    });
    svc.callService({}, (res: any) => setMaps(res.basenames));
  }, [ros]);

  // cargar mapa seleccionado 
  const loadMap = (basename: string) => {
    const srv = new window.ROSLIB.Service({
      ros,
      name: "/map_mode/load_map",         
      serviceType: "nav2_msgs/srv/LoadMap",
    });

    const yamlPath = `file:///home/neverdiedooms/maps/${basename}.yaml`;

    srv.callService({ map_url: yamlPath }, (r: any) => {
      if (r.success) onSelect("dashboard");
      else console.error(r.error_string);
    });
  };


  const host = window.location.hostname;

  return (
    <Grid container spacing={2} p={2}>
      {maps.map(m => (
        <Grid item xs={12} sm={6} md={4} key={m}>
          <Card>
            <CardActionArea onClick={() => loadMap(m)}>
              <CardMedia
                component="img"
                image={`http://${host}:8001/${m}.png`}
                alt={m}
                sx={{ height: 200, objectFit: "cover" }}
              />
            </CardActionArea>
            <Typography align="center" p={1}>{m}</Typography>
          </Card>
        </Grid>
      ))}
    </Grid>
  );
}
