import ROSLIB from "roslib";
import { Button, Stack } from "@mui/material";
import { useRos } from "./RosContext";
import { useMemo, useCallback } from "react";

type Props = { onSelect: (page: string) => void };

export default function ExploreButtons({ onSelect }: Props) {
  const ros = useRos();

  // SLAM 
  const startSvc = useMemo(
    () =>
      new ROSLIB.Service({
        ros,
        name: "/map_mode/set_explore",
        serviceType: "std_srvs/Trigger",
      }),
    [ros]
  );

  // Parar robot
  const stopSvc = useMemo(
    () =>
      new ROSLIB.Service({
        ros,
        name: "/map_mode/stop",         
        serviceType: "std_srvs/Trigger",
      }),
    [ros]
  );

  const startExplore = useCallback(() => {
    if (!ros) return;
    startSvc.callService({}, (res: any) => console.log(res.message));
  }, [ros, startSvc]);

  const stopAll = useCallback(() => {
    if (!ros) return;
    stopSvc.callService({}, (res: any) => console.log(res.message));
  }, [ros, stopSvc]);

  const openGallery = () => onSelect("maps");

  return (
    <Stack direction="row" spacing={2} mt={2}>
      <Button variant="contained" color="success" onClick={startExplore}>
        Nuevo mapa
      </Button>

      <Button variant="contained" color="error" onClick={stopAll}>
        Parar
      </Button>

      <Button variant="contained" color="primary" onClick={openGallery}>
        Cargar mapa
      </Button>
    </Stack>
  );
}
