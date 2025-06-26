import { useEffect, useState, useMemo } from "react";
import ROSLIB from "roslib";
import { Chip, Stack, Typography } from "@mui/material";
import PowerIcon     from "@mui/icons-material/PowerSettingsNew";
import ExploreIcon   from "@mui/icons-material/TravelExplore";
import PatrolIcon    from "@mui/icons-material/DirectionsRun";
import PauseIcon     from "@mui/icons-material/PauseCircle";
import { useRos }    from "./RosContext";
import type { ReactElement } from "react";

export default function RobotStatus() {
  const ros = useRos();

  const [powered, setPowered] = useState<boolean>(false);
  const [mode,    setMode]    = useState<string>("unknown");

  useEffect(() => {
    if (!ros) return;

    const powerSub = new ROSLIB.Topic({
      ros,
      name: "/robot/powered",
      messageType: "std_msgs/Bool",
    });
    const modeSub = new ROSLIB.Topic({
      ros,
      name: "/robot/mode",
      messageType: "std_msgs/String",
    });

    powerSub.subscribe((msg: any) => setPowered(msg.data));
    modeSub.subscribe((msg: any) => setMode(msg.data));

    return () => {
      powerSub.unsubscribe();
      modeSub.unsubscribe();
    };
  }, [ros]);

  const powerChip = useMemo(
    () => (
      <Chip
        label={powered ? "Encendido" : "Apagado"}
        color={powered ? "success" : "default"}
        icon={<PowerIcon />}
      />
    ),
    [powered]
  );

  const modeChip = useMemo(() => {
    if (!powered) return null;        

    const map: Record<
        string,
        { label: string; color: any; icon: ReactElement }
        > = {
        idle:       { label: "En reposo",   color: "info",    icon: <PauseIcon /> },
        patrolling: { label: "Patrullando", color: "warning", icon: <PatrolIcon /> },
        exploring:  { label: "Explorando",  color: "primary", icon: <ExploreIcon /> },
        stopped:    { label: "Parado",      color: "error",   icon: <PauseIcon /> },
        };

    const { label, color, icon } = map[mode] || {
      label: mode,
      color: "default",
      icon:  <PauseIcon />,
    };

    return <Chip label={label} color={color as any} icon={icon} />;
  }, [powered, mode]);

  return (
    <Stack spacing={1}>
      <Typography variant="subtitle1">Estado del robot</Typography>
      <Stack direction="row" spacing={1}>
        {powerChip}
        {modeChip}
      </Stack>
    </Stack>
  );
}
