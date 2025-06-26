import { Grid, Paper, Typography, Stack } from "@mui/material";
import CameraWithDetections from "../ros/CameraWithDetections";
import ExploreBtns         from "../ros/ExploreButtons";
import PersonAlert         from "../ros/PersonAlert";
import RobotStatus from "../ros/RobotStatus";   
import MicPublisher from "../ros/MicPublisher";

type Props = { onSelect: (page: string) => void };


export default function Dashboard({ onSelect }: Props) {
  return (
    <>
      <Grid container spacing={3}>
        {}
        <Grid item xs={12} md={8}>
          <Paper elevation={4} sx={{ p: 1 }}>
            <Typography variant="subtitle1" gutterBottom>
              Transmisión en vivo
            </Typography>
            <CameraWithDetections />
          </Paper>
        </Grid>

        {}
        <Grid item xs={12} md={4}>
          <Paper elevation={4} sx={{ p: 2 }}>
            {}
            <RobotStatus />

            <Typography variant="subtitle1" sx={{ mt: 3 }}>
              Acciones
            </Typography>
            <Stack spacing={2} mt={2}>
              <ExploreBtns onSelect={onSelect} />
            </Stack>
          </Paper>
        </Grid>
      </Grid>

      {}
      <PersonAlert />
      {}
      <MicPublisher />
    </>
  );
}
