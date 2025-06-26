/* TeleopPage.tsx */
import { Grid, Paper, Typography, Box } from "@mui/material";
import CameraViewer from "../ros/CameraViewer";
import Teleop        from "../ros/Teleop";
import MicPublisher from "../ros/MicPublisher";
export default function TeleopPage() {
  return (
    <Grid container spacing={3}>
      {}
      <Grid item xs={12}>
        <Paper elevation={4} sx={{ p:2 }}>
          <Typography variant="h6" gutterBottom>
            Control manual
          </Typography>

          {}
          <Box sx={{ position:"relative", width:"100%", paddingTop:"56.25%" }}>
            <Box sx={{
              position:"absolute", inset:0,
              display:"flex", justifyContent:"center", alignItems:"center",
            }}>
              <CameraViewer width={1280} height={720}/>
            </Box>
          </Box>
        </Paper>
      </Grid>
      {}
      <MicPublisher />

      {}
      <Grid item xs={12}>
        <Paper elevation={4} sx={{ p:3, textAlign:"center" }}>
          <Box sx={{ fontSize:18, mb:2 }}>
            Usa las flechas o WASD
          </Box>
          <Teleop />  {}
        </Paper>
      </Grid>
    </Grid>
  );
}
