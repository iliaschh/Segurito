import {
  AppBar,
  Toolbar,
  IconButton,
  Typography,
  Drawer,
  List,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Divider,
} from "@mui/material";
import MenuIcon         from "@mui/icons-material/Menu";
import MapIcon          from "@mui/icons-material/Map";
import VideogameIcon    from "@mui/icons-material/VideogameAsset";
import VideoLibraryIcon from "@mui/icons-material/VideoLibrary";
import BugReportIcon    from "@mui/icons-material/BugReport";
import { useState } from "react";

export default function TopBar({
  onSelect,
}: {
  onSelect: (page: string) => void;
}) {
  const [open, setOpen] = useState(false);
  const toggleDrawer = () => setOpen((o) => !o);

  return (
    <>
      <AppBar position="fixed" color="primary">
        <Toolbar>
          <IconButton edge="start" size="large" color="inherit" onClick={toggleDrawer}>
            <MenuIcon />
          </IconButton>
          <Typography variant="h6" component="div">
            Segurito — Panel
          </Typography>
        </Toolbar>
      </AppBar>

      <Drawer anchor="left" open={open} onClose={toggleDrawer}>
        <List sx={{ width: 240 }}>
          {/*  DASHBOARD  */}
          <ListItemButton
            onClick={() => {
              onSelect("dashboard");
              toggleDrawer();
            }}
          >
            <ListItemIcon>
              <MapIcon />
            </ListItemIcon>
            <ListItemText primary="Dashboard" />
          </ListItemButton>

          {/*  TELE-OP  */}
          <ListItemButton
            onClick={() => {
              onSelect("teleop");
              toggleDrawer();
            }}
          >
            <ListItemIcon>
              <VideogameIcon />
            </ListItemIcon>
            <ListItemText primary="Tele-op" />
          </ListItemButton>

          {/*  VÍDEOS  */}
          <ListItemButton
            onClick={() => {
              onSelect("videos");
              toggleDrawer();
            }}
          >
            <ListItemIcon>
              <VideoLibraryIcon />
            </ListItemIcon>
            <ListItemText primary="Vídeos" />
          </ListItemButton>

          <Divider />

          {/*  LOGS  (no existe de momento :( )*/}
          <ListItemButton
            onClick={() => {
              onSelect("logs");
              toggleDrawer();
            }}
          >
            <ListItemIcon>
              <BugReportIcon />
            </ListItemIcon>
            <ListItemText primary="Logs" />
          </ListItemButton>
        </List>
      </Drawer>
    </>
  );
}
