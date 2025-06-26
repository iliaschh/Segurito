import { useState } from "react";
import { Toolbar, Container } from "@mui/material";
import TopBar      from "./layout/TopBar";
import Dashboard   from "./pages/Dashboard";
import TeleopPage  from "./pages/TeleopPage";
import MapGallery  from "./pages/MapGallery";    
import VideoGallery from "./pages/VideoGallery";
import "./styles/map.css";

export default function App() {
  const [page, setPage] = useState("dashboard");

  const render = () => ({
    dashboard: <Dashboard   onSelect={setPage} />, 
    teleop:    <TeleopPage />,
    maps:      <MapGallery  onSelect={setPage} />,  
    videos:    <VideoGallery />,
    //logs:    <Logs />,
  }[page]);

  return (
    <>
      <TopBar onSelect={setPage} />
      <Toolbar />
      <Container maxWidth="xl" sx={{ py: 3 }}>
        {render()}
      </Container>
    </>
  );
}
