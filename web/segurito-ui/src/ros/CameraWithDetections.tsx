import CameraViewer from "./CameraViewer";
import DetectionOverlay from "./DetectorOverlay";
export default function CameraWithDetections(){
  return(
    <div style={{position:"relative", width:"100%"}}>
      <CameraViewer width="100%" height={720}/>
      <DetectionOverlay/>
    </div>
  );
}