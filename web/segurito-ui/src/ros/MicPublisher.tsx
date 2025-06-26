import { useRef, useState } from "react";
import ROSLIB from "roslib";
import { Stack, IconButton, Typography } from "@mui/material";
import MicIcon from "@mui/icons-material/Mic";
import MicOffIcon from "@mui/icons-material/MicOff";
import { useRos } from "../ros/RosContext";

const SAMPLE_RATE = 48000;
const CHUNK       = 1024;

export default function MicPublisher() {
  const ros = useRos();
  const topicRef = useRef<ROSLIB.Topic | null>(null);
  const procRef  = useRef<ScriptProcessorNode | null>(null);
  const [active, setActive] = useState(false);

  const toggleMic = async () => {
    if (active) {
      procRef.current?.disconnect();
      procRef.current = null;
      topicRef.current = null;
      setActive(false);
      return;
    }

    if (!ros) return;

    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const ctx    = new AudioContext({ sampleRate: SAMPLE_RATE });
    const src    = ctx.createMediaStreamSource(stream);
    const proc   = ctx.createScriptProcessor(CHUNK, 1, 1);
    procRef.current = proc;

    topicRef.current = new ROSLIB.Topic({
      ros,
      name: "/audio",
      messageType: "audio_common_msgs/AudioData",
    });

    proc.onaudioprocess = (e) => {
      const floatBuf = e.inputBuffer.getChannelData(0);
      const int16    = new Int16Array(floatBuf.length);
      for (let i = 0; i < floatBuf.length; i++) {
        const s = Math.max(-1, Math.min(1, floatBuf[i]));
        int16[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
      }
      topicRef.current?.publish(
        new ROSLIB.Message({ data: [...new Uint8Array(int16.buffer)] })
      );
    };

    src.connect(proc);
    proc.connect(ctx.destination);
    setActive(true);
  };

  return (
    <Stack direction="row" alignItems="center" spacing={1}>
      <IconButton
        color={active ? "error" : "primary"}
        onClick={toggleMic}
        size="large"
      >
        {active ? <MicOffIcon /> : <MicIcon />}
      </IconButton>

      <Typography
        variant="body2"
        sx={{ userSelect: "none", fontWeight: 500, minWidth: 120 }}
      >
        {active ? "Micrófono activo" : "Pulsa para hablar"}
      </Typography>
    </Stack>
  );
}
