#!/usr/bin/env python3
"""
speaker.py  ──  reproduce AudioData en la tarjeta I²S (MAX98357A)

• Topic de entrada ............ /audio   (audio_common_msgs/AudioData)
• Frecuencia ................. 16 000 Hz mono 16-bit LE
• Dispositivo ALSA ........... se detecta por nombre, o se puede fijar por CLI

Dependencias:
    sudo apt install python3-pyaudio
    sudo apt install ros-humble-audio-common-msgs   #  mensajes .msg
"""

import argparse
import sys
import pyaudio
import numpy as np
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
RATE_HZ   = 48000     # ⇐ debe coincidir con lo que envía el navegador
FORMAT    = pyaudio.paInt16
DEVICE   = None

class Speaker(Node):
    

    def __init__(self, device_index=None):
        super().__init__("speaker_node")

        self.p = pyaudio.PyAudio()

        if device_index is None:                 # buscar por nombre
            device_index = self._guess_i2s()

        info = self.p.get_device_info_by_index(device_index)
        self.get_logger().info(
            f"Usando salida ALSA #{device_index}: {info['name']}")

        self.stream = self.p.open(
            format   = pyaudio.paInt16,
            channels = 2,
            rate     = RATE_HZ,
            output   = True,
            output_device_index = None,
            output_host_api_specific_stream_info = None,
            # name="plughw:0,0" con PyAudio <0.2.14 es complicado,
            # así que queda más simple abrir "default" y dejar que ALSA reencienda.
        )



        self.create_subscription(AudioData, "/audio", self.cb_audio, 10)
    # ------------------------------------------------------------
    def _guess_i2s(self):
        """Busca la tarjeta que incluye 'max98357' o 'i2s' en el nombre."""
        target = ("max98357", "i2s")
        for idx in range(self.p.get_device_count()):
            name = self.p.get_device_info_by_index(idx)["name"].lower()
            if any(t in name for t in target):
                return idx
        self.get_logger().warn(
            "No se encontró la tarjeta I²S; se usará la salida predeterminada")
        return self.p.get_default_output_device_info()["index"]

    # ------------------------------------------------------------
    def cb_audio(self, msg: AudioData) -> None:
        """
        Recibe un mensaje /audio (AudioData) con muestras PCM
        y lo envía al dispositivo ALSA abierto en self.stream.
        """
        # --- 1) bytes → array mono int16 ---------------------------------
        # msg.data llega como lista de uint8 → conviértelo a int16 little-endian
        mono = np.frombuffer(bytes(msg.data), dtype=np.int16)

        # --- 2) si el hardware exige 2 canales, duplica ------------------
        if True:
            # L  L  L ...  →  L L L ...   (estéreo duplicado)
            stereo = np.repeat(mono, 2)          # [L,L, L,L, …]
            self.stream.write(stereo.tobytes())
        else:
            # hardware admite mono directamente
            self.stream.write(mono.tobytes())
    # ------------------------------------------------------------
    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()


# ======================  main  ======================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device", type=int, default=None,
        help="Índice ALSA a usar (salida de 'aplay -l'); "
             "si se omite, se detecta automáticamente.")
    args = parser.parse_args()

    rclpy.init()
    node = Speaker(device_index=args.device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
