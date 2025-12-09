#!/usr/bin/env python3
"""
Whisper Voice Recognition Node for ROS 2

Purpose: Real-time speech-to-text using OpenAI Whisper
Environment: ROS 2 Humble, Whisper, PyAudio
Usage: ros2 run voice_control whisper_voice_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import queue
import threading


class WhisperVoiceNode(Node):
    """ROS 2 node for continuous speech recognition"""

    def __init__(self):
        super().__init__('whisper_voice_node')

        # Parameters
        self.declare_parameter('model_size', 'base')  # tiny, base, small, medium, large
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)

        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded')

        # Audio buffer
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.chunk_duration = 3.0  # 3-second chunks
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)

        # Publisher
        self.transcription_pub = self.create_publisher(
            String,
            '/voice/transcription',
            10
        )

        # Start recording
        self.start_recording()

    def audio_callback(self, indata, frames, time, status):
        """Called by sounddevice for each audio chunk"""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.audio_queue.put(indata.copy())

    def start_recording(self):
        """Start continuous microphone recording"""
        self.is_recording = True

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,  # Mono
            dtype='float32',
            callback=self.audio_callback,
            blocksize=int(self.sample_rate * 0.1)  # 100ms blocks
        )
        self.stream.start()
        self.get_logger().info('Microphone recording started')

        # Start transcription thread
        self.transcription_thread = threading.Thread(target=self.transcription_loop)
        self.transcription_thread.daemon = True
        self.transcription_thread.start()

    def transcription_loop(self):
        """Continuously transcribe audio chunks"""
        audio_buffer = []

        while self.is_recording and rclpy.ok():
            try:
                chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.append(chunk)

                # Accumulate 3 seconds of audio
                if len(audio_buffer) >= self.chunk_samples / len(chunk):
                    audio_data = np.concatenate(audio_buffer, axis=0).flatten()

                    # Transcribe
                    result = self.model.transcribe(
                        audio_data,
                        language=self.language,
                        fp16=True
                    )

                    transcription = result["text"].strip()
                    if transcription:
                        # Publish to ROS 2
                        msg = String()
                        msg.data = transcription
                        self.transcription_pub.publish(msg)
                        self.get_logger().info(f'Transcription: {transcription}')

                    # Clear buffer (keep 0.5s overlap)
                    overlap_samples = int(0.5 * self.sample_rate)
                    audio_buffer = [audio_data[-overlap_samples:]]

            except queue.Empty:
                continue

    def stop_recording(self):
        """Stop microphone recording"""
        self.is_recording = False
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
