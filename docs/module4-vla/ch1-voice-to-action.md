---
id: ch1-voice-to-action
title: "Voice-to-Action: Speech Integration for Humanoid Control"
sidebar_label: "Voice-to-Action"
sidebar_position: 10
description: Learn to integrate speech recognition using Whisper, build natural language command parsers, and map voice commands to ROS 2 actions for intuitive humanoid robot control.
keywords:
  - Whisper
  - speech recognition
  - voice control
  - natural language processing
  - intent recognition
  - ROS 2 actions
  - voice commands
  - STT
prerequisites:
  - Completion of Modules 1-3 (ROS 2, Simulation, Isaac Platform)
  - Python 3.10+ with transformers library
  - Basic understanding of natural language processing concepts
learning_objectives:
  - Integrate OpenAI Whisper for real-time speech-to-text transcription
  - Implement natural language command parsing and intent recognition
  - Create intent-to-action mappers that trigger ROS 2 action servers
  - Build complete voice-controlled humanoid robot systems
estimated_time: 90 minutes
---

# Voice-to-Action: Speech Integration for Humanoid Control

## Learning Objectives

By the end of this chapter, you will be able to:

- Integrate OpenAI Whisper for accurate real-time speech recognition
- Parse natural language commands and extract actionable intents
- Map voice commands to ROS 2 action calls for humanoid control
- Build complete voice-controlled navigation and manipulation pipelines

## Prerequisites

Before starting this chapter, you should:

- Have completed [Module 3: Isaac Platform](../module3-isaac/ch1-isaac-sim.md)
- Understand ROS 2 actions and services from [Module 1](../module1-ros2/ch2-ros2-humanoids.md)
- Have Python 3.10+ installed with pip
- Have basic familiarity with speech recognition concepts

## Introduction

Voice control represents the most natural human-robot interface. Instead of joysticks, keyboards, or mobile apps, users simply speak: "Go to the kitchen", "Pick up the red mug", "Follow me". For humanoid robots operating in homes and offices, voice commands enable intuitive interaction with non-expert users—elderly care recipients, office workers, or children.

Modern speech recognition has reached human-level accuracy thanks to deep learning models like **OpenAI Whisper**. Whisper achieves 95%+ word accuracy across 99 languages, handles noisy environments, and runs efficiently on edge devices like NVIDIA Jetson. Combined with large language models (LLMs) for intent parsing, we can build robust voice control systems that understand context, handle ambiguity, and gracefully recover from errors.

This chapter covers the complete voice-to-action pipeline: capturing audio, transcribing speech with Whisper, parsing commands to extract intents ("navigate to kitchen" → `NavigateToPose` action), and executing actions via ROS 2. You'll build a system where saying "Turn around" triggers a 180° rotation, and "Bring me a bottle" initiates grasping and navigation.

---

## Section 1: Speech Recognition with Whisper

### Subsection 1.1: Why Whisper for Robotics?

**OpenAI Whisper** is a transformer-based automatic speech recognition (ASR) model trained on 680,000 hours of multilingual data.

**Key Advantages for Humanoid Robots**:
- **Robustness**: Handles background noise, accents, and mispronunciations
- **Multilingual**: Supports 99 languages (critical for global deployment)
- **Low Latency**: Real-time transcription on RTX GPUs or Jetson Xavier
- **Open Source**: BSD-3 license, runs offline (privacy for home robots)
- **No Fine-Tuning**: Works out-of-box for robotics commands

**Model Sizes**:

| Model | Parameters | Memory | Speed (RTX 3060) | Use Case |
|-------|-----------|--------|------------------|----------|
| Tiny | 39M | 1GB | 32x realtime | Edge devices (Jetson Nano) |
| Base | 74M | 1GB | 16x realtime | Jetson Xavier/Orin |
| Small | 244M | 2GB | 6x realtime | Desktop/laptop |
| Medium | 769M | 5GB | 2x realtime | High accuracy |
| Large | 1550M | 10GB | 1x realtime | Production (best accuracy) |

**For humanoid robots**: Use `small` or `base` for real-time performance.

### Subsection 1.2: Installing Whisper

```bash
# Install Whisper via pip
pip install openai-whisper

# Install PyTorch with CUDA support (for GPU acceleration)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install audio libraries
sudo apt-get install ffmpeg portaudio19-dev

# Install Python audio interface
pip install pyaudio sounddevice
```

**Verify Installation**:
```bash
python -c "import whisper; print('Whisper version:', whisper.__version__)"
```

### Subsection 1.3: Basic Whisper Usage

**Example: Transcribe Audio File**:

```python
import whisper

# Load model (downloads on first run, ~1GB for 'base')
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("audio.mp3")

print("Transcription:", result["text"])
print("Language detected:", result["language"])

# Access word-level timestamps
for segment in result["segments"]:
    print(f"[{segment['start']:.2f}s - {segment['end']:.2f}s] {segment['text']}")
```

**Output**:
```
Transcription: Go to the kitchen and bring me a bottle of water.
Language detected: en
[0.00s - 1.50s] Go to the kitchen
[1.50s - 3.20s] and bring me a bottle of water.
```

### Subsection 1.4: Real-Time Microphone Transcription

**Example: Continuous Voice Recognition**:

```python
#!/usr/bin/env python3
"""
Real-time Whisper speech recognition

Purpose: Capture audio from microphone and transcribe in real-time
Environment: Python 3.10+, Whisper, PyAudio
"""

import whisper
import numpy as np
import sounddevice as sd
import queue
import threading

class WhisperMicrophone:
    """Real-time speech recognition using Whisper"""

    def __init__(self, model_size="base", language="en"):
        # Load Whisper model
        print(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        self.language = language

        # Audio parameters
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 3.0  # Process 3-second chunks
        self.chunk_samples = int(self.sample_rate * self.chunk_duration)

        # Audio buffer
        self.audio_queue = queue.Queue()
        self.is_recording = False

    def audio_callback(self, indata, frames, time, status):
        """Called by sounddevice for each audio chunk"""
        if status:
            print(f"Audio status: {status}")
        # Add audio to queue
        self.audio_queue.put(indata.copy())

    def start_recording(self):
        """Start continuous microphone recording"""
        self.is_recording = True

        # Start audio stream
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,  # Mono
            dtype='float32',
            callback=self.audio_callback,
            blocksize=int(self.sample_rate * 0.1)  # 100ms blocks
        )
        self.stream.start()
        print("Recording started. Speak into microphone...")

    def stop_recording(self):
        """Stop microphone recording"""
        self.is_recording = False
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        print("Recording stopped.")

    def transcribe_stream(self):
        """Transcribe audio chunks as they arrive"""
        audio_buffer = []

        while self.is_recording:
            try:
                # Get audio chunk (non-blocking, 100ms timeout)
                chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.append(chunk)

                # Accumulate enough audio (3 seconds)
                if len(audio_buffer) >= self.chunk_samples / len(chunk):
                    # Concatenate buffer
                    audio_data = np.concatenate(audio_buffer, axis=0)
                    audio_data = audio_data.flatten()

                    # Transcribe with Whisper
                    result = self.model.transcribe(
                        audio_data,
                        language=self.language,
                        fp16=True  # Use FP16 for speed (requires GPU)
                    )

                    transcription = result["text"].strip()
                    if transcription:
                        yield transcription

                    # Clear buffer (keep last 0.5s for overlap)
                    overlap_samples = int(0.5 * self.sample_rate)
                    audio_buffer = [audio_data[-overlap_samples:]]

            except queue.Empty:
                continue


# Usage example
if __name__ == '__main__':
    mic = WhisperMicrophone(model_size="base")
    mic.start_recording()

    try:
        for transcription in mic.transcribe_stream():
            print(f"[Transcription] {transcription}")
            # Process command here (next section)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        mic.stop_recording()
```

**Running**:
```bash
python whisper_microphone.py

# Output:
# Loading Whisper model: base
# Recording started. Speak into microphone...
# [Transcription] Go to the kitchen
# [Transcription] Turn around
# [Transcription] Pick up the red mug
```

---

## Section 2: Natural Language Command Parsing

### Subsection 2.1: Intent Recognition Basics

**Intent**: The action the user wants the robot to perform.

**Example Mappings**:

| Voice Command | Intent | Parameters |
|---------------|--------|-----------|
| "Go to the kitchen" | NAVIGATE | location="kitchen" |
| "Turn left" | ROTATE | direction="left", angle=90 |
| "Pick up the mug" | GRASP | object="mug" |
| "Follow me" | FOLLOW_PERSON | target="user" |
| "Stop" | STOP | None |

**Approaches**:
1. **Rule-Based**: Keyword matching (fast, limited)
2. **ML-Based**: Intent classification with BERT/RoBERTa (accurate, requires training)
3. **LLM-Based**: Few-shot prompting with GPT-4/Claude (flexible, requires API)

### Subsection 2.2: Rule-Based Command Parser

**Example: Simple Keyword Matcher**:

```python
#!/usr/bin/env python3
"""
Rule-based command parser for humanoid robot

Purpose: Extract intents from voice commands using keyword matching
Environment: Python 3.10+
"""

import re
from dataclasses import dataclass
from typing import Optional, Dict, Any


@dataclass
class Intent:
    """Parsed intent from voice command"""
    action: str  # NAVIGATE, ROTATE, GRASP, etc.
    confidence: float  # 0.0-1.0
    parameters: Dict[str, Any]
    raw_command: str


class CommandParser:
    """Parse natural language commands into actionable intents"""

    def __init__(self):
        # Define command patterns
        self.patterns = {
            'NAVIGATE': [
                (r'go to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
                (r'navigate to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
                (r'move to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
            ],
            'ROTATE': [
                (r'turn (left|right)', lambda m: {'direction': m.group(1), 'angle': 90}),
                (r'rotate (\d+) degrees', lambda m: {'angle': int(m.group(1))}),
                (r'turn around', lambda m: {'angle': 180}),
            ],
            'GRASP': [
                (r'pick up (?:the )?(\w+)', lambda m: {'object': m.group(1)}),
                (r'grab (?:the )?(\w+)', lambda m: {'object': m.group(1)}),
                (r'grasp (?:the )?(\w+)', lambda m: {'object': m.group(1)}),
            ],
            'FOLLOW': [
                (r'follow me', lambda m: {'target': 'user'}),
                (r'follow (?:the )?(\w+)', lambda m: {'target': m.group(1)}),
            ],
            'STOP': [
                (r'stop', lambda m: {}),
                (r'halt', lambda m: {}),
                (r'freeze', lambda m: {}),
            ],
        }

    def parse(self, command: str) -> Optional[Intent]:
        """
        Parse voice command into intent

        Args:
            command: Transcribed voice command (lowercase)

        Returns:
            Intent object or None if no match
        """
        command = command.lower().strip()

        # Try each intent pattern
        for action, patterns in self.patterns.items():
            for pattern, extractor in patterns:
                match = re.search(pattern, command)
                if match:
                    parameters = extractor(match)
                    return Intent(
                        action=action,
                        confidence=1.0,  # Rule-based is deterministic
                        parameters=parameters,
                        raw_command=command
                    )

        # No match found
        return None


# Usage example
if __name__ == '__main__':
    parser = CommandParser()

    commands = [
        "Go to the kitchen",
        "Turn left",
        "Pick up the red mug",
        "Turn around",
        "Stop",
        "Make me a sandwich"  # No match
    ]

    for cmd in commands:
        intent = parser.parse(cmd)
        if intent:
            print(f"Command: '{cmd}'")
            print(f"  Intent: {intent.action}")
            print(f"  Parameters: {intent.parameters}")
        else:
            print(f"Command: '{cmd}' - NO MATCH")
        print()
```

**Output**:
```
Command: 'Go to the kitchen'
  Intent: NAVIGATE
  Parameters: {'location': 'kitchen'}

Command: 'Turn left'
  Intent: ROTATE
  Parameters: {'direction': 'left', 'angle': 90}

Command: 'Pick up the red mug'
  Intent: GRASP
  Parameters: {'object': 'red'}

Command: 'Make me a sandwich' - NO MATCH
```

### Subsection 2.3: LLM-Based Intent Extraction (Advanced)

For complex commands, use an LLM like GPT-4 or Claude.

**Example: Using OpenAI API**:

```python
import openai
import json

def parse_with_llm(command: str) -> Intent:
    """Parse command using GPT-4 with structured output"""

    prompt = f"""
You are a parser for humanoid robot voice commands.
Extract the intent and parameters from the following command.

Command: "{command}"

Respond in JSON format:
{{
    "action": "NAVIGATE|ROTATE|GRASP|FOLLOW|STOP",
    "parameters": {{}},
    "confidence": 0.0-1.0
}}

Examples:
- "Go to the kitchen" → {{"action": "NAVIGATE", "parameters": {{"location": "kitchen"}}, "confidence": 0.95}}
- "Turn around" → {{"action": "ROTATE", "parameters": {{"angle": 180}}, "confidence": 1.0}}
- "Pick up the red mug" → {{"action": "GRASP", "parameters": {{"object": "mug", "color": "red"}}, "confidence": 0.9}}
"""

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.0  # Deterministic
    )

    result = json.loads(response.choices[0].message.content)

    return Intent(
        action=result['action'],
        confidence=result['confidence'],
        parameters=result['parameters'],
        raw_command=command
    )
```

---

## Section 3: Intent-to-Action Mapping

### Subsection 3.1: ROS 2 Action Integration

**Map intents to ROS 2 action calls**:

```python
#!/usr/bin/env python3
"""
Intent-to-Action mapper for humanoid robot

Purpose: Execute ROS 2 actions based on parsed voice intents
Environment: ROS 2 Humble, action servers from Module 1
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand
import math


class IntentActionMapper(Node):
    """Map voice intents to ROS 2 actions"""

    def __init__(self):
        super().__init__('intent_action_mapper')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.grasp_client = ActionClient(self, GripperCommand, 'gripper_action')

        # Velocity publisher (for rotation)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Location database (hardcoded for demo)
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0, 'yaw': 0.0},
            'living room': {'x': -3.0, 'y': 1.0, 'yaw': 1.57},
            'bedroom': {'x': 0.0, 'y': -4.0, 'yaw': 3.14},
        }

        self.get_logger().info('Intent-Action mapper ready')

    def execute_intent(self, intent):
        """Execute action based on intent"""
        self.get_logger().info(f"Executing: {intent.action} with {intent.parameters}")

        if intent.action == 'NAVIGATE':
            self.execute_navigate(intent.parameters)
        elif intent.action == 'ROTATE':
            self.execute_rotate(intent.parameters)
        elif intent.action == 'GRASP':
            self.execute_grasp(intent.parameters)
        elif intent.action == 'STOP':
            self.execute_stop()
        else:
            self.get_logger().warn(f"Unknown action: {intent.action}")

    def execute_navigate(self, params):
        """Navigate to named location"""
        location_name = params.get('location')

        if location_name not in self.locations:
            self.get_logger().error(f"Unknown location: {location_name}")
            return

        # Get coordinates
        loc = self.locations[location_name]

        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = loc['x']
        goal_msg.pose.pose.position.y = loc['y']

        # Convert yaw to quaternion
        qz = math.sin(loc['yaw'] / 2.0)
        qw = math.cos(loc['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Send goal
        self.get_logger().info(f"Navigating to {location_name}: ({loc['x']}, {loc['y']})")
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info('Navigation completed!')

    def execute_rotate(self, params):
        """Rotate in place by specified angle"""
        angle = params.get('angle', 90)  # Default 90 degrees
        direction = params.get('direction', 'left')

        # Convert to radians
        angle_rad = math.radians(angle)
        if direction == 'right':
            angle_rad = -angle_rad

        # Calculate rotation duration (assuming 30 deg/s)
        angular_velocity = 0.5  # rad/s
        duration = abs(angle_rad / angular_velocity)

        # Publish rotation command
        twist = Twist()
        twist.angular.z = angular_velocity if angle_rad > 0 else -angular_velocity

        self.get_logger().info(f"Rotating {angle}° {direction}")

        # Publish for calculated duration
        rate = self.create_rate(10)  # 10 Hz
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Rotation complete")

    def execute_grasp(self, params):
        """Execute grasping action"""
        object_name = params.get('object', 'unknown')
        self.get_logger().info(f"Grasping {object_name} (placeholder)")

        # TODO: Integrate with grasp planning system
        # 1. Detect object with vision
        # 2. Plan grasp pose
        # 3. Execute grasp with gripper action

    def execute_stop(self):
        """Emergency stop"""
        self.get_logger().warn("STOP command received - halting all motion")

        # Stop velocity
        self.cmd_vel_pub.publish(Twist())

        # Cancel all navigation goals
        # self.nav_client.cancel_all_goals()


def main(args=None):
    rclpy.init(args=args)
    mapper = IntentActionMapper()

    # Example: Execute intents from parser
    from command_parser import CommandParser, Intent

    parser = CommandParser()

    # Simulate voice commands
    commands = [
        "Go to the kitchen",
        "Turn around",
        "Stop"
    ]

    for cmd in commands:
        intent = parser.parse(cmd)
        if intent:
            mapper.execute_intent(intent)
            rclpy.spin_once(mapper, timeout_sec=2.0)

    mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Section 4: Complete Voice Control System

### Subsection 4.1: Integrated Voice Control Node

**Example: Full Pipeline**:

```python
#!/usr/bin/env python3
"""
Complete voice control system for humanoid robot

Purpose: Whisper → Parser → Action execution pipeline
Environment: ROS 2 Humble, Whisper, GPU recommended
"""

import rclpy
from rclpy.node import Node
import threading
from whisper_microphone import WhisperMicrophone
from command_parser import CommandParser
from intent_action_mapper import IntentActionMapper


class VoiceControlNode(Node):
    """Complete voice-to-action control system"""

    def __init__(self):
        super().__init__('voice_control_node')

        # Initialize components
        self.whisper = WhisperMicrophone(model_size="base")
        self.parser = CommandParser()
        self.action_mapper = IntentActionMapper()

        # Voice activation
        self.declare_parameter('wake_word', 'robot')  # "Robot, go to kitchen"
        self.wake_word = self.get_parameter('wake_word').value.lower()
        self.listening = False

        self.get_logger().info(f'Voice control ready. Wake word: "{self.wake_word}"')

    def start(self):
        """Start voice recognition loop"""
        self.whisper.start_recording()

        # Run transcription in separate thread
        transcription_thread = threading.Thread(target=self.transcription_loop)
        transcription_thread.daemon = True
        transcription_thread.start()

    def transcription_loop(self):
        """Process transcriptions continuously"""
        for transcription in self.whisper.transcribe_stream():
            self.process_transcription(transcription)

    def process_transcription(self, text: str):
        """Process transcribed text"""
        text_lower = text.lower()

        # Check for wake word
        if self.wake_word in text_lower:
            self.listening = True
            self.get_logger().info(f"Wake word detected: '{self.wake_word}'")

            # Extract command after wake word
            command = text_lower.split(self.wake_word, 1)[1].strip()
            if command:
                self.execute_command(command)
            return

        # If already listening, process command
        if self.listening:
            self.execute_command(text_lower)
            self.listening = False  # Single-shot mode

    def execute_command(self, command: str):
        """Parse and execute voice command"""
        self.get_logger().info(f"Command received: '{command}'")

        # Parse intent
        intent = self.parser.parse(command)

        if intent:
            self.get_logger().info(
                f"Intent: {intent.action} (confidence: {intent.confidence:.2f})"
            )
            # Execute action
            self.action_mapper.execute_intent(intent)
        else:
            self.get_logger().warn(f"Could not parse command: '{command}'")


def main(args=None):
    rclpy.init(args=args)
    voice_control = VoiceControlNode()

    voice_control.start()

    try:
        rclpy.spin(voice_control)
    except KeyboardInterrupt:
        pass
    finally:
        voice_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running the Complete System**:

```bash
# Terminal 1: Launch robot simulation (Isaac Sim or Gazebo)
ros2 launch humanoid_bringup simulation.launch.py

# Terminal 2: Launch Nav2 navigation
ros2 launch nav2_bringup bringup_launch.py

# Terminal 3: Start voice control
ros2 run voice_control voice_control_node

# Output:
# [INFO] Voice control ready. Wake word: "robot"
# [INFO] Recording started. Speak into microphone...
# [INFO] Wake word detected: 'robot'
# [INFO] Command received: 'go to the kitchen'
# [INFO] Intent: NAVIGATE (confidence: 1.00)
# [INFO] Navigating to kitchen: (5.0, 2.0)
# [INFO] Navigation completed!
```

---

## Hands-On Project: Voice-Controlled Navigation

**Goal**: Build a voice-controlled humanoid that navigates to locations on command.

**Duration**: 45 minutes

**What You'll Learn**:
- Set up Whisper for real-time transcription
- Parse navigation commands
- Execute Nav2 goals from voice input
- Handle errors gracefully

### Step 1: Install Dependencies

```bash
pip install openai-whisper sounddevice pyaudio
```

### Step 2: Test Whisper

```bash
python -c "import whisper; model = whisper.load_model('base'); print('Whisper ready')"
```

### Step 3: Implement Voice Node

(Use complete code from Section 4.1)

### Step 4: Define Locations

Edit location database in `intent_action_mapper.py`:

```python
self.locations = {
    'kitchen': {'x': 5.0, 'y': 2.0, 'yaw': 0.0},
    'entrance': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'office': {'x': -2.0, 'y': 3.0, 'yaw': 1.57},
}
```

### Step 5: Test Voice Commands

```bash
ros2 run voice_control voice_control_node

# Say: "Robot, go to the kitchen"
# Say: "Robot, turn around"
# Say: "Robot, stop"
```

---

## Challenge: Test Your Understanding

1. **Basic**: Add support for "move forward 2 meters" command
   - *Hint*: Parse distance, publish `Twist` message for calculated duration

2. **Intermediate**: Implement voice-controlled object grasping with vision
   - *Hint*: Use YOLO from Module 3, find closest object matching voice description

3. **Advanced**: Add conversation context (multi-turn dialogue)
   - *Hint*: Maintain state machine, "go there" refers to last mentioned location

---

## Summary

In this chapter, you learned:

- **Whisper integration**: Real-time speech recognition with 95%+ accuracy
- **Intent parsing**: Rule-based and LLM approaches for command understanding
- **Action mapping**: Connecting voice intents to ROS 2 actions
- **Complete pipeline**: End-to-end voice control system

**Key Concepts**:
- **Wake word detection**: Activate robot before command
- **Confidence scoring**: Handle ambiguous commands gracefully
- **Action clients**: Asynchronous execution of navigation/manipulation tasks
- **Error handling**: Fallback for unrecognized commands

---

## Further Reading

Official Documentation:
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [ROS 2 Action Servers](https://docs.ros.org/en/humble/Tutorials/Actions.html)
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/)

Tutorials:
- [Building Voice Assistants with Whisper](https://platform.openai.com/docs/guides/speech-to-text)
- [Intent Recognition with Transformers](https://huggingface.co/tasks/text-classification)

Research Papers:
- "Robust Speech Recognition via Large-Scale Weak Supervision" (Whisper paper, 2022)
- "Natural Language Commands for Robot Navigation" (Tellex et al., 2011)

---

## Next Steps

**Next Chapter**: [Chapter 4.2: Cognitive Planning with LLMs](./ch2-cognitive-planning.md)

In the next chapter, you'll learn how to use large language models to decompose complex tasks ("clean the room") into executable action sequences, enabling truly intelligent humanoid behavior.

**Optional Practice**:
- Add support for Spanish or French commands (Whisper multilingual)
- Implement voice feedback ("I'm going to the kitchen now")
- Create a mobile app that sends voice commands over ROS 2
