---
id: module-4-chapter-1
slug: /module-4/chapter-1
title: "Chapter 1: Voice-to-Action with Whisper"
sidebar_position: 2
---

# Chapter 1: Voice-to-Action with Whisper

## Introduction

Voice-to-action systems enable natural human-robot interaction by converting spoken commands into executable robot actions. This chapter explores how to implement voice-to-action systems using OpenAI's Whisper for speech recognition and integration with robotic control systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement speech recognition using OpenAI Whisper
- Design command parsing systems for robot actions
- Integrate voice recognition with robotic control
- Handle ambiguous or unclear voice commands

## Whisper Speech Recognition Setup

```python
import whisper
import torch
import numpy as np
import pyaudio
import wave
from queue import Queue
import threading

class WhisperVoiceRecognizer:
    def __init__(self, model_size="base"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)
        self.audio_queue = Queue()

        # Audio parameters
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

    def start_listening(self):
        """Start recording audio in a separate thread"""
        audio_thread = threading.Thread(target=self._record_audio)
        audio_thread.daemon = True
        audio_thread.start()

    def _record_audio(self):
        """Record audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        while True:
            data = stream.read(self.chunk_size)
            self.audio_queue.put(data)

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        # Convert audio data to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize to [-1, 1]

        # Transcribe using Whisper
        result = self.model.transcribe(audio_array)
        return result["text"]

    def get_transcription(self):
        """Get transcription from recorded audio"""
        if not self.audio_queue.empty():
            audio_chunks = []
            while not self.audio_queue.empty():
                audio_chunks.append(self.audio_queue.get())

            if audio_chunks:
                # Combine audio chunks
                full_audio = b''.join(audio_chunks)
                return self.transcribe_audio(full_audio)

        return None
```

## Command Parsing and Action Mapping

```python
import re
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class RobotAction:
    """Represents a robot action with parameters"""
    action_type: str
    parameters: Dict[str, any]
    confidence: float = 1.0

class CommandParser:
    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move': [
                r'move (?P<direction>forward|backward|left|right) (?P<distance>\d+(?:\.\d+)?) meters?',
                r'go (?P<direction>forward|backward|left|right) (?P<distance>\d+(?:\.\d+)?) meters?',
                r'walk (?P<direction>forward|backward|left|right) (?P<distance>\d+(?:\.\d+)?) meters?'
            ],
            'turn': [
                r'turn (?P<direction>left|right) (?P<angle>\d+(?:\.\d+)?) degrees?',
                r'rotate (?P<direction>left|right) (?P<angle>\d+(?:\.\d+)?) degrees?',
                r'pivot (?P<direction>left|right) (?P<angle>\d+(?:\.\d+)?) degrees?'
            ],
            'arm': [
                r'move arm to (?P<position>front|back|side|up|down)',
                r'raise arm',
                r'lower arm',
                r'extend arm'
            ],
            'greet': [
                r'hello',
                r'hi',
                r'greetings',
                r'say hello',
                r'say hi'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'freeze',
                r'pause'
            ]
        }

    def parse_command(self, text: str) -> Optional[RobotAction]:
        """Parse voice command and return corresponding action"""
        text = text.lower().strip()

        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    params = match.groupdict()
                    # Convert numeric parameters to appropriate types
                    for key, value in params.items():
                        if value and self._is_numeric(value):
                            params[key] = float(value)

                    return RobotAction(action_type, params)

        return None  # No matching command found

    def _is_numeric(self, value: str) -> bool:
        """Check if a string represents a number"""
        try:
            float(value)
            return True
        except ValueError:
            return False

class VoiceToActionSystem:
    def __init__(self):
        self.voice_recognizer = WhisperVoiceRecognizer()
        self.command_parser = CommandParser()
        self.action_queue = Queue()

    def process_voice_command(self, audio_data):
        """Process voice command and queue appropriate action"""
        # Transcribe audio
        transcription = self.voice_recognizer.transcribe_audio(audio_data)

        if transcription:
            print(f"Recognized: {transcription}")

            # Parse command
            action = self.command_parser.parse_command(transcription)

            if action:
                print(f"Action: {action.action_type} with params {action.parameters}")
                self.action_queue.put(action)
                return action
            else:
                print(f"Unknown command: {transcription}")

        return None
```

## Robot Action Execution

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
import time

class RobotActionExecutor(Node):
    def __init__(self):
        super().__init__('voice_action_executor')

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/tts_input', 10)

        # Action execution parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.action_queue = Queue()

    def execute_action(self, action: RobotAction):
        """Execute the parsed robot action"""
        if action.action_type == 'move':
            self._execute_move_action(action)
        elif action.action_type == 'turn':
            self._execute_turn_action(action)
        elif action.action_type == 'arm':
            self._execute_arm_action(action)
        elif action.action_type == 'greet':
            self._execute_greet_action(action)
        elif action.action_type == 'stop':
            self._execute_stop_action(action)
        else:
            self.get_logger().warn(f"Unknown action type: {action.action_type}")

    def _execute_move_action(self, action: RobotAction):
        """Execute movement action"""
        direction = action.parameters.get('direction', 'forward')
        distance = action.parameters.get('distance', 1.0)

        # Calculate movement time
        move_time = distance / self.linear_speed

        # Create twist message
        twist = Twist()

        if direction == 'forward':
            twist.linear.x = self.linear_speed
        elif direction == 'backward':
            twist.linear.x = -self.linear_speed
        elif direction == 'left':
            twist.linear.y = self.linear_speed
        elif direction == 'right':
            twist.linear.y = -self.linear_speed

        # Execute movement
        self.cmd_vel_pub.publish(twist)
        time.sleep(move_time)

        # Stop robot
        self.cmd_vel_pub.publish(Twist())

    def _execute_turn_action(self, action: RobotAction):
        """Execute turning action"""
        direction = action.parameters.get('direction', 'left')
        angle = action.parameters.get('angle', 90.0)

        # Convert angle to radians
        angle_rad = angle * 3.14159 / 180.0

        # Calculate turn time
        turn_time = angle_rad / self.angular_speed

        # Create twist message
        twist = Twist()

        if direction == 'left':
            twist.angular.z = self.angular_speed
        elif direction == 'right':
            twist.angular.z = -self.angular_speed

        # Execute turn
        self.cmd_vel_pub.publish(twist)
        time.sleep(turn_time)

        # Stop robot
        self.cmd_vel_pub.publish(Twist())

    def _execute_greet_action(self, action: RobotAction):
        """Execute greeting action"""
        greeting = "Hello! How can I help you today?"
        msg = String()
        msg.data = greeting
        self.speech_pub.publish(msg)

    def _execute_stop_action(self, action: RobotAction):
        """Execute stop action"""
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot stopped by voice command")
```

## Advanced Voice Processing

### Context-Aware Command Processing

```python
from enum import Enum
import json

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    MANIPULATING = "manipulating"
    LISTENING = "listening"

class ContextAwareVoiceProcessor:
    def __init__(self):
        self.current_state = RobotState.IDLE
        self.command_context = {}
        self.conversation_history = []

    def update_context(self, state: RobotState, environment_data: dict = None):
        """Update processing context based on robot state and environment"""
        self.current_state = state
        if environment_data:
            self.command_context.update(environment_data)

    def filter_commands_by_context(self, raw_command: str) -> str:
        """Filter commands based on current context"""
        # Example: If robot is moving, prioritize stop commands
        if self.current_state == RobotState.MOVING:
            if "stop" in raw_command.lower() or "halt" in raw_command.lower():
                return raw_command

        # Example: If robot is in a narrow corridor, filter out wide turning commands
        if self.command_context.get('narrow_space', False):
            if 'turn' in raw_command.lower() and 'wide' not in raw_command.lower():
                # Allow only careful turning in narrow spaces
                return raw_command.replace('turn', 'careful turn')

        return raw_command

    def handle_ambiguous_command(self, command_text: str) -> List[str]:
        """Handle commands that could have multiple interpretations"""
        possible_interpretations = []

        # Check for ambiguous phrases
        if "go to the" in command_text.lower():
            # Could be referring to multiple objects
            possible_interpretations.append(f"Did you mean the {self.command_context.get('closest_object')}?")
            possible_interpretations.append(f"Or did you mean the {self.command_context.get('alternative_object')}?")

        elif "move" in command_text.lower() and not any(d in command_text.lower() for d in ['forward', 'backward', 'left', 'right']):
            # Movement without direction
            possible_interpretations.append("In which direction would you like me to move?")

        return possible_interpretations
```

### Voice Activity Detection

```python
import numpy as np
from scipy import signal

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.energy_threshold = 0.01  # Adjust based on environment
        self.silence_duration_threshold = 0.5  # seconds
        self.min_speech_duration = 0.2  # seconds

    def detect_voice_activity(self, audio_data):
        """Detect if voice is present in audio data"""
        # Convert to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize

        # Calculate energy
        energy = np.mean(audio_array ** 2)

        # Apply high-pass filter to remove DC offset
        b, a = signal.butter(3, 0.01, 'highpass', fs=self.sample_rate)
        filtered_audio = signal.filtfilt(b, a, audio_array)

        # Calculate energy of filtered signal
        filtered_energy = np.mean(filtered_audio ** 2)

        # Voice activity if energy exceeds threshold
        return filtered_energy > self.energy_threshold

    def segment_speech(self, audio_stream):
        """Segment speech from continuous audio stream"""
        speech_segments = []
        current_segment = []
        silence_count = 0
        samples_per_silence = int(self.silence_duration_threshold * self.sample_rate)

        for audio_chunk in audio_stream:
            if self.detect_voice_activity(audio_chunk):
                current_segment.append(audio_chunk)
                silence_count = 0  # Reset silence counter
            else:
                silence_count += len(audio_chunk)
                if current_segment and silence_count > samples_per_silence:
                    # End of speech segment
                    if len(current_segment) * len(audio_chunk) > self.min_speech_duration * self.sample_rate:
                        speech_segments.append(b''.join(current_segment))
                    current_segment = []

        return speech_segments
```

## Integration with ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Twist

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Initialize voice processing components
        self.voice_recognizer = WhisperVoiceRecognizer()
        self.command_parser = CommandParser()
        self.action_executor = RobotActionExecutor()
        self.vad = VoiceActivityDetector()

        # ROS 2 interfaces
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10
        )

        self.command_pub = self.create_publisher(
            String, '/parsed_command', 10
        )

        self.get_logger().info("Voice-to-Action system initialized")

    def audio_callback(self, msg: AudioData):
        """Process incoming audio data"""
        # Detect voice activity
        if self.vad.detect_voice_activity(msg.data):
            # Process voice command
            transcription = self.voice_recognizer.transcribe_audio(msg.data)

            if transcription:
                # Parse command
                action = self.command_parser.parse_command(transcription)

                if action:
                    # Execute action
                    self.action_executor.execute_action(action)

                    # Publish parsed command
                    cmd_msg = String()
                    cmd_msg.data = f"{action.action_type}: {action.parameters}"
                    self.command_pub.publish(cmd_msg)

                    self.get_logger().info(f"Executed action: {cmd_msg.data}")
                else:
                    self.get_logger().warn(f"Unrecognized command: {transcription}")
```

## Performance and Accuracy Considerations

### Accuracy Improvements

```python
class EnhancedWhisperProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.language = "en"  # Set to robot's primary language

    def transcribe_with_options(self, audio_array, task="transcribe", temperature=0.0):
        """Transcribe with specific options for better accuracy"""
        options = {
            "task": task,
            "language": self.language,
            "temperature": temperature,  # Lower for more consistent results
            "best_of": 5,  # Generate multiple options and pick best
            "beam_size": 5,  # Use beam search for better results
        }

        result = self.model.transcribe(audio_array, **options)
        return result

    def process_with_context(self, audio_array, context_phrases=None):
        """Process audio with known context phrases for better accuracy"""
        if context_phrases:
            # Use the known phrases to bias recognition
            # This is a simplified approach - actual implementation would use more sophisticated methods
            pass

        return self.transcribe_with_options(audio_array)
```

## Error Handling and Robustness

```python
class RobustVoiceToActionSystem:
    def __init__(self):
        self.voice_recognizer = EnhancedWhisperProcessor()
        self.command_parser = CommandParser()
        self.action_executor = RobotActionExecutor()
        self.consecutive_errors = 0
        self.max_consecutive_errors = 3

    def safe_process_command(self, audio_data):
        """Process command with error handling"""
        try:
            # Transcribe audio
            transcription = self.voice_recognizer.transcribe_with_options(audio_data)

            if not transcription or transcription.strip() == "":
                self.get_logger().warn("Empty transcription received")
                return None

            # Parse command
            action = self.command_parser.parse_command(transcription)

            if action is None:
                self.get_logger().warn(f"No valid command found in: {transcription}")
                self.consecutive_errors += 1
                return None

            # Execute action
            self.action_executor.execute_action(action)
            self.consecutive_errors = 0  # Reset error counter

            return action

        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {str(e)}")
            self.consecutive_errors += 1

            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().error("Too many consecutive errors, system may need reset")

            return None
```

## Summary

Voice-to-action systems using Whisper enable natural human-robot interaction by converting spoken commands into executable robot actions. The system requires careful integration of speech recognition, command parsing, and robot control to provide a responsive and reliable user experience. Proper handling of ambiguous commands, environmental noise, and error conditions is essential for robust operation.