#!/usr/bin/env python3
"""
Natural language command parser

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
        # Define command patterns (regex → parameter extractor)
        self.patterns = {
            'NAVIGATE': [
                (r'go to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
                (r'navigate to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
                (r'move to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
                (r'head to (?:the )?(\w+)', lambda m: {'location': m.group(1)}),
            ],
            'ROTATE': [
                (r'turn (left|right)', lambda m: {'direction': m.group(1), 'angle': 90}),
                (r'rotate (\d+) degrees', lambda m: {'angle': int(m.group(1))}),
                (r'turn around', lambda m: {'angle': 180}),
                (r'spin around', lambda m: {'angle': 360}),
            ],
            'GRASP': [
                (r'pick up (?:the |a )?(\w+)', lambda m: {'object': m.group(1)}),
                (r'grab (?:the |a )?(\w+)', lambda m: {'object': m.group(1)}),
                (r'grasp (?:the |a )?(\w+)', lambda m: {'object': m.group(1)}),
                (r'take (?:the |a )?(\w+)', lambda m: {'object': m.group(1)}),
            ],
            'FOLLOW': [
                (r'follow me', lambda m: {'target': 'user'}),
                (r'follow (?:the )?(\w+)', lambda m: {'target': m.group(1)}),
                (r'come with me', lambda m: {'target': 'user'}),
            ],
            'STOP': [
                (r'stop', lambda m: {}),
                (r'halt', lambda m: {}),
                (r'freeze', lambda m: {}),
                (r'wait', lambda m: {}),
            ],
        }

    def parse(self, command: str) -> Optional[Intent]:
        """
        Parse voice command into intent

        Args:
            command: Transcribed voice command

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
                        confidence=1.0,
                        parameters=parameters,
                        raw_command=command
                    )

        return None


# Testing
if __name__ == '__main__':
    parser = CommandParser()

    test_commands = [
        "Go to the kitchen",
        "Turn left",
        "Pick up the red mug",
        "Turn around",
        "Stop",
        "Follow me",
        "Navigate to the entrance"
    ]

    for cmd in test_commands:
        intent = parser.parse(cmd)
        if intent:
            print(f"'{cmd}' → {intent.action}({intent.parameters})")
        else:
            print(f"'{cmd}' → NO MATCH")
