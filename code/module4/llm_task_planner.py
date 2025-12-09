#!/usr/bin/env python3
"""
LLM-based task planner using GPT-4 or Claude

Purpose: Decompose high-level tasks into executable action sequences
Environment: Python 3.10+, OpenAI or Anthropic API
"""

import json
from typing import List, Dict, Any


class LLMTaskPlanner:
    """Task decomposition using LLM (GPT-4 or Claude)"""

    def __init__(self, api_key: str, model: str = "gpt-4", provider: str = "openai"):
        """
        Args:
            api_key: OpenAI or Anthropic API key
            model: Model name (gpt-4, claude-3-opus, etc.)
            provider: "openai" or "anthropic"
        """
        self.model = model
        self.provider = provider

        if provider == "openai":
            import openai
            openai.api_key = api_key
            self.client = openai
        elif provider == "anthropic":
            import anthropic
            self.client = anthropic.Anthropic(api_key=api_key)

        # Available actions for the robot
        self.actions = [
            "navigate_to(location: str)",
            "detect_objects(object_type: str)",
            "grasp(object_id: str)",
            "place(location: str)",
            "open(container: str)",
            "close(container: str)",
            "rotate(angle_degrees: int)",
            "wait(duration_seconds: float)"
        ]

        # Few-shot examples for prompting
        self.examples = [
            {
                "task": "Bring me a water bottle",
                "plan": [
                    {"action": "navigate_to", "params": {"location": "kitchen"}},
                    {"action": "detect_objects", "params": {"object_type": "bottle"}},
                    {"action": "grasp", "params": {"object_id": "bottle_0"}},
                    {"action": "navigate_to", "params": {"location": "user"}},
                    {"action": "place", "params": {"location": "user_hand"}}
                ]
            },
            {
                "task": "Clean the table",
                "plan": [
                    {"action": "navigate_to", "params": {"location": "dining_table"}},
                    {"action": "detect_objects", "params": {"object_type": "clutter"}},
                    {"action": "grasp", "params": {"object_id": "clutter_0"}},
                    {"action": "navigate_to", "params": {"location": "trash_bin"}},
                    {"action": "place", "params": {"location": "trash_bin"}}
                ]
            }
        ]

    def plan(self, task: str) -> List[Dict[str, Any]]:
        """
        Generate action sequence for task

        Args:
            task: Natural language task description

        Returns:
            List of action dictionaries
        """
        prompt = self._build_prompt(task)

        if self.provider == "openai":
            return self._plan_openai(prompt)
        elif self.provider == "anthropic":
            return self._plan_anthropic(prompt)

    def _build_prompt(self, task: str) -> str:
        """Build few-shot prompt"""
        prompt = "You are a task planning AI for a humanoid robot.\n\n"
        prompt += "Decompose the task into a sequence of primitive actions.\n\n"
        prompt += "Available actions:\n"
        for action in self.actions:
            prompt += f"- {action}\n"
        prompt += "\nExamples:\n\n"

        for ex in self.examples:
            prompt += f"Task: \"{ex['task']}\"\n"
            prompt += f"Plan:\n{json.dumps(ex['plan'], indent=2)}\n\n"

        prompt += f"Now decompose:\n\nTask: \"{task}\"\nPlan:\n"
        return prompt

    def _plan_openai(self, prompt: str) -> List[Dict[str, Any]]:
        """Generate plan using OpenAI"""
        response = self.client.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "Output valid JSON only."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,
            max_tokens=1000
        )

        return self._parse_response(response.choices[0].message.content)

    def _plan_anthropic(self, prompt: str) -> List[Dict[str, Any]]:
        """Generate plan using Anthropic Claude"""
        message = self.client.messages.create(
            model=self.model,
            max_tokens=1000,
            temperature=0.2,
            messages=[{"role": "user", "content": prompt}]
        )

        return self._parse_response(message.content[0].text)

    def _parse_response(self, text: str) -> List[Dict[str, Any]]:
        """Parse LLM response to JSON"""
        try:
            # Handle markdown code blocks
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0]
            elif "```" in text:
                text = text.split("```")[1].split("```")[0]

            plan = json.loads(text.strip())
            return plan
        except json.JSONDecodeError as e:
            print(f"JSON parse error: {e}")
            print(f"Raw text: {text}")
            return []


# Testing
if __name__ == '__main__':
    import os

    # Example with OpenAI (set OPENAI_API_KEY environment variable)
    api_key = os.environ.get('OPENAI_API_KEY', 'sk-...')
    planner = LLMTaskPlanner(api_key=api_key, provider="openai")

    tasks = [
        "Set the table for dinner",
        "Make a cup of coffee",
        "Water the plants"
    ]

    for task in tasks:
        print(f"\nTask: {task}")
        plan = planner.plan(task)
        print("Plan:")
        for i, step in enumerate(plan):
            print(f"  {i+1}. {step['action']}({step['params']})")
