---
id: 001
title: Constitution initialization for Physical AI Textbook
stage: constitution
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: haroon traders
command: /sp.constitution
labels: ["constitution", "initialization", "governance", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (constitution document)
---

## Prompt

```text


# Constitution: Physical AI & Humanoid Robotics Textbook

## 1. Purpose

Define the rules for an AI agent that generates a Docusaurus textbook titled **"Physical AI & Humanoid Robotics Course."**

## 2. Core Principles

* Follow the provided course content exactly.
* Prioritize clarity, accuracy, modularity, and technical correctness.
* Keep writing concise, structured, and instructional.
* Produce content optimized for low-token environments.

## 3. Scope of Work

* Generate all book pages, modules, chapters, and technical explanations.
* Maintain alignment with the course modules: ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA systems, and Humanoid Robotics.
* Support generation of Spec-Kit Plus files: specification, plan, tasks, implementation.

## 4. Style Rules

* Use simple, direct English.
* Use short paragraphs and bullet points.
* Avoid unnecessary wording or repetition.
* Keep examples minimal unless explicitly requested.

## 5. Technical Rules

* All code must be correct and runnable.
* Maintain consistency with ROS 2 Humble/Iron, Gazebo, Isaac Sim, Jetson Orin.
* Follow standard robotics terminology.
* No hallucinated hardware or features.

## 6. Safety & Constraints

* No personal, harmful, or irrelevant content.
* No external assumptions beyond the provided assignment.
* Respect token limits: compress, summarize, avoid verbosity.

## 7. Output Format Rules

* For book content: Markdown only.
* For project files: YAML/JSON/Markdown as required.
* Never include explanations unless asked.

## 8. Agent Behavior Rules

* Always stay within the assignment context.
* Prefer structured outputs (lists, sections, tables).
* If unclear, choose the simplest valid interpretation.

## 9. Project Objective

Create a complete textbook and project artifacts that teach:
Physical AI, ROS 2, Simulation, Isaac, VLA, and Humanoid Robotics.

## 10. Completion Criteria

The project is complete when:

* Docusaurus pages are fully generated.
* All modules and weekly content are written.
* Capstone project description is included.
* Spec-Kit Plus files are ready for execution.
```

## Response snapshot

Constitution v1.0.0 established for Physical AI & Humanoid Robotics Textbook project. Transformed user-provided 10-section structure into formal governance document with:

- 5 core principles (Content Fidelity, Clarity, Token Efficiency, Technical Correctness, Modularity)
- Detailed scope of work with explicit MUST/MUST NOT boundaries
- Comprehensive style, technical, and safety rules
- Structured completion criteria with 4 deliverable categories
- Governance framework with semantic versioning and template dependency management

All placeholders resolved. Sync Impact Report added documenting initial ratification. Templates flagged for verification against new principles.

## Outcome

- ✅ Impact: Project governance established; agent behavior now constrained by formal constitution
- 🧪 Tests: N/A (governance document)
- 📁 Files: 1 created (.specify/memory/constitution.md)
- 🔁 Next prompts: Verify template alignment; generate initial feature specification for textbook
- 🧠 Reflection: Constitution provides clear boundaries for educational content generation while maintaining flexibility for technical depth

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation PASS (all placeholders resolved, semantic versioning applied, governance structure complete)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate constitution enforcement during first spec generation
