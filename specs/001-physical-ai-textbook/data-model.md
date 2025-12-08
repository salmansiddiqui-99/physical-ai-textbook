# Content Entity Model
**Feature**: Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-08
**Phase**: 1 (Design & Contracts)

## Overview

This document defines the content entities and their relationships for the Physical AI & Humanoid Robotics textbook. These entities structure the educational content and ensure consistency across all 12 chapters.

---

## Entity Definitions

### 1. Module

**Purpose**: Top-level learning unit grouping related chapters

**Attributes**:
- `module_id` (integer, 1-4): Unique identifier
- `title` (string): Full module name (e.g., "The Robotic Nervous System (ROS 2)")
- `description` (string, 200-500 chars): Module overview explaining scope and relevance
- `learning_outcomes` (array of strings): 3-5 high-level skills students gain from completing module
- `prerequisites` (array of strings): Required knowledge before starting module
- `estimated_hours` (integer): Total time to complete all 3 chapters (reading + hands-on)
- `slug` (string): URL-safe identifier (e.g., "module1-ros2")

**Relationships**:
- **Contains**: Exactly 3 `Chapter` entities
- **Precedes**: Next `Module` in sequence (except Module 4)

**Validation Rules**:
- `module_id` must be unique (1-4)
- Must contain exactly 3 chapters
- `learning_outcomes` must use action verbs (create, implement, configure, analyze)
- `estimated_hours` must be sum of chapter `estimated_time` values

**Example**:
```json
{
  "module_id": 1,
  "title": "The Robotic Nervous System (ROS 2)",
  "description": "Master the Robot Operating System 2 middleware for humanoid robot control, from basic pub/sub patterns to action servers and URDF modeling.",
  "learning_outcomes": [
    "Create ROS 2 nodes using rclpy for robot control",
    "Implement action servers for complex humanoid behaviors",
    "Design URDF models for humanoid robot simulation"
  ],
  "prerequisites": [
    "Basic Python programming (functions, classes, decorators)",
    "Linux command-line familiarity",
    "Understanding of coordinate frames and transforms"
  ],
  "estimated_hours": 8,
  "slug": "module1-ros2"
}
```

---

### 2. Chapter

**Purpose**: Self-contained educational unit teaching specific concepts within a module

**Attributes**:
- `chapter_id` (string, format "M.C"): Unique identifier (e.g., "1.2" = Module 1, Chapter 2)
- `title` (string, 10-80 chars): Full chapter title
- `sidebar_label` (string, max 40 chars): Abbreviated title for navigation
- `sidebar_position` (integer, 1-12): Global sequential position across all modules
- `module_id` (integer): Parent module reference
- `description` (string, 50-200 chars): SEO-optimized one-sentence summary
- `keywords` (array of strings): 3-10 technical search terms
- `prerequisites` (array of strings): Prior chapters or external knowledge required
- `learning_objectives` (array of strings): 3-6 measurable outcomes (SMART format)
- `estimated_time` (string): Format "[N] minutes" for total chapter completion
- `content_sections` (array of Section objects): Ordered list of content sections
- `code_examples` (array of CodeExample objects): All runnable code snippets
- `slug` (string): URL-safe identifier (e.g., "ch1-ros2-basics")

**Relationships**:
- **Belongs to**: One `Module`
- **Contains**: 3-8 `Section` entities
- **Contains**: 4-12 `CodeExample` entities
- **Precedes**: Next `Chapter` in module or course (except ch3-capstone-project)

**Validation Rules**:
- `chapter_id` must match pattern `[1-4]\.[1-3]`
- `sidebar_position` must be unique (1-12) and sequential
- Content must be <50KB Markdown
- CEFR B1-B2 readability score (Flesch-Kincaid 60-70)
- No placeholder text ([TODO], [TBD], [NEEDS CLARIFICATION])
- All internal links must resolve
- All code blocks must have language identifiers

**Example**:
```json
{
  "chapter_id": "1.1",
  "title": "ROS 2 Basics: Architecture, Nodes, and Communication",
  "sidebar_label": "ROS 2 Basics",
  "sidebar_position": 1,
  "module_id": 1,
  "description": "Learn ROS 2 architecture fundamentals including nodes, topics, services, and the publish-subscribe pattern for robot communication.",
  "keywords": ["ROS 2", "nodes", "topics", "services", "pub/sub", "rclpy", "middleware"],
  "prerequisites": [
    "Python 3 basics (functions, classes)",
    "Linux terminal commands (cd, ls, source)"
  ],
  "learning_objectives": [
    "Explain ROS 2 architecture and DDS middleware",
    "Create and run ROS 2 nodes using rclpy",
    "Implement publish-subscribe communication with topics",
    "Use ROS 2 services for synchronous request-response patterns"
  ],
  "estimated_time": "90 minutes",
  "slug": "ch1-ros2-basics"
}
```

---

### 3. Section

**Purpose**: Thematic subdivision of chapter content (not a standalone entity, embedded in Chapter)

**Attributes**:
- `section_id` (string): Within-chapter identifier (e.g., "intro", "architecture", "hands-on")
- `title` (string): Section heading (H2 level)
- `content_type` (enum): "concept" | "tutorial" | "example" | "summary"
- `paragraph_count` (integer): Number of paragraphs (for readability tracking)
- `has_code` (boolean): Whether section contains code examples

**Validation Rules**:
- Each section max 4 sentences per paragraph
- Tutorial sections must include step-by-step instructions
- Summary sections must use bullet lists

---

### 4. CodeExample

**Purpose**: Runnable code snippet demonstrating specific concept or technique

**Attributes**:
- `example_id` (string): Unique identifier within chapter (e.g., "minimal-publisher", "action-server-goal")
- `title` (string): Descriptive name (e.g., "Minimal Publisher Node")
- `purpose` (string): One-sentence explanation of what code demonstrates
- `language` (enum): "python" | "bash" | "yaml" | "xml" | "cpp"
- `code_block` (string): Full source code with comments
- `dependencies` (array of strings): Required packages/libraries
- `environment` (enum): "ROS2-Humble" | "ROS2-Iron" | "Gazebo-Fortress" | "IsaacSim" | "Unity"
- `validation_status` (enum): "syntax-valid" | "tested-passes" | "tested-fails" | "not-tested"
- `installation_commands` (string, optional): Bash commands to install dependencies
- `execution_commands` (string): How to run the code
- `expected_output` (string, optional): Sample output for verification
- `troubleshooting` (array of objects, optional): Common issues and solutions

**Relationships**:
- **Embedded in**: One `Chapter`
- **References**: Optional external resources (ROS 2 docs, GitHub repos)

**Validation Rules**:
- `code_block` must have valid syntax for `language`
- Python code must include all imports
- ROS 2 code must include shebang `#!/usr/bin/env python3`
- `dependencies` must list all non-standard packages
- `validation_status` must be "tested-passes" before chapter publication
- Maximum code block size: 100 lines (longer examples split or provided as downloadable files)

**Example**:
```json
{
  "example_id": "minimal-publisher",
  "title": "Minimal Publisher Node",
  "purpose": "Demonstrates basic ROS 2 node creating and publishing string messages to a topic",
  "language": "python",
  "code_block": "#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom std_msgs.msg import String\n\nclass MinimalPublisher(Node):\n    def __init__(self):\n        super().__init__('minimal_publisher')\n        self.publisher_ = self.create_publisher(String, 'topic', 10)\n        self.timer = self.create_timer(0.5, self.timer_callback)\n        self.i = 0\n\n    def timer_callback(self):\n        msg = String()\n        msg.data = f'Hello World: {self.i}'\n        self.publisher_.publish(msg)\n        self.get_logger().info(f'Publishing: \"{msg.data}\"')\n        self.i += 1\n\ndef main(args=None):\n    rclpy.init(args=args)\n    node = MinimalPublisher()\n    rclpy.spin(node)\n    node.destroy_node()\n    rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()",
  "dependencies": ["rclpy", "std_msgs"],
  "environment": "ROS2-Humble",
  "validation_status": "tested-passes",
  "installation_commands": "sudo apt install ros-humble-rclpy ros-humble-std-msgs",
  "execution_commands": "python3 minimal_publisher.py",
  "expected_output": "[INFO] [minimal_publisher]: Publishing: \"Hello World: 0\"\\n[INFO] [minimal_publisher]: Publishing: \"Hello World: 1\"",
  "troubleshooting": [
    {
      "issue": "ModuleNotFoundError: No module named 'rclpy'",
      "solution": "Source ROS 2 setup: source /opt/ros/humble/setup.bash"
    }
  ]
}
```

---

### 5. FrontMatter

**Purpose**: YAML metadata for Docusaurus page configuration and SEO

**Attributes**:
- `id` (string): Must match file path (e.g., "module1-ros2/ch1-ros2-basics")
- `title` (string): Full chapter title (displayed as H1)
- `sidebar_label` (string, optional): Abbreviated navigation label
- `sidebar_position` (integer): Sequential position (1-12)
- `description` (string): SEO meta description
- `keywords` (array of strings): Search terms
- `prerequisites` (array of strings): Required prior knowledge
- `learning_objectives` (array of strings): Measurable outcomes
- `estimated_time` (string): Completion time estimate

**Relationships**:
- **Belongs to**: One `Chapter` (1:1 relationship)

**Validation Rules**:
- All required fields must be present (`id`, `title`, `sidebar_position`, `description`, `learning_objectives`)
- `id` must match file path without `.md` extension
- `sidebar_position` must be unique across all chapters
- `description` length: 50-200 characters
- `keywords` count: 3-10 items

**JSON Schema**: See `contracts/frontmatter-schema.json` for formal validation schema

---

## Entity Relationship Diagram (ERD)

```
┌─────────────────┐
│     Module      │
│  (4 instances)  │
│                 │
│ - module_id     │
│ - title         │
│ - description   │
│ - learning_out  │
│ - prerequisites │
│ - estimated_hrs │
└────────┬────────┘
         │
         │ contains (1:3)
         │
         ▼
┌─────────────────┐
│     Chapter     │
│ (12 instances)  │
│                 │
│ - chapter_id    │
│ - title         │
│ - sidebar_pos   │
│ - learning_obj  │
│ - prerequisites │
└────┬───────┬────┘
     │       │
     │       │ has (1:1)
     │       │
     │       ▼
     │  ┌────────────┐
     │  │ FrontMatter│
     │  │            │
     │  │ - id       │
     │  │ - keywords │
     │  │ - descrip. │
     │  └────────────┘
     │
     │ contains (1:many)
     │
     ▼
┌──────────────────┐
│   CodeExample    │
│ (50-100 total)   │
│                  │
│ - example_id     │
│ - language       │
│ - code_block     │
│ - dependencies   │
│ - validation     │
└──────────────────┘
```

---

## Content Inventory

| Module | Chapters | Code Examples (Est.) | Total Words (Est.) |
|--------|----------|----------------------|--------------------|
| Module 1: ROS 2 | 3 | 12-15 | 8,000-9,000 |
| Module 2: Simulation | 3 | 10-12 | 7,000-8,000 |
| Module 3: Isaac | 3 | 12-15 | 8,000-9,000 |
| Module 4: VLA | 3 | 15-20 | 9,000-10,000 |
| **Total** | **12** | **50-62** | **32,000-36,000** |

**File Size Estimates**:
- Average chapter: 35-40KB Markdown
- Total documentation: ~450-500KB
- Code examples (separate files): ~50KB
- Images/diagrams: ~5MB (compressed)

---

## Validation Checklist

Before chapter publication, verify:

- [ ] Chapter entity has all required attributes
- [ ] `chapter_id` follows pattern `[1-4]\.[1-3]`
- [ ] `sidebar_position` is unique and sequential
- [ ] Markdown content <50KB
- [ ] CEFR B1-B2 readability score (Flesch-Kincaid 60-70)
- [ ] No placeholder text ([TODO], [TBD])
- [ ] All code examples have `validation_status: tested-passes`
- [ ] All internal chapter links resolve
- [ ] All code blocks have language identifiers
- [ ] Front matter validates against JSON schema
- [ ] Learning objectives use action verbs and are measurable

---

## Next Steps

1. ✅ Data model defined
2. → Generate chapter templates in `contracts/`
3. → Generate front matter JSON schema
4. → Generate quickstart.md development guide
5. → Proceed to Phase 2 task generation (`/sp.tasks`)

**Data Model Status**: ✅ Complete
**Date**: 2025-12-08
