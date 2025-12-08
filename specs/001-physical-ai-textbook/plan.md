# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

**Note**: This plan outlines the architecture for generating a complete Docusaurus-based textbook with 12 chapters across 4 modules covering ROS 2, simulation, NVIDIA Isaac, and VLA systems.

## Summary

Generate a comprehensive educational textbook using Docusaurus as the static site generator, organized into 4 modules with 3 chapters each. Content will cover Physical AI fundamentals from ROS 2 basics through advanced VLA systems, culminating in an integrated capstone project. The approach uses modular content generation to maintain token efficiency while ensuring technical correctness and educational clarity. Deployment target is GitHub Pages with complete CI/CD configuration.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+) for Docusaurus; Python 3.10+ for code examples; Markdown for content
**Primary Dependencies**: Docusaurus 2.x, React 18+, ROS 2 Humble/Iron, Gazebo Classic/Fortress, Isaac Sim
**Storage**: Git repository for version control; static files for deployment; no database required
**Testing**: Markdown linting (markdownlint), code validation (Python pytest for examples), Docusaurus build verification
**Target Platform**: Web (GitHub Pages); code examples target Ubuntu 22.04 LTS with ROS 2 Humble/Iron
**Project Type**: Documentation site (Docusaurus static site generator)
**Performance Goals**: Page load <3s on broadband; Markdown files <50KB each; site build <5min
**Constraints**: CEFR B1-B2 English readability; token-efficient content; no placeholder text; ROS 2 Humble/Iron only
**Scale/Scope**: 12 chapters, 4 modules, ~50-100 code examples, ~200-300 pages estimated output

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**I. Content Fidelity** ✅ PASS
- Plan strictly follows 4-module, 12-chapter structure from spec
- No additional modules or content beyond ROS 2, Gazebo/Unity, Isaac, VLA
- Rationale: Adheres to constitutional principle of exact content alignment

**II. Clarity and Structure** ✅ PASS
- Modular chapter organization supports independent learning paths
- Each chapter self-contained per spec requirements
- Rationale: Modularity principle ensures flexible navigation and maintenance

**III. Token Efficiency** ✅ PASS
- Phased generation approach prevents token overflow
- Content structured for chunked, iterative creation
- Rationale: Respects token limit constraints; enables scalable generation

**IV. Technical Correctness** ✅ PASS
- Code examples limited to verified ROS 2 Humble/Iron APIs
- Standard robotics terminology enforced via linting/review
- Rationale: Ensures runnable code and industry-standard nomenclature

**V. Modularity and Reusability** ✅ PASS
- Each chapter independently deployable and testable
- Cross-references used for concept reuse vs. duplication
- Rationale: Supports constitution's modularity requirement

**Safety & Constraints** ✅ PASS
- No personal opinions, speculation, or off-topic content
- Token limits respected via modular generation strategy
- Rationale: Aligns with content restrictions and resource constraints

**Output Format Rules** ✅ PASS
- Markdown only for chapters with YAML front matter
- Code blocks include language identifiers
- Rationale: Matches constitutional formatting standards

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - technology decisions and patterns
├── data-model.md        # Phase 1 output - content entity model
├── quickstart.md        # Phase 1 output - Docusaurus setup guide
├── contracts/           # Phase 1 output - chapter templates and schemas
│   ├── chapter-template.md
│   ├── frontmatter-schema.json
│   └── code-example-template.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus documentation site structure
docs/
├── intro.md                      # Course overview and prerequisites
├── module1-ros2/                 # Module 1: The Robotic Nervous System
│   ├── ch1-ros2-basics.md
│   ├── ch2-ros2-humanoids.md
│   └── ch3-urdf-humanoids.md
├── module2-simulation/           # Module 2: The Digital Twin
│   ├── ch1-gazebo-essentials.md
│   ├── ch2-sensor-simulation.md
│   └── ch3-unity-visualization.md
├── module3-isaac/                # Module 3: The AI-Robot Brain
│   ├── ch1-isaac-sim.md
│   ├── ch2-isaac-ros-perception.md
│   └── ch3-navigation-humanoids.md
└── module4-vla/                  # Module 4: Vision-Language-Action
    ├── ch1-voice-to-action.md
    ├── ch2-cognitive-planning.md
    └── ch3-capstone-project.md

static/
├── img/                          # Diagrams and images
│   ├── module1/
│   ├── module2/
│   ├── module3/
│   └── module4/
└── code/                         # Downloadable code examples
    ├── module1/
    ├── module2/
    ├── module3/
    └── module4/

src/
├── css/                          # Custom Docusaurus styles
│   └── custom.css
└── pages/                        # Landing page and additional pages
    └── index.js

# Configuration files (repository root)
docusaurus.config.js              # Docusaurus configuration
sidebars.js                       # Navigation sidebar definition
package.json                      # Node.js dependencies
.github/
└── workflows/
    └── deploy.yml                # GitHub Pages deployment workflow

# Testing infrastructure
tests/
├── markdown/                     # Markdown linting tests
├── code-examples/                # Python code validation tests
└── build/                        # Docusaurus build tests
```

**Structure Decision**: Selected Docusaurus documentation site structure (static site generator pattern). This matches the spec requirement for a Docusaurus-based textbook deployed to GitHub Pages. The 4-module organization directly mirrors the course structure, with each module containing 3 chapters as specified. Code examples are stored both inline in Markdown and separately in `/static/code/` for download.

## Complexity Tracking

> No constitution violations requiring justification. All gates passed without exceptions.

---

## Phase 0: Research & Design Decisions

### Research Tasks

1. **Docusaurus Best Practices for Technical Documentation**
   - Research: Optimal sidebar organization for multi-module courses
   - Research: Front matter schema for educational content (learning objectives, prerequisites, difficulty)
   - Research: Code block features (line highlighting, live editing, copy button)
   - Research: Search plugin configuration for technical terminology

2. **ROS 2 Documentation Standards**
   - Research: Official ROS 2 code example formatting conventions
   - Research: Package dependency specification (rosdep format)
   - Research: URDF/Xacro best practices for educational examples
   - Research: Action server example patterns for Humble/Iron

3. **Educational Content Structure**
   - Research: Chapter length guidelines for online technical courses (readability studies)
   - Research: Code-to-explanation ratio for programming tutorials
   - Research: Progressive complexity patterns (beginner → advanced transitions)
   - Research: Assessment/exercise integration without interactive features

4. **Simulation Environment Documentation**
   - Research: Gazebo world file documentation standards
   - Research: Unity ROS integration package references (ROS-TCP-Connector)
   - Research: Isaac Sim USD scene structure documentation
   - Research: Sensor plugin parameter documentation (LiDAR, cameras, IMU)

5. **GitHub Pages Deployment**
   - Research: GitHub Actions workflow for Docusaurus deployment
   - Research: Custom domain configuration (if applicable)
   - Research: Build optimization for large documentation sites
   - Research: Asset optimization (image compression, code bundling)

### Technology Selection Rationale

**Docusaurus 2.x** (selected over alternatives):
- **Alternatives considered**: GitBook, MkDocs Material, Sphinx
- **Decision**: Docusaurus chosen for React-based extensibility, excellent search, and built-in versioning
- **Rationale**: Best GitHub Pages integration, MDX support for interactive examples, active community for robotics docs

**Markdown + MDX** (selected over alternatives):
- **Alternatives considered**: AsciiDoc, reStructuredText
- **Decision**: Markdown with MDX for React component embedding
- **Rationale**: Universally supported, simple for educational content, allows future interactivity

**Python pytest for Code Validation** (selected over alternatives):
- **Alternatives considered**: Manual testing, ROS 2 launch tests
- **Decision**: pytest with mocking for ROS 2 dependencies
- **Rationale**: Enables CI/CD validation without full ROS 2 environment; fast feedback

**GitHub Actions for CI/CD** (selected over alternatives):
- **Alternatives considered**: GitLab CI, Travis CI
- **Decision**: GitHub Actions with Docusaurus deployment action
- **Rationale**: Native GitHub integration, free for public repos, established Docusaurus workflows

---

## Phase 1: Content Model & Contracts

### Content Entity Model (data-model.md)

**Module Entity**:
- **Attributes**: module_id (1-4), title, description, learning_outcomes (list), prerequisites (list), estimated_hours
- **Relationships**: Contains 3 Chapter entities
- **Validation**: Must have exactly 3 chapters; module_id unique

**Chapter Entity**:
- **Attributes**: chapter_id (e.g., "1.2"), title, module_id, learning_objectives (list), prerequisites (list), content_sections (list), code_examples (list), estimated_time (minutes)
- **Relationships**: Belongs to one Module; contains multiple CodeExample entities
- **Validation**: Content <50KB; CEFR B1-B2 readability score; no placeholders

**CodeExample Entity**:
- **Attributes**: example_id, language (python/bash/yaml/xml), code_block, description, dependencies (list), environment (ROS2/Gazebo/Isaac), validation_status
- **Relationships**: Embedded in Chapter
- **Validation**: Syntax valid; dependencies declared; includes imports; executes in target environment

**FrontMatter Entity** (YAML metadata):
- **Attributes**: id (chapter path), title, sidebar_label, sidebar_position, description, keywords (list), prerequisites (list), learning_objectives (list), estimated_time
- **Validation**: All required fields present; keywords relevant; sidebar_position sequential

### API Contracts (chapter templates)

**Chapter Template Contract** (`contracts/chapter-template.md`):
```markdown
---
id: [module-name]/[chapter-slug]
title: [Chapter Title]
sidebar_label: [Short Label]
sidebar_position: [1-12]
description: [One-sentence chapter summary]
keywords: [relevant, technical, terms]
prerequisites:
  - [Prior knowledge item 1]
  - [Prior knowledge item 2]
learning_objectives:
  - [Objective 1: Action verb + specific outcome]
  - [Objective 2: Action verb + specific outcome]
estimated_time: [X minutes]
---

# [Chapter Title]

## Learning Objectives

By the end of this chapter, you will be able to:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Prerequisites

- [Prerequisite 1 with link if applicable]
- [Prerequisite 2]

## Introduction

[2-3 paragraph introduction: why topic matters, real-world context, what students will build]

## Section 1: [Concept Name]

[Explanation with max 4 sentences per paragraph]

### Subsection 1.1: [Specific Topic]

[Content]

```language
# Code example with comments
[example code]
```

**Explanation**: [Walk through code line-by-line or block-by-block]

## Section 2: [Next Concept]

[Continue pattern]

## Hands-On Example: [Project Name]

[Step-by-step tutorial applying concepts]

## Summary

[Recap of key points in bullet list]

## Further Reading

- [Resource 1 with link]
- [Resource 2 with link]

## Next Steps

Continue to [Next Chapter](../link/to/next.md) to learn about [next topic].
```

**Code Example Template Contract** (`contracts/code-example-template.md`):
```markdown
## Example: [Descriptive Title]

**Purpose**: [One sentence explaining what this code demonstrates]

**Environment**: [ROS 2 Humble / Gazebo Fortress / Isaac Sim / Unity]

**Dependencies**:
```bash
# Installation commands
sudo apt install [packages]
pip install [packages]
```

**Code**:
```python
#!/usr/bin/env python3
"""
[Docstring explaining purpose, inputs, outputs]
"""

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    """[Class purpose]"""

    def __init__(self):
        super().__init__('example_node')
        # [Implementation with inline comments]

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the Code**:
```bash
# Step-by-step execution instructions
python3 example_node.py
```

**Expected Output**:
```
[Sample output showing successful execution]
```

**Troubleshooting**:
- **Issue**: [Common problem 1]
  **Solution**: [Fix]
- **Issue**: [Common problem 2]
  **Solution**: [Fix]
```

**Front Matter Schema Contract** (`contracts/frontmatter-schema.json`):
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["id", "title", "sidebar_position", "description", "learning_objectives"],
  "properties": {
    "id": {
      "type": "string",
      "pattern": "^module[1-4]-.+/ch[1-3]-.+$",
      "description": "Unique chapter identifier matching file path"
    },
    "title": {
      "type": "string",
      "minLength": 10,
      "maxLength": 80,
      "description": "Full chapter title"
    },
    "sidebar_label": {
      "type": "string",
      "maxLength": 40,
      "description": "Abbreviated title for sidebar"
    },
    "sidebar_position": {
      "type": "integer",
      "minimum": 1,
      "maximum": 12,
      "description": "Sequential position in course (1-12)"
    },
    "description": {
      "type": "string",
      "minLength": 50,
      "maxLength": 200,
      "description": "One-sentence chapter summary for SEO"
    },
    "keywords": {
      "type": "array",
      "items": {"type": "string"},
      "minItems": 3,
      "maxItems": 10,
      "description": "Technical terms for search indexing"
    },
    "prerequisites": {
      "type": "array",
      "items": {"type": "string"},
      "description": "Prior knowledge required"
    },
    "learning_objectives": {
      "type": "array",
      "items": {"type": "string"},
      "minItems": 3,
      "maxItems": 6,
      "description": "Measurable learning outcomes"
    },
    "estimated_time": {
      "type": "string",
      "pattern": "^[0-9]+ minutes$",
      "description": "Expected completion time"
    }
  }
}
```

### Quickstart Guide (quickstart.md)

**Development Environment Setup**:
1. Install Node.js 18+ and Yarn
2. Clone repository and install dependencies: `yarn install`
3. Start dev server: `yarn start`
4. Open http://localhost:3000

**Content Creation Workflow**:
1. Create new chapter file in appropriate `docs/moduleX/` directory
2. Copy chapter template from `specs/001-physical-ai-textbook/contracts/chapter-template.md`
3. Fill front matter with chapter metadata
4. Write content following template structure
5. Add code examples using code example template
6. Validate Markdown: `yarn lint:markdown`
7. Test code examples: `pytest tests/code-examples/`
8. Build site locally: `yarn build`

**Deployment Process**:
1. Push changes to `001-physical-ai-textbook` branch
2. GitHub Actions workflow triggers automatically
3. Build verification runs (Markdown lint, code tests, Docusaurus build)
4. On success, deploy to GitHub Pages
5. Verify deployment at https://[username].github.io/humanoid_aibook/

---

## Phase 2: Implementation Tasks (Generated by /sp.tasks)

**Note**: Task breakdown will be created by the `/sp.tasks` command after this plan is approved. Expected task categories:

1. **Infrastructure Setup** (Docusaurus scaffold, GitHub Actions, dependencies)
2. **Module 1 Content Generation** (3 chapters on ROS 2)
3. **Module 2 Content Generation** (3 chapters on simulation)
4. **Module 3 Content Generation** (3 chapters on Isaac)
5. **Module 4 Content Generation** (3 chapters on VLA + capstone)
6. **Code Example Validation** (pytest tests, ROS 2 environment checks)
7. **Integration & QA** (cross-references, navigation, search, final review)
8. **Deployment** (GitHub Pages configuration, CI/CD verification)

---

## Post-Phase 1 Constitution Re-Check

**I. Content Fidelity** ✅ PASS
- Design strictly adheres to 4-module, 12-chapter structure
- No scope creep beyond specified technologies

**II. Clarity and Structure** ✅ PASS
- Chapter template enforces clear learning objectives and prerequisites
- Modular design supports independent chapter development

**III. Token Efficiency** ✅ PASS
- Template-based approach reduces generation tokens
- Chunked content creation strategy defined

**IV. Technical Correctness** ✅ PASS
- Code validation infrastructure defined (pytest, linting)
- Example templates enforce imports, dependencies, error handling

**V. Modularity and Reusability** ✅ PASS
- Chapter template supports standalone chapters
- Code examples reusable across chapters via static/code/ directory

**All constitutional requirements satisfied. Ready for task generation.**

---

## Architectural Decision Records (ADR)

**ADR Candidates Identified**:
1. ✅ **Docusaurus vs. Alternatives** - Documented in Phase 0 research
2. ✅ **Markdown/MDX for Content** - Documented in Phase 0 research
3. ✅ **GitHub Pages Deployment** - Documented in Phase 0 research

**Recommendation**: Create ADRs for above decisions if this becomes a reference project for future educational content generation.

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Code examples break with ROS 2 updates | High | Pin specific Humble/Iron versions; include version info in all examples |
| Token limits exceeded during generation | Medium | Phased generation with checkpoints; content chunking per chapter |
| Docusaurus build failures | Medium | CI/CD with build verification; local testing before deployment |
| Isaac Sim examples inaccessible to students | Low | Provide academic license guidance; alternative concepts for GPU-less users |
| Content readability below CEFR B1-B2 | Low | Use readability scoring tools; peer review for clarity |

---

## Next Steps

1. **Review and Approve Plan** - Stakeholder validation of approach
2. **Generate research.md** - Complete Phase 0 documentation
3. **Generate data-model.md** - Formalize content entity model
4. **Generate contracts/** - Chapter and code templates
5. **Generate quickstart.md** - Development workflow guide
6. **Run /sp.tasks** - Create detailed implementation task list
7. **Begin Implementation** - Start with infrastructure setup and Module 1

---

**Plan Status**: ✅ Complete - Ready for `/sp.tasks` command
**Branch**: `001-physical-ai-textbook`
**Spec**: [spec.md](./spec.md)
