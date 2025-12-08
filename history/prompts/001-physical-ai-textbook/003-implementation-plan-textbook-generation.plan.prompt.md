---
id: 003
title: Implementation plan for textbook generation
stage: plan
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: haroon traders
command: /sp.plan
labels: ["plan", "architecture", "docusaurus", "ros2", "textbook", "phase0", "phase1"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/plan.md
  - specs/001-physical-ai-textbook/research.md
  - specs/001-physical-ai-textbook/data-model.md
  - specs/001-physical-ai-textbook/quickstart.md
  - specs/001-physical-ai-textbook/contracts/chapter-template.md
  - specs/001-physical-ai-textbook/contracts/frontmatter-schema.json
  - specs/001-physical-ai-textbook/contracts/code-example-template.md
tests:
  - Constitution check validation (all gates PASS)
  - Front matter schema validation (JSON Schema compliant)
  - Chapter template structure verification
---

## Prompt

```text
Execution Plan for Building the Physical AI & Humanoid Robotics Textbook
Using Spec-Kit Plus + Claude Code + Docusaurus

## 1. Objective

Generate a complete Docusaurus-based textbook in **4 phases**, aligned with the /sp.specify file. Each phase outputs 1 module (3 chapters).

## 2. High-Level Strategy

1. Use Spec-Kit Plus to orchestrate generation.
2. Use Claude Code Router for low-token iterative content creation.
3. Build chapters in modular chunks to avoid token overflow.
4. After text generation, assemble the Docusaurus structure.
5. Deploy to GitHub Pages.

## 3. Work Phases

### **Phase 1 — Module 1 (ROS 2)**

* Generate 3 chapters on ROS 2 basics, humanoid control, and URDF.
* Produce Markdown pages: `module1/ch1.md`, `ch2.md`, `ch3.md`.
* Update sidebar + nav config.

### **Phase 2 — Module 2 (Gazebo & Unity)**

* Generate 3 chapters covering simulation, sensors, Unity.
* Create `module2` folder with 3 MD files.
* Validate images, diagrams, code blocks.

### **Phase 3 — Module 3 (NVIDIA Isaac)**

* Generate 3 chapters for Isaac Sim, Isaac ROS, and Nav2.
* Add diagrams for perception + navigation pipelines.
* Integrate into Docusaurus docs tree.

### **Phase 4 — Module 4 (VLA + Capstone)**

* Generate 3 chapters for Whisper, LLM planning, and capstone humanoid.
* Final integration and consistency review.
* Append capstone instructions + evaluation rubric.

## 4. Technical Plan

### Docusaurus Setup

* Scaffold project: `npx create-docusaurus@latest textbook classic`.
* Create folders: `/docs/module1`, `/module2`, `/module3`, `/module4`.
* Add sidebar entries for all 12 chapters.

### GitHub Pages Deployment

* Add `docusaurus.config.js` GitHub settings.
* Run:

  * `GIT_USER=<username> yarn deploy`
  * Or GitHub Actions workflow.

### Tool Workflow

* **Claude Code** → content generation, chunked writing, low-token refinement.
* **Spec-Kit Plus** → enforce structure, guide phase-by-phase execution.
* **Docusaurus** → book hosting.

## 5. Output Requirements

* Markdown only.
* Clear, concise chapters.
* Accurate robotics terminology (ROS 2, Gazebo, Isaac, VLA).
* No hallucination; follow course details strictly.

## 6. Success Criteria

* All 4 phases completed (12 chapters).
* Docusaurus site builds without errors.
* GitHub Pages successfully deployed.
* Capstone instructions included and technically correct.
```

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics textbook with complete Phase 0 (research) and Phase 1 (design & contracts) artifacts.

**Plan Components Generated**:

1. **plan.md** (530 lines):
   - Technical context (Docusaurus 2.x, ROS 2 Humble/Iron, Python 3.10+)
   - Constitution compliance validation (all 5 principles PASS)
   - Detailed project structure (docs/, static/, src/, specs/)
   - Phase 0 research framework (5 research areas)
   - Phase 1 content model and contracts
   - Risk assessment and ADR candidates

2. **research.md**:
   - Docusaurus vs. alternatives (GitBook, MkDocs, Sphinx rejected)
   - ROS 2 documentation standards (official style + educational enhancements)
   - Educational content structure (2000-3000 words/chapter, progressive complexity)
   - Multi-simulator documentation approach (Gazebo, Unity, Isaac)
   - GitHub Actions deployment workflow
   - Technology stack summary table

3. **data-model.md**:
   - 5 content entities defined (Module, Chapter, Section, CodeExample, FrontMatter)
   - ERD showing relationships
   - Validation rules for each entity
   - Content inventory (12 chapters, 50-62 code examples, 32K-36K words)
   - Validation checklist

4. **contracts/** (3 templates):
   - **chapter-template.md**: Full chapter structure with front matter, sections, code examples, challenges
   - **frontmatter-schema.json**: JSON Schema for YAML validation (required fields, patterns, constraints)
   - **code-example-template.md**: Standardized code example format with dependencies, explanation, troubleshooting

5. **quickstart.md**:
   - 10-part development workflow guide
   - Environment setup (Node.js, Yarn, ROS 2)
   - Chapter creation process
   - Validation and testing procedures
   - GitHub Pages deployment steps
   - Troubleshooting common issues

**Key Architectural Decisions**:
- Docusaurus 2.x for static site generation (React-based, GitHub Pages integration)
- Modular 4-phase content generation (token efficiency)
- Template-driven consistency (chapter, code example, front matter templates)
- CI/CD via GitHub Actions (automated build + deploy)
- Multi-simulator coverage (Gazebo, Unity, Isaac Sim)
- pytest for code validation (ROS 2 mocking without full environment)

**Constitution Compliance**:
- ✅ Content Fidelity: Strict 4-module, 12-chapter structure
- ✅ Clarity and Structure: Template-enforced modularity
- ✅ Token Efficiency: Phased generation with chunking strategy
- ✅ Technical Correctness: Code validation infrastructure defined
- ✅ Modularity: Independent chapters with cross-reference capability

## Outcome

- ✅ Impact: Complete architectural blueprint for textbook generation; clear path from design to deployment
- 🧪 Tests: Constitution check (all gates PASS), front matter schema validation (JSON Schema), template structure verification
- 📁 Files: 7 created (plan.md, research.md, data-model.md, quickstart.md, 3 contract templates)
- 🔁 Next prompts: `/sp.tasks` to generate detailed implementation task list; begin Module 1 content generation
- 🧠 Reflection: User-provided execution plan aligned perfectly with Spec-Kit Plus workflow; phased approach critical for managing 12-chapter scope

## Evaluation notes (flywheel)

- Failure modes observed: None - user input provided clear phase structure
- Graders run and results (PASS/FAIL): Constitution check PASS (all 5 principles satisfied), template validation PASS (complete coverage)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate template effectiveness during first chapter generation; consider automation scripts for repetitive chapter setup tasks
