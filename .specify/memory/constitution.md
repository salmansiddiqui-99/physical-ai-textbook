<!--
Sync Impact Report
==================
Version change: [UNVERSIONED] → 1.0.0
Reason: Initial constitution establishment for Physical AI & Humanoid Robotics Textbook project

Modified principles: N/A (initial version)
Added sections:
  - All 10 sections from Purpose through Completion Criteria

Removed sections: N/A

Templates requiring updates:
  ✅ .specify/templates/plan-template.md (to be verified)
  ✅ .specify/templates/spec-template.md (to be verified)
  ✅ .specify/templates/tasks-template.md (to be verified)
  ⚠ .specify/templates/commands/*.md (pending verification)

Follow-up TODOs:
  - Verify template alignment with new constitution principles
  - Ensure Docusaurus configuration reflects technical rules
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## 1. Purpose

This constitution defines the rules for an AI agent that generates a Docusaurus textbook titled **"Physical AI & Humanoid Robotics Course."**

The agent MUST produce accurate, structured, and technically correct educational content aligned with modern robotics frameworks and pedagogical best practices.

## 2. Core Principles

### I. Content Fidelity
Follow the provided course content exactly. No additions, omissions, or interpretations beyond the specified modules: ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA systems, and Humanoid Robotics.

**Rationale**: Educational materials demand precision. Deviation creates confusion and breaks learning continuity.

### II. Clarity and Structure
Prioritize clarity, accuracy, modularity, and technical correctness in all outputs. Keep writing concise, structured, and instructional.

**Rationale**: Students require unambiguous explanations. Complex topics demand simple language and logical organization.

### III. Token Efficiency
Produce content optimized for low-token environments. Avoid verbosity, repetition, and unnecessary elaboration.

**Rationale**: AI generation incurs costs. Conciseness ensures scalability and faster iteration cycles.

### IV. Technical Correctness
All code MUST be correct and runnable. Maintain consistency with ROS 2 Humble/Iron, Gazebo, Isaac Sim, and Jetson Orin. Follow standard robotics terminology.

**Rationale**: Incorrect code damages trust and learning outcomes. Standards ensure portability and industry alignment.

### V. Modularity and Reusability
Each module, chapter, and code example MUST be self-contained and independently usable.

**Rationale**: Modular content supports flexible learning paths and easier maintenance.

## 3. Scope of Work

The agent MUST generate:
- All book pages, modules, and chapters
- Technical explanations aligned with course modules
- Spec-Kit Plus files: specification, plan, tasks, implementation
- Capstone project description

The agent MUST NOT:
- Add content outside defined modules
- Introduce dependencies not in the tech stack
- Generate personal, harmful, or irrelevant content

## 4. Style Rules

### Writing Standards
- Use simple, direct English (CEFR B1-B2 level)
- Use short paragraphs (max 4 sentences)
- Prefer bullet points over prose for lists and steps
- Avoid passive voice except when describing system behavior
- Keep examples minimal unless explicitly requested

### Formatting Standards
- Markdown only for book content
- YAML/JSON/Markdown for project files
- Code blocks MUST include language identifiers
- Headings MUST follow semantic hierarchy (H1 → H2 → H3, no skips)

## 5. Technical Rules

### Code Quality
- All code MUST execute without errors in specified environments
- Use consistent naming conventions (ROS 2 snake_case, Python PEP 8)
- Include necessary imports and dependencies
- Provide brief inline comments for non-obvious logic

### Technology Stack Constraints
- ROS 2: Humble/Iron distributions only
- Simulators: Gazebo Classic/Fortress, Unity with ROS integration, Isaac Sim
- Hardware references: NVIDIA Jetson Orin, standard sensors (LiDAR, cameras, IMU)
- VLA frameworks: As specified in course modules only

### Terminology Standards
- Use official ROS 2 terminology (node, topic, service, action)
- Follow IEEE/ISO robotics standards where applicable
- Define domain-specific terms on first use

## 6. Safety & Constraints

### Content Restrictions
- NO personal opinions or subjective evaluations
- NO speculation on hardware/features not in scope
- NO external assumptions beyond provided assignment
- NO political, religious, or culturally sensitive examples

### Resource Constraints
- Respect token limits: compress explanations, prioritize clarity over completeness
- Reuse definitions and concepts via cross-references
- Summarize lengthy processes with links to external docs

## 7. Output Format Rules

### Book Content
- Markdown files only
- Front matter YAML for Docusaurus metadata
- Consistent heading structure across chapters
- Code examples in fenced blocks with syntax highlighting

### Project Files
- YAML for configuration and metadata
- JSON for structured data exchanges
- Markdown for documentation and plans

### Response Protocol
- NEVER include explanations unless explicitly asked
- Output ONLY requested artifacts
- Use structured formats (lists, tables, sections) over paragraphs

## 8. Agent Behavior Rules

### Execution Discipline
- Always stay within assignment context
- Prefer smallest valid interpretation when ambiguous
- Choose structured outputs over prose
- Validate against constitution before finalizing

### Error Handling
- If spec unclear, ask targeted clarifying questions (max 3)
- If multiple valid approaches exist, present options with tradeoffs
- If blocked, state constraint and request guidance
- NEVER invent requirements or features

## 9. Project Objective

Create a complete textbook and project artifacts that teach:
- Physical AI fundamentals
- ROS 2 development and integration
- Simulation environments (Gazebo, Unity, Isaac)
- Vision-Language-Action (VLA) systems
- Humanoid robotics applications

Target audience: Advanced undergraduate/graduate students with programming experience and basic robotics knowledge.

## 10. Completion Criteria

The project is complete when ALL of the following are satisfied:

**Content Deliverables**
- [ ] Docusaurus site structure generated
- [ ] All modules and weekly content written
- [ ] Capstone project description included
- [ ] Code examples tested and validated

**Spec-Kit Plus Deliverables**
- [ ] Feature specifications completed
- [ ] Implementation plans documented
- [ ] Task lists generated with acceptance criteria
- [ ] Prompt History Records (PHRs) captured for all major work

**Quality Gates**
- [ ] No placeholder content ([TODO], [TBD], etc.)
- [ ] All code blocks execute in target environments
- [ ] Cross-references resolve correctly
- [ ] Markdown validates without errors

**Review & Validation**
- [ ] Constitution compliance verified
- [ ] Technical accuracy review completed
- [ ] Pedagogical flow validated
- [ ] Token efficiency targets met

## Governance

### Amendment Process
1. Propose changes with rationale and impact analysis
2. Update version following semantic versioning (MAJOR.MINOR.PATCH)
3. Document in Sync Impact Report (HTML comment at file top)
4. Propagate changes to dependent templates and command files
5. Create ADR for significant governance changes

### Versioning Policy
- **MAJOR**: Breaking changes to principles, scope redefinitions
- **MINOR**: New principles, expanded sections, new constraints
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

### Compliance Review
- All specs MUST reference applicable constitution sections
- All tasks MUST align with technical rules and style standards
- All PRs MUST verify constitution compliance before merge
- Agents MUST create PHRs for constitution amendments

### Dependency Management
Templates requiring alignment with this constitution:
- `.specify/templates/plan-template.md`
- `.specify/templates/spec-template.md`
- `.specify/templates/tasks-template.md`
- `.specify/templates/commands/*.md`

Runtime guidance files:
- `README.md`
- `docs/**/*.md`
- `CLAUDE.md` (agent-specific instructions)

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08
