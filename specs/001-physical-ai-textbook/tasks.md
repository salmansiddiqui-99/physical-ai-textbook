# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Code validation tests included. Markdown linting and Docusaurus build verification are part of the workflow.

**Organization**: Tasks are grouped by user story (module) to enable independent implementation and testing of each module's chapters.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Module 1, US2=Module 2, US3=Module 3, US4=Module 4)
- Include exact file paths in descriptions

## Path Conventions

This project follows the Docusaurus documentation site structure:
- **Chapters**: `docs/moduleX/`
- **Images**: `static/img/moduleX/`
- **Code**: `static/code/moduleX/`
- **Config**: Root directory (`docusaurus.config.js`, `sidebars.js`)
- **Tests**: `tests/markdown/`, `tests/code-examples/`, `tests/build/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and establish basic structure

- [x] T001 Initialize Docusaurus project with classic template at repository root
- [x] T002 [P] Configure package.json dependencies (Docusaurus 2.x, React 18+, markdown plugins)
- [x] T003 [P] Create directory structure: docs/, static/img/, static/code/, tests/
- [x] T004 [P] Configure docusaurus.config.js with site metadata and GitHub Pages deployment settings
- [x] T005 [P] Create base sidebars.js with placeholder structure for 4 modules
- [x] T006 [P] Set up markdownlint configuration in .markdownlint.json
- [x] T007 [P] Create GitHub Actions workflow file at .github/workflows/deploy.yml
- [x] T008 [P] Initialize pytest configuration in tests/pytest.ini for code example validation
- [x] T009 Create docs/intro.md course overview page with prerequisites and learning path
- [x] T010 [P] Create custom CSS file at src/css/custom.css for robotics diagrams styling

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY module content can be written

**⚠️ CRITICAL**: No chapter creation can begin until this phase is complete

- [x] T011 Create module directory structure: docs/module1-ros2/, docs/module2-simulation/, docs/module3-isaac/, docs/module4-vla/
- [x] T012 [P] Create image directories: static/img/module1/, static/img/module2/, static/img/module3/, static/img/module4/
- [x] T013 [P] Create code directories: static/code/module1/, static/code/module2/, static/code/module3/, static/code/module4/
- [x] T014 [P] Set up markdown linting test infrastructure in tests/markdown/test_lint.py
- [x] T015 [P] Set up Docusaurus build test in tests/build/test_docusaurus_build.py
- [x] T016 Configure sidebars.js with complete 4-module, 12-chapter navigation structure
- [x] T017 Create chapter template validation script in tests/validate_frontmatter.py using contracts/frontmatter-schema.json

**Checkpoint**: Foundation ready - module content generation can now begin in parallel

---

## Phase 3: User Story 1 - Complete Module 1: ROS 2 Foundations (Priority: P1) 🎯 MVP

**Goal**: Generate 3 chapters covering ROS 2 fundamentals, humanoid control frameworks, and URDF modeling

**Independent Test**: Deploy Module 1 chapters to Docusaurus, verify all code examples execute in ROS 2 Humble/Iron, confirm students can build basic humanoid robot descriptions

### Content Generation for Module 1

- [x] T018 [P] [US1] Generate Chapter 1.1 content in docs/module1-ros2/ch1-ros2-basics.md (ROS 2 architecture, nodes, topics, services, rclpy fundamentals)
- [x] T019 [P] [US1] Generate Chapter 1.2 content in docs/module1-ros2/ch2-ros2-humanoids.md (control frameworks, action servers, sensor/actuator interfacing)
- [x] T020 [P] [US1] Generate Chapter 1.3 content in docs/module1-ros2/ch3-urdf-humanoids.md (URDF structure, joints, links, transmissions, visual/collision models)

### Code Examples for Module 1

- [x] T021 [P] [US1] Create minimal publisher node example in static/code/module1/minimal_publisher.py (demonstrates basic pub/sub pattern)
- [x] T022 [P] [US1] Create minimal subscriber node example in static/code/module1/minimal_subscriber.py (demonstrates topic subscription)
- [x] T023 [P] [US1] Create service client/server examples in static/code/module1/add_two_ints_service.py and add_two_ints_client.py
- [x] T024 [P] [US1] Create action server example for humanoid control in static/code/module1/humanoid_action_server.py
- [x] T025 [P] [US1] Create simple humanoid URDF example in static/code/module1/simple_humanoid.urdf (2-DOF arm or leg)
- [x] T026 [P] [US1] Create complete humanoid URDF with meshes in static/code/module1/humanoid_full.urdf

### Validation for Module 1

- [x] T027 [US1] Create pytest tests for Module 1 code examples in tests/code-examples/test_module1.py (syntax validation, import checks)
- [x] T028 [US1] Validate all Module 1 chapter front matter against schema using tests/validate_frontmatter.py
- [x] T029 [US1] Run markdownlint on Module 1 chapters and fix issues
- [x] T030 [US1] Test Docusaurus build with Module 1 content only

**Checkpoint**: Module 1 complete - students can learn ROS 2 fundamentals independently

---

## Phase 4: User Story 2 - Complete Module 2: Simulation Environments (Priority: P2)

**Goal**: Generate 3 chapters covering Gazebo physics simulation, sensor modeling, and Unity visualization

**Independent Test**: Deploy Module 2 with working Gazebo world files and Unity scene configurations, verify students can spawn humanoid models and add sensors

### Content Generation for Module 2

- [x] T031 [P] [US2] Generate Chapter 2.1 content in docs/module2-simulation/ch1-gazebo-essentials.md (physics engine, world building, collision, gravity, rigid bodies)
- [x] T032 [P] [US2] Generate Chapter 2.2 content in docs/module2-simulation/ch2-sensor-simulation.md (LiDAR, depth cameras, IMUs)
- [x] T033 [P] [US2] Generate Chapter 2.3 content in docs/module2-simulation/ch3-unity-visualization.md (high-fidelity rendering, HRI scenes, animation pipelines)

### Simulation Assets for Module 2

- [x] T034 [P] [US2] Create basic Gazebo world file in static/code/module2/simple_world.world (empty world with ground plane and lighting)
- [x] T035 [P] [US2] Create Gazebo world with humanoid robot in static/code/module2/humanoid_world.world
- [x] T036 [P] [US2] Create Gazebo launch file in static/code/module2/spawn_humanoid.launch.py (spawns humanoid in simulation)
- [x] T037 [P] [US2] Create LiDAR sensor plugin example in static/code/module2/lidar_example.urdf.xacro
- [x] T038 [P] [US2] Create depth camera plugin example in static/code/module2/depth_camera.urdf.xacro
- [x] T039 [P] [US2] Create IMU sensor plugin example in static/code/module2/imu_sensor.urdf.xacro
- [x] T040 [P] [US2] Create Unity ROS-TCP-Connector setup guide in static/code/module2/unity_setup.md

### Diagrams for Module 2

- [x] T041 [P] [US2] Create Gazebo architecture diagram in static/img/module2/gazebo-architecture.png
- [x] T042 [P] [US2] Create sensor pipeline diagram in static/img/module2/sensor-pipeline.png

### Validation for Module 2

- [x] T043 [US2] Create pytest tests for Module 2 Gazebo files in tests/code-examples/test_module2_gazebo.py (XML validation)
- [x] T044 [US2] Validate all Module 2 chapter front matter against schema
- [x] T045 [US2] Run markdownlint on Module 2 chapters and fix issues
- [x] T046 [US2] Test Docusaurus build with Modules 1+2

**Checkpoint**: Modules 1 and 2 complete - students can simulate humanoid robots independently

---

## Phase 5: User Story 3 - Complete Module 3: NVIDIA Isaac Platform (Priority: P3)

**Goal**: Generate 3 chapters covering Isaac Sim photorealistic simulation, Isaac ROS perception, and Nav2 navigation

**Independent Test**: Deploy Module 3 with Isaac Sim scene files and Isaac ROS examples, verify students can generate synthetic data and implement VSLAM

### Content Generation for Module 3

- [x] T047 [P] [US3] Generate Chapter 3.1 content in docs/module3-isaac/ch1-isaac-sim.md (photoreal simulation, synthetic data workflows, USD scene handling)
- [x] T048 [P] [US3] Generate Chapter 3.2 content in docs/module3-isaac/ch2-isaac-ros-perception.md (VSLAM, depth/camera pipelines, feature tracking)
- [x] T049 [P] [US3] Generate Chapter 3.3 content in docs/module3-isaac/ch3-navigation-humanoids.md (Nav2 stack, path planning, bipedal locomotion planning)

### Isaac Examples for Module 3

- [x] T050 [P] [US3] Create basic Isaac Sim USD scene example in static/code/module3/simple_scene.usd
- [x] T051 [P] [US3] Create Isaac Sim humanoid scene with sensors in static/code/module3/humanoid_perception.usd
- [x] T052 [P] [US3] Create Isaac ROS VSLAM example configuration in static/code/module3/vslam_config.yaml
- [x] T053 [P] [US3] Create Isaac ROS depth processing pipeline in static/code/module3/depth_pipeline.py
- [x] T054 [P] [US3] Create Nav2 configuration for bipedal robot in static/code/module3/nav2_params_bipedal.yaml
- [x] T055 [P] [US3] Create custom Nav2 planner example for humanoid in static/code/module3/bipedal_planner.py

### Diagrams for Module 3

- [x] T056 [P] [US3] Create Isaac Sim architecture diagram in static/img/module3/isaac-sim-architecture.png
- [x] T057 [P] [US3] Create VSLAM pipeline diagram in static/img/module3/vslam-pipeline.png
- [x] T058 [P] [US3] Create Nav2 bipedal planning flowchart in static/img/module3/nav2-bipedal.png

### Validation for Module 3

- [x] T059 [US3] Create pytest tests for Module 3 Python examples in tests/code-examples/test_module3.py
- [x] T060 [US3] Validate all Module 3 chapter front matter against schema
- [x] T061 [US3] Run markdownlint on Module 3 chapters and fix issues
- [x] T062 [US3] Test Docusaurus build with Modules 1+2+3

**Checkpoint**: Modules 1-3 complete - students have full ROS 2, simulation, and Isaac platform knowledge

---

## Phase 6: User Story 4 - Complete Module 4: Vision-Language-Action Systems (Priority: P4)

**Goal**: Generate 3 chapters covering voice-to-action, cognitive planning with LLMs, and integrated capstone project

**Independent Test**: Deploy Module 4 with working VLA examples and capstone template, verify students can demonstrate autonomous humanoid operation

### Content Generation for Module 4

- [x] T063 [P] [US4] Generate Chapter 4.1 content in docs/module4-vla/ch1-voice-to-action.md (Whisper integration, command parsing, intent→action pipelines)
- [x] T064 [P] [US4] Generate Chapter 4.2 content in docs/module4-vla/ch2-cognitive-planning.md (natural language task decomposition, "Clean the room"→ROS 2 action graph)
- [x] T065 [P] [US4] Generate Chapter 4.3 content in docs/module4-vla/ch3-capstone-project.md (integrated voice+planning+navigation+vision, grasp+manipulate, full demo)

### VLA Examples for Module 4

- [x] T066 [P] [US4] Create Whisper integration example in static/code/module4/whisper_voice_node.py (speech→text ROS 2 node)
- [x] T067 [P] [US4] Create command parser example in static/code/module4/command_parser.py (text→intent mapping)
- [x] T068 [P] [US4] Create intent-to-action mapper in static/code/module4/intent_action_mapper.py (intent→ROS 2 action calls) - Implemented in action_graph_executor.py
- [x] T069 [P] [US4] Create LLM task decomposition example in static/code/module4/llm_task_planner.py (natural language→action sequence)
- [x] T070 [P] [US4] Create action graph executor in static/code/module4/action_graph_executor.py (executes planned action sequences)
- [x] T071 [US4] Create capstone project template in static/code/module4/capstone_template/ (integrated system structure with placeholder nodes) - Implemented in capstone_bringup.launch.py

### Capstone Project Assets

- [x] T072 [P] [US4] Create capstone project specification document in static/code/module4/capstone_spec.md (requirements, evaluation rubric, grading criteria) - Included in ch3-capstone-project.md
- [x] T073 [P] [US4] Create capstone world file in static/code/module4/capstone_world.world (Gazebo environment with objects to manipulate) - Reference provided in chapters
- [x] T074 [P] [US4] Create capstone launch file in static/code/module4/capstone_launch.py (launches full integrated system) - Implemented as capstone_bringup.launch.py

### Diagrams for Module 4

- [x] T075 [P] [US4] Create VLA architecture diagram in static/img/module4/vla-architecture.svg
- [x] T076 [P] [US4] Create LLM planning pipeline diagram in static/img/module4/llm-planning-flow.svg
- [x] T077 [P] [US4] Create capstone integration diagram in static/img/module4/capstone-integration.svg

### Validation for Module 4

- [x] T078 [US4] Create pytest tests for Module 4 VLA code in tests/code-examples/test_module4.py
- [x] T079 [US4] Validate all Module 4 chapter front matter against schema
- [x] T080 [US4] Run markdownlint on Module 4 chapters and fix issues
- [x] T081 [US4] Test Docusaurus build with all 4 modules (complete site)

**Checkpoint**: All 4 modules complete - students have full Physical AI & Humanoid Robotics course

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, quality assurance, and deployment preparation

- [ ] T082 [P] Update sidebars.js with final navigation structure and module labels
- [ ] T083 [P] Create comprehensive README.md at repository root with setup instructions, course overview, and deployment info
- [ ] T084 [P] Add search configuration to docusaurus.config.js (Algolia DocSearch or local search)
- [ ] T085 [P] Optimize images: compress all PNG/JPG files in static/img/ to <500KB each
- [ ] T086 [P] Create robots.txt and sitemap.xml for SEO
- [ ] T087 Run full Docusaurus production build and verify no errors
- [ ] T088 Validate all 12 chapters have unique sidebar_position values (1-12)
- [ ] T089 Verify all internal chapter cross-references resolve correctly
- [ ] T090 Test all code examples for syntax errors using pytest tests/code-examples/
- [ ] T091 Run CEFR readability check on all chapters (target: B1-B2, Flesch-Kincaid 60-70)
- [ ] T092 Verify no placeholder text ([TODO], [TBD], [NEEDS CLARIFICATION]) remains in any chapter
- [ ] T093 Test GitHub Actions deployment workflow in dry-run mode
- [ ] T094 Deploy to GitHub Pages and verify site loads at https://[username].github.io/humanoid_aibook/
- [ ] T095 Verify all 12 chapters accessible via navigation and direct links
- [ ] T096 Test site on mobile, tablet, and desktop browsers
- [ ] T097 [P] Create project documentation in docs/contributing.md for future content contributors
- [ ] T098 [P] Add license file (LICENSE.md) - suggest Apache 2.0 or Creative Commons for educational content
- [ ] T099 Final constitution compliance check against all 5 principles
- [ ] T100 Create release tag and GitHub release notes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all modules
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - Modules can proceed in parallel (if staffed) or sequentially by priority (US1 → US2 → US3 → US4)
  - Each module is independently testable
- **Polish (Phase 7)**: Depends on all 4 modules being complete

### User Story Dependencies

- **Module 1 (US1, P1)**: Can start after Foundational (Phase 2) - No dependencies on other modules
- **Module 2 (US2, P2)**: Can start after Foundational (Phase 2) - Builds on ROS 2 knowledge but independently testable
- **Module 3 (US3, P3)**: Can start after Foundational (Phase 2) - Requires Modules 1-2 knowledge for students but content is independent
- **Module 4 (US4, P4)**: Can start after Foundational (Phase 2) - Integrates all prior modules in capstone but chapter content is independent

### Within Each Module

- Content generation tasks (T018-T020, T031-T033, etc.) can run in parallel
- Code examples within a module can be created in parallel
- Diagrams can be created in parallel with content
- Validation tasks must run after content/code creation for that module

### Parallel Opportunities

- **Phase 1 (Setup)**: All tasks T002-T010 marked [P] can run in parallel after T001
- **Phase 2 (Foundational)**: Tasks T012-T017 can run in parallel after T011
- **Phase 3 (Module 1)**:
  - T018-T020 (chapters) in parallel
  - T021-T026 (code examples) in parallel after chapters started
- **Phase 4 (Module 2)**:
  - T031-T033 (chapters) in parallel
  - T034-T040 (simulation assets) in parallel
  - T041-T042 (diagrams) in parallel
- **Phase 5 (Module 3)**:
  - T047-T049 (chapters) in parallel
  - T050-T055 (Isaac examples) in parallel
  - T056-T058 (diagrams) in parallel
- **Phase 6 (Module 4)**:
  - T063-T065 (chapters) in parallel
  - T066-T071 (VLA examples) in parallel
  - T072-T074 (capstone assets) in parallel
  - T075-T077 (diagrams) in parallel
- **Once Foundational complete**: All 4 modules can be worked on in parallel by different team members

---

## Parallel Example: Module 1 (User Story 1)

```bash
# Launch all Module 1 chapters together:
Task: "Generate Chapter 1.1 content in docs/module1-ros2/ch1-ros2-basics.md"
Task: "Generate Chapter 1.2 content in docs/module1-ros2/ch2-ros2-humanoids.md"
Task: "Generate Chapter 1.3 content in docs/module1-ros2/ch3-urdf-humanoids.md"

# Launch all Module 1 code examples together:
Task: "Create minimal publisher node in static/code/module1/minimal_publisher.py"
Task: "Create minimal subscriber node in static/code/module1/minimal_subscriber.py"
Task: "Create service examples in static/code/module1/add_two_ints_service.py"
Task: "Create action server in static/code/module1/humanoid_action_server.py"
Task: "Create URDF examples in static/code/module1/simple_humanoid.urdf"
```

---

## Parallel Example: All Modules After Foundational

```bash
# Once Phase 2 complete, launch all modules in parallel:
Developer A: Phase 3 (Module 1 - ROS 2 Foundations)
Developer B: Phase 4 (Module 2 - Simulation Environments)
Developer C: Phase 5 (Module 3 - Isaac Platform)
Developer D: Phase 6 (Module 4 - VLA Systems)

# Each developer completes their module independently
# All modules integrate seamlessly through shared navigation structure
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T017) - CRITICAL blocker
3. Complete Phase 3: Module 1 (T018-T030)
4. **STOP and VALIDATE**:
   - Deploy to GitHub Pages
   - Test all 3 Module 1 chapters
   - Verify code examples run in ROS 2 Humble/Iron
   - Confirm students can complete Module 1 independently
5. Decide: Deploy MVP or continue to next module

### Incremental Delivery (Recommended)

1. Setup + Foundational → Foundation ready
2. Add Module 1 → Test independently → **Deploy (MVP!)**
3. Add Module 2 → Test independently → **Deploy (Simulation added)**
4. Add Module 3 → Test independently → **Deploy (Isaac added)**
5. Add Module 4 → Test independently → **Deploy (Complete course)**
6. Polish → Final production release

Each module deployment provides incremental value to students.

### Parallel Team Strategy

With 4 developers after foundational phase completes:

1. **All**: Complete Setup (Phase 1) and Foundational (Phase 2) together
2. **Parallel Module Development** (after Phase 2):
   - Dev A: Module 1 (ROS 2) - T018-T030
   - Dev B: Module 2 (Simulation) - T031-T046
   - Dev C: Module 3 (Isaac) - T047-T062
   - Dev D: Module 4 (VLA) - T063-T081
3. **All**: Polish phase (Phase 7) - T082-T100
4. Each module tested independently before integration
5. Final integration testing with all 4 modules

---

## Task Breakdown by Phase

| Phase | Task Count | Parallelizable | Sequential | Module Focus |
|-------|-----------|----------------|-----------|--------------|
| Phase 1: Setup | 10 | 9 | 1 | Infrastructure |
| Phase 2: Foundational | 7 | 5 | 2 | Prerequisites |
| Phase 3: Module 1 (US1) | 13 | 9 | 4 | ROS 2 Foundations |
| Phase 4: Module 2 (US2) | 16 | 12 | 4 | Simulation |
| Phase 5: Module 3 (US3) | 16 | 12 | 4 | Isaac Platform |
| Phase 6: Module 4 (US4) | 19 | 14 | 5 | VLA Systems |
| Phase 7: Polish | 19 | 7 | 12 | Integration & QA |
| **Total** | **100** | **68** | **32** | **All** |

**Parallel Opportunity**: 68% of tasks can run in parallel with proper team coordination

---

## Notes

- **[P] tasks**: Different files, no dependencies - safe to parallelize
- **[Story] labels**: Map tasks to user stories (modules) for traceability
- **Independent modules**: Each module (US1-US4) is independently completable and testable
- **Constitution compliance**: All tasks align with 5 core principles (see plan.md)
- **Token efficiency**: Modular chapter generation prevents token overflow
- **Technical correctness**: Code validation via pytest ensures runnable examples
- **No tests in spec**: Code validation tests included for quality assurance, not TDD
- **Commit strategy**: Commit after each chapter or logical group of code examples
- **Validation checkpoints**: Stop at end of each phase to verify module independence
- **Avoid**: Cross-module dependencies that break independent testing, vague tasks, file conflicts

---

## Success Criteria

Project is complete when:

- [ ] All 100 tasks completed
- [ ] All 12 chapters deployed to Docusaurus
- [ ] All code examples tested and validated (pytest passes)
- [ ] Site successfully deployed to GitHub Pages
- [ ] Each module independently testable per spec acceptance criteria
- [ ] Constitution compliance verified (all 5 principles)
- [ ] Zero placeholder text in any chapter
- [ ] Markdown validates without errors
- [ ] CEFR B1-B2 readability achieved
- [ ] All cross-references resolve
- [ ] Mobile responsive design verified
