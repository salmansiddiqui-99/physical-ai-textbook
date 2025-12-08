# Research & Technology Decisions
**Feature**: Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-08
**Phase**: 0 (Research & Design Decisions)

## Overview

This document consolidates research findings and technology decisions for building a Docusaurus-based educational textbook covering ROS 2, simulation environments, NVIDIA Isaac, and Vision-Language-Action systems.

---

## 1. Docusaurus Best Practices for Technical Documentation

### Decision: Docusaurus 2.x with Classic Theme

**Research Findings**:
- **Sidebar Organization**: Multi-level sidebar with collapsible categories best practice for courses with 4+ modules
- **Front Matter Schema**: Educational content requires custom fields beyond default (learning_objectives, prerequisites, estimated_time)
- **Code Block Features**: Docusaurus supports line highlighting (`{1,4-6}`), title attributes, and language-specific syntax highlighting out-of-box
- **Search Configuration**: Algolia DocSearch (free for open-source) provides best-in-class technical term search; local search via `@easyops-cn/docusaurus-search-local` for offline use

**Rationale**:
- Chosen over GitBook (vendor lock-in, limited customization), MkDocs Material (Python ecosystem, less React flexibility), Sphinx (steep learning curve, restructuredText)
- React-based extensibility allows future interactive components (simulated robot visualizations)
- Strong GitHub Pages integration with official deployment GitHub Action
- Active robotics documentation community (ROS 2 docs use similar stack)

**Alternatives Considered**:
- **GitBook**: Rejected due to paid tiers for private repos, limited control over build process
- **MkDocs Material**: Rejected due to Python dependency (project uses Node.js), less community support for robotics
- **Sphinx**: Rejected due to reStructuredText complexity vs. Markdown simplicity for educational content

**Implementation Notes**:
- Use `docusaurus-plugin-content-docs` for multi-version support (future ROS distributions)
- Enable `prism-additional-languages` for URDF/XML, YAML, Bash syntax highlighting
- Configure custom CSS for robotics diagrams (`.robot-diagram` class)

---

## 2. ROS 2 Documentation Standards

### Decision: Follow Official ROS 2 Style with Educational Enhancements

**Research Findings**:
- **Code Example Format**: ROS 2 docs use `#!/usr/bin/env python3` shebang, class-based nodes, comprehensive docstrings
- **Package Dependencies**: `package.xml` format for ROS 2 packages; `rosdep` for system dependencies
- **URDF/Xacro Best Practices**: Use xacro macros for reusable components; separate visual/collision meshes; document joint limits
- **Action Server Patterns (Humble/Iron)**: Use `rclpy.action.ActionServer` with goal/feedback/result callbacks; always implement cancellation

**Rationale**:
- Consistency with official ROS 2 documentation reduces student confusion
- Educational enhancements (more verbose comments, step-by-step explanations) align with CEFR B1-B2 readability target
- Version-specific examples (Humble/Iron) prevent API compatibility issues

**Alternatives Considered**:
- **Simplified Examples**: Rejected - students need production-ready patterns, not toy code
- **ROS 1 Compatibility**: Rejected per constitutional constraint (ROS 2 Humble/Iron only)

**Implementation Notes**:
- Pin package versions in examples: `rclpy==3.3.7` (Humble), `rclpy==5.1.0` (Iron)
- Include `colcon build --symlink-install` in all build instructions
- Provide Docker images for students without Linux: `osrf/ros:humble-desktop-full`

---

## 3. Educational Content Structure

### Decision: Modular Chapters with Progressive Complexity

**Research Findings**:
- **Chapter Length**: Optimal online technical content is 2000-3000 words (~15-20min reading + 30-40min hands-on)
- **Code-to-Explanation Ratio**: Best practice is 30% code, 70% explanation for beginner content; 50/50 for advanced
- **Progressive Complexity**: "Spiral curriculum" approach - revisit concepts with increasing depth across modules
- **Assessment Integration**: Since interactive exercises are out-of-scope, use "Challenge" sections with expected outputs for self-assessment

**Rationale**:
- Shorter chapters (<50KB Markdown) align with token efficiency and mobile readability
- Progressive complexity supports heterogeneous student backgrounds (some know ROS 1, others complete beginners)
- Self-assessment challenges provide learning checkpoints without auto-grading infrastructure

**Alternatives Considered**:
- **Long-Form Chapters**: Rejected - exceeds 50KB constraint, reduces modularity
- **Interactive Jupyter Notebooks**: Rejected - out of scope, requires backend infrastructure

**Implementation Notes**:
- Module 1 chapters: 40% code, 60% explanation (beginner-focused)
- Module 3-4 chapters: 50% code, 50% explanation (assumes Module 1-2 foundation)
- Each chapter ends with "Test Your Understanding" section (3-5 challenges with solution hints)

---

## 4. Simulation Environment Documentation

### Decision: Multi-Simulator Approach with Environment-Specific Guides

**Research Findings**:
- **Gazebo Documentation**: Fortress (latest LTS) uses SDF 1.9; best practice is separate world files per tutorial with inline comments
- **Unity ROS Integration**: ROS-TCP-Connector is community standard; requires separate Unity project (not embedded in docs)
- **Isaac Sim USD Structure**: NVIDIA Omniverse Kit 105+ required; USD assets should reference official Isaac library (`http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/`)
- **Sensor Plugin Parameters**: Gazebo uses `libgazebo_ros_ray_sensor.so` for LiDAR; document all critical params (samples, range, resolution)

**Rationale**:
- Multi-simulator coverage matches spec requirements (Gazebo, Unity, Isaac)
- Environment-specific guides prevent confusion (e.g., Gazebo SDF vs. Isaac USD)
- Reference official asset libraries reduces maintenance burden (no custom mesh hosting)

**Alternatives Considered**:
- **Single Simulator**: Rejected - spec requires coverage of all three
- **Custom Simulator**: Rejected - out of scope per constitution

**Implementation Notes**:
- Provide Gazebo `.world` files in `/static/code/module2/` for download
- Unity chapter links to separate GitHub repo with ROS-TCP-Connector setup (avoid embedding large Unity projects)
- Isaac Sim examples assume USD Composer 2023.1.0+ (free academic license)

---

## 5. GitHub Pages Deployment

### Decision: GitHub Actions with Official Docusaurus Deploy Action

**Research Findings**:
- **Workflow**: Official `peaceiris/actions-gh-pages@v3` action handles build + deploy in single step
- **Custom Domain**: Requires `CNAME` file in `/static/` and DNS configuration (optional for this project)
- **Build Optimization**: Enable production build with `NODE_ENV=production` reduces bundle size ~40%
- **Asset Optimization**: Use `@docusaurus/plugin-ideal-image` for responsive images; manual SVG optimization for diagrams

**Rationale**:
- GitHub Actions free for public repositories, native integration
- Official Docusaurus action reduces custom script maintenance
- Build optimization critical for <3s page load target

**Alternatives Considered**:
- **Manual Deployment**: Rejected - error-prone, no CI/CD
- **Netlify/Vercel**: Rejected - GitHub Pages sufficient for static site, free tier

**Implementation Notes**:
- Workflow trigger: `push` to `001-physical-ai-textbook` branch
- Build command: `yarn build` (production mode)
- Deploy to `gh-pages` branch with `CNAME` preservation
- Add build status badge to README

---

## Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| **Static Site Generator** | Docusaurus | 2.4.0+ | React-based, best GitHub Pages integration, MDX support |
| **Content Format** | Markdown + MDX | N/A | Universal support, simple syntax, React component embedding |
| **Code Examples (Runtime)** | Python, Bash, YAML, XML | Python 3.10+ | ROS 2 primary language; multi-language support needed |
| **ROS 2 Distribution** | Humble Hawksbill, Iron Irwini | Humble: May 2022, Iron: May 2023 | LTS + latest stable per spec |
| **Simulation (Physics)** | Gazebo Classic, Gazebo Fortress | Classic: 11.x, Fortress: LTS | Community standard, ROS 2 integration |
| **Simulation (Rendering)** | Unity | 2021.3 LTS+ | High-fidelity visualization, ROS-TCP-Connector support |
| **AI Simulation** | NVIDIA Isaac Sim | 2023.1.0+ | Photoreal rendering, synthetic data, Isaac ROS integration |
| **Code Validation** | pytest | 7.x+ | Python unit testing, ROS 2 mocking support |
| **Markdown Linting** | markdownlint-cli | 0.35+ | Enforce consistent formatting |
| **CI/CD** | GitHub Actions | N/A | Free for public repos, official Docusaurus action |
| **Deployment** | GitHub Pages | N/A | Free static hosting, custom domain support |

---

## Open Questions Resolved

| Question | Resolution |
|----------|------------|
| **How to handle Isaac Sim's GPU requirement?** | Document academic license path; provide conceptual explanations with screenshots for GPU-less students |
| **Should code examples be tested in CI?** | Yes - use pytest with ROS 2 mocking for fast validation without full environment |
| **Custom domain for GitHub Pages?** | No - use default `<username>.github.io/humanoid_aibook/` to avoid DNS configuration dependency |
| **Versioning strategy for future ROS distributions?** | Use Docusaurus versioning feature; current scope limited to Humble/Iron per spec |

---

## Next Steps

1. ✅ Research complete - all technology decisions documented
2. → Generate `data-model.md` with content entity model
3. → Generate `contracts/` with chapter templates
4. → Generate `quickstart.md` with development workflow
5. → Proceed to Phase 2 task generation (`/sp.tasks`)

**Research Status**: ✅ Complete
**Date**: 2025-12-08
