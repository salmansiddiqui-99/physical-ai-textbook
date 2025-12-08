# Quickstart Guide: Development Workflow
**Feature**: Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-08
**Phase**: 1 (Design & Contracts)

## Overview

This guide provides step-by-step instructions for setting up the development environment, creating textbook content, and deploying to GitHub Pages.

---

## Prerequisites

Before starting, ensure you have:

- **Operating System**: Ubuntu 22.04 LTS, macOS 12+, or Windows 11 with WSL2
- **Node.js**: Version 18.x or later ([download](https://nodejs.org/))
- **Yarn**: Package manager ([install](https://yarnpkg.com/getting-started/install))
- **Git**: Version control ([install](https://git-scm.com/downloads))
- **Text Editor**: VS Code recommended with Markdown extensions
- **ROS 2** (for code validation): Humble or Iron distribution ([install](https://docs.ros.org/en/humble/Installation.html))
- **Python**: 3.10+ with pip

**Recommended VS Code Extensions**:
- Markdown All in One
- markdownlint
- YAML
- Python (Microsoft)
- EditorConfig for VS Code

---

## Part 1: Initial Environment Setup

### Step 1: Clone the Repository

```bash
# Clone the repository
git clone <repository-url> humanoid_aibook
cd humanoid_aibook

# Checkout feature branch
git checkout 001-physical-ai-textbook
```

### Step 2: Install Docusaurus Dependencies

```bash
# Install Node.js dependencies
yarn install

# Verify installation
yarn --version
node --version
```

**Expected Output**:
```
yarn install v1.22.x
[1/4] Resolving packages...
[2/4] Fetching packages...
[3/4] Linking dependencies...
[4/4] Building fresh packages...
success Saved lockfile.
Done in X.XXs.
```

### Step 3: Initialize Docusaurus (First Time Only)

```bash
# If Docusaurus not yet initialized, scaffold project
npx create-docusaurus@latest . classic --typescript

# Answer prompts:
# - Project name: Physical AI & Humanoid Robotics
# - Template: classic
# - Use TypeScript: Yes (optional, can use JavaScript)
```

### Step 4: Start Development Server

```bash
# Start local development server
yarn start
```

Open browser to [http://localhost:3000](http://localhost:3000). You should see the Docusaurus welcome page.

**Hot Reload**: Changes to Markdown files will automatically reload in the browser.

---

## Part 2: Project Structure Overview

```
humanoid_aibook/
├── docs/                          # Documentation content (your chapters go here)
│   ├── intro.md                   # Course introduction
│   ├── module1-ros2/              # Module 1 chapters
│   ├── module2-simulation/        # Module 2 chapters
│   ├── module3-isaac/             # Module 3 chapters
│   └── module4-vla/               # Module 4 chapters
├── static/                        # Static assets
│   ├── img/                       # Images and diagrams
│   └── code/                      # Downloadable code examples
├── src/                           # Custom React components and pages
│   ├── css/                       # Custom styles
│   └── pages/                     # Landing page
├── specs/                         # Spec-Kit Plus artifacts
│   └── 001-physical-ai-textbook/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md          # This file
│       └── contracts/             # Templates
├── docusaurus.config.js           # Docusaurus configuration
├── sidebars.js                    # Sidebar navigation configuration
└── package.json                   # Node.js dependencies
```

---

## Part 3: Creating a New Chapter

### Step 1: Choose the Module

Determine which module your chapter belongs to:
- **Module 1**: `docs/module1-ros2/` (ROS 2 foundations)
- **Module 2**: `docs/module2-simulation/` (Gazebo & Unity)
- **Module 3**: `docs/module3-isaac/` (NVIDIA Isaac)
- **Module 4**: `docs/module4-vla/` (Vision-Language-Action)

### Step 2: Copy the Chapter Template

```bash
# Navigate to appropriate module directory
cd docs/module1-ros2/

# Copy the chapter template
cp ../../specs/001-physical-ai-textbook/contracts/chapter-template.md ch1-ros2-basics.md
```

### Step 3: Fill Front Matter

Edit the YAML front matter at the top of the chapter file:

```yaml
---
id: module1-ros2/ch1-ros2-basics
title: "ROS 2 Basics: Architecture, Nodes, and Communication"
sidebar_label: "ROS 2 Basics"
sidebar_position: 1
description: "Learn ROS 2 architecture fundamentals including nodes, topics, services, and the publish-subscribe pattern for robot communication."
keywords:
  - ROS 2
  - nodes
  - topics
  - pub/sub
  - rclpy
prerequisites:
  - "Python 3 basics (functions, classes)"
  - "Linux terminal commands (cd, ls, source)"
learning_objectives:
  - "Explain ROS 2 architecture and DDS middleware"
  - "Create and run ROS 2 nodes using rclpy"
  - "Implement publish-subscribe communication with topics"
estimated_time: "90 minutes"
---
```

**Key Rules**:
- `id` must match file path (without `.md`)
- `sidebar_position` must be unique (1-12)
- Learning objectives start with action verbs
- Description is 50-200 characters for SEO

### Step 4: Write Chapter Content

Follow the template structure:

1. **Learning Objectives**: 3-6 measurable outcomes
2. **Prerequisites**: What students need to know beforehand
3. **Introduction**: 2-3 paragraphs explaining why topic matters
4. **Sections**: 3-5 major sections with subsections
5. **Code Examples**: Use code example template for consistency
6. **Hands-On Project**: Practical application of concepts
7. **Challenge**: Self-assessment exercises
8. **Summary**: Key takeaways and commands
9. **Further Reading**: Links to official docs
10. **Next Steps**: Link to next chapter

**Writing Guidelines**:
- Max 4 sentences per paragraph (CEFR B1-B2 readability)
- Use bullet points for lists
- Add code comments explaining WHY, not just WHAT
- Include troubleshooting for common errors
- Keep chapter under 50KB Markdown

### Step 5: Add Code Examples

Use the code example template:

```markdown
## Example: Minimal Publisher Node

**Purpose**: Demonstrates basic ROS 2 node creating and publishing string messages to a topic

**Environment**: ROS 2 Humble

**Dependencies**:
\```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs
\```

**Code**:
\```python
#!/usr/bin/env python3
# [Full code with comments]
\```

**Code Explanation**:
[Step-by-step breakdown]

**Running the Code**:
\```bash
python3 minimal_publisher.py
\```

**Expected Output**:
\```
[Sample output]
\```

**Troubleshooting**:
| Issue | Solution |
|-------|----------|
| ... | ... |
```

---

## Part 4: Validation and Testing

### Step 1: Lint Markdown

```bash
# Install markdownlint (first time only)
npm install -g markdownlint-cli

# Lint all Markdown files
markdownlint docs/**/*.md

# Lint specific file
markdownlint docs/module1-ros2/ch1-ros2-basics.md
```

**Fix Common Issues**:
- Remove trailing spaces
- Use consistent heading hierarchy (no skipping levels)
- Add blank lines before/after code blocks
- Use language identifiers for code blocks

### Step 2: Validate Front Matter

```bash
# Install YAML validator (first time only)
pip3 install yamllint

# Extract and validate front matter
# (Manual step - check YAML between --- markers)
```

**Check Against Schema**:
- Ensure all required fields present (`id`, `title`, `sidebar_position`, `description`, `learning_objectives`)
- Verify `sidebar_position` is unique
- Confirm learning objectives start with action verbs

### Step 3: Test Code Examples

```bash
# Create test directory
mkdir -p tests/code-examples/module1

# Copy code example from chapter
# Extract code blocks to .py files

# Test with pytest
pytest tests/code-examples/module1/test_minimal_publisher.py

# OR manually test
source /opt/ros/humble/setup.bash
python3 minimal_publisher.py
```

### Step 4: Build Docusaurus Site

```bash
# Build production site
yarn build

# Check for errors
# If successful, output in build/ directory
```

**Common Build Errors**:
- **Broken Links**: `Error: Docs markdown link couldn't be resolved`
  - Fix: Correct relative paths to other chapters
- **Duplicate IDs**: `Duplicate docs id found`
  - Fix: Ensure each chapter has unique `id` in front matter
- **Invalid Front Matter**: `Error parsing front matter`
  - Fix: Check YAML syntax (proper indentation, quotes)

### Step 5: Preview Built Site

```bash
# Serve built site locally
yarn serve

# Open browser to http://localhost:3000
```

Verify:
- [ ] Chapter appears in sidebar navigation
- [ ] All internal links work
- [ ] Code blocks have syntax highlighting
- [ ] Images load correctly
- [ ] Search finds chapter keywords

---

## Part 5: Updating Sidebar Navigation

### Edit `sidebars.js`

```javascript
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      items: [
        'module1-ros2/ch1-ros2-basics',
        'module1-ros2/ch2-ros2-humanoids',
        'module1-ros2/ch3-urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module2-simulation/ch1-gazebo-essentials',
        'module2-simulation/ch2-sensor-simulation',
        'module2-simulation/ch3-unity-visualization',
      ],
    },
    // ... Module 3 and 4
  ],
};
```

**Verify**:
- Categories match module structure
- Items reference correct chapter `id` values
- Order matches `sidebar_position` numbering

---

## Part 6: Deployment to GitHub Pages

### Step 1: Configure Docusaurus for GitHub Pages

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course on ROS 2, simulation, and VLA systems',
  url: 'https://<username>.github.io',
  baseUrl: '/humanoid_aibook/',
  organizationName: '<username>',
  projectName: 'humanoid_aibook',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
  // ... other config
};
```

### Step 2: Set Up GitHub Actions Workflow

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - 001-physical-ai-textbook

jobs:
  deploy:
    runs-on: ubuntu-latest
    permissions:
      contents: write
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: yarn

      - name: Install dependencies
        run: yarn install --frozen-lockfile

      - name: Build website
        run: yarn build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### Step 3: Enable GitHub Pages

1. Go to repository **Settings** > **Pages**
2. Set **Source** to `gh-pages` branch
3. Click **Save**

### Step 4: Push Changes and Deploy

```bash
# Add all changes
git add .

# Commit with descriptive message
git commit -m "Add Chapter 1.1: ROS 2 Basics"

# Push to trigger deployment
git push origin 001-physical-ai-textbook
```

### Step 5: Verify Deployment

1. Go to **Actions** tab in GitHub repository
2. Wait for workflow to complete (green checkmark)
3. Visit `https://<username>.github.io/humanoid_aibook/`
4. Verify chapter appears and loads correctly

**Deployment Time**: Typically 2-5 minutes

---

## Part 7: Common Development Tasks

### Adding Images

```bash
# Create module-specific image directory
mkdir -p static/img/module1

# Add image file
cp ~/Downloads/ros2-architecture.png static/img/module1/

# Reference in Markdown
![ROS 2 Architecture](../../static/img/module1/ros2-architecture.png)
```

**Image Guidelines**:
- Use PNG for diagrams, JPEG for photos
- Compress images (<500KB each)
- Use descriptive filenames (no spaces)
- Include alt text for accessibility

### Adding Downloadable Code

```bash
# Create module code directory
mkdir -p static/code/module1

# Add code file
cp example_node.py static/code/module1/

# Link in chapter Markdown
Download: [example_node.py](../../static/code/module1/example_node.py)
```

### Updating Dependencies

```bash
# Update Docusaurus and plugins
yarn upgrade @docusaurus/core@latest @docusaurus/preset-classic@latest

# Verify no breaking changes
yarn build
```

---

## Part 8: Troubleshooting

### Issue: `Module not found` Error

**Cause**: Missing Node.js dependencies

**Solution**:
```bash
rm -rf node_modules yarn.lock
yarn install
```

### Issue: Port 3000 Already in Use

**Cause**: Another process using port 3000

**Solution**:
```bash
# Use different port
yarn start --port 3001

# OR kill process on port 3000 (Linux/macOS)
lsof -ti:3000 | xargs kill
```

### Issue: Changes Not Appearing

**Cause**: Browser caching

**Solution**:
- Hard refresh: `Ctrl+Shift+R` (Windows/Linux) or `Cmd+Shift+R` (macOS)
- Clear browser cache
- Restart dev server: `Ctrl+C` then `yarn start`

### Issue: GitHub Pages Shows 404

**Cause**: Incorrect `baseUrl` in config

**Solution**:
- Verify `docusaurus.config.js` has correct `baseUrl: '/humanoid_aibook/'`
- Ensure repository name matches project name
- Check GitHub Pages is enabled in repository settings

---

## Part 9: Best Practices

### Content Creation
- ✅ Write chapters incrementally (section by section)
- ✅ Test code examples as you write them
- ✅ Use chapter template for consistency
- ✅ Commit frequently with descriptive messages
- ❌ Don't write entire chapter without testing
- ❌ Don't skip front matter validation
- ❌ Don't use absolute URLs for internal links

### Code Quality
- ✅ Test all code examples in target environment
- ✅ Include error handling in examples
- ✅ Provide troubleshooting for common issues
- ✅ Use descriptive variable names
- ❌ Don't use deprecated ROS 2 APIs
- ❌ Don't hardcode paths or values
- ❌ Don't skip dependency documentation

### Documentation
- ✅ Link to official ROS 2/Gazebo/Isaac docs
- ✅ Explain WHY, not just WHAT
- ✅ Use simple language (CEFR B1-B2)
- ✅ Include diagrams for complex concepts
- ❌ Don't assume prior knowledge not listed in prerequisites
- ❌ Don't use jargon without defining it
- ❌ Don't skip the "Next Steps" section

---

## Part 10: Quick Reference

### Essential Commands

```bash
# Development
yarn start                  # Start dev server
yarn build                  # Build production site
yarn serve                  # Serve built site locally

# Validation
markdownlint docs/**/*.md   # Lint Markdown
yarn build                  # Test for build errors

# Deployment
git push origin 001-physical-ai-textbook  # Trigger GitHub Actions deploy

# Cleanup
rm -rf node_modules build .docusaurus  # Clean build artifacts
```

### File Paths

| Content Type | Location | Example |
|--------------|----------|---------|
| Chapters | `docs/moduleX/` | `docs/module1-ros2/ch1-ros2-basics.md` |
| Images | `static/img/moduleX/` | `static/img/module1/diagram.png` |
| Code | `static/code/moduleX/` | `static/code/module1/example.py` |
| Templates | `specs/001-physical-ai-textbook/contracts/` | `contracts/chapter-template.md` |

### Key Configuration Files

- `docusaurus.config.js`: Site metadata, plugins, deployment settings
- `sidebars.js`: Sidebar navigation structure
- `package.json`: Node.js dependencies and scripts
- `.github/workflows/deploy.yml`: GitHub Actions deployment workflow

---

## Next Steps

1. ✅ Environment set up
2. → Create your first chapter using this guide
3. → Validate and test locally
4. → Push to trigger GitHub Pages deployment
5. → Iterate: write → test → commit → deploy

**For Help**:
- Review templates in `specs/001-physical-ai-textbook/contracts/`
- Check existing chapters for examples
- Consult [Docusaurus documentation](https://docusaurus.io/docs)
- See [ROS 2 documentation](https://docs.ros.org/en/humble/)

**Quickstart Status**: ✅ Complete
**Date**: 2025-12-08
