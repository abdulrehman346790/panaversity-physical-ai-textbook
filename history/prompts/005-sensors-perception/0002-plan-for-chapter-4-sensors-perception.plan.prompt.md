---
id: {{ID}}
title: {{TITLE}}
stage: {{STAGE}}
date: {{DATE_ISO}}
surface: {{SURFACE}}
model: {{MODEL}}
feature: {{FEATURE}}
branch: {{BRANCH}}
user: {{USER}}
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

### PROMPT_TEXT

```text
/sp.plan
id: "chapter-4-sensors-and-perception"
title: "Implementation Plan — Chapter 4 (Sensors & Perception)"
owner: "panaversity-physical-ai"
status: "draft"
iteration: "v1"

phases:

  - id: PH4-0
    title: "Preparation & Environment Setup"
    description: "Ensure the environment is ready to run sensor examples and RViz2 visualizations."
    tasks:
      - "Verify ROS 2 Jazzy installation (required for sensor message types)."
      - "Install RViz2 if missing."
      - "Confirm Webots installation + test a world from Chapter 3."
      - "Add folder structure for Chapter 4 inside docs and examples."
      - "Decide primary demo sensor: Camera OR LiDAR (default: Camera)."

  - id: PH4-1
    title: "Documentation Structure Setup (Docusaurus)"
    description: "Create the main docs layout for Chapter 4."
    tasks:
      - "Create directory: docs/04-sensors/"
      - "Add intro file: docs/04-sensors/index.md"
      - "Add separate sections for each sensor:
            docs/04-sensors/camera.md
            docs/04-sensors/lidar.md
            docs/04-sensors/imu.md"
      - "Add RViz2 visualization guide: docs/04-sensors/rviz-visualization.md"
      - "Add troubleshooting section: docs/04-sensors/troubleshooting.md"
      - "Add comparison table (Camera vs LiDAR vs IMU)."

  - id: PH4-2
    title: "Webots Simulation Examples"
    description: "Build runnable Webots sensor demos."
    tasks:
      - "Create Webots world: webots/worlds/sensor_demo.wbt"
      - "Add a mobile robot with camera and IMU."
      - "Enable Webots → ROS 2 Jazzy bridge for:

            /camera/image
            /imu
            /lidar (optional)
        "
      - "Write README for running sensor simulation."
      - "Capture screenshots for documentation."

  - id: PH4-3
    title: "ROS 2 Python Examples (Sensor Subscribers)"
    description: "Develop simple Python scripts for reading sensor data."
    tasks:
      - "Create example package: examples/ros2/sensors/"
      - "Add node: camera_subscriber.py — reads sensor_msgs/Image"
      - "Add node: imu_subscriber.py — reads sensor_msgs/Imu"
      - "Add node: lidar_subscriber.py — reads sensor_msgs/LaserScan"
      - "Add launch file to start subscribers together."
      - "Include inline comments for students."

  - id: PH4-4
    title: "RViz2 Visualization Workflow"
    description: "Show students how to visualize sensor topics."
    tasks:
      - "Create RViz2 config: examples/ros2/sensors/sensors.rviz"
      - "Document how to load camera image"
      - "Document how to view LaserScan"
      - "Document IMU visualization (orientation)."
      - "Record GIF/screenshot for documentation."

  - id: PH4-5
    title: "Troubleshooting & Educational Material"
    description: "Add the beginner-friendly debugging section."
    tasks:
      - "Write common sensor issues:
            - No image appears
            - Topic not published
            - RViz2 warnings (TF missing)
            - FPS drop"
      - "Add fixes with commands & explanations."
      - "Add a diagram: 'How sensor data flows in ROS 2'."

  - id: PH4-6
    title: "Quality Assurance & Pedagogy Review"
    description: "Ensure content works and matches NFRs."
    tasks:
      - "Test Webots simulation on Linux + Windows."
      - "Test ROS 2 subscribers for performance (<3 sec startup)."
      - "Open sensors in RViz2 and confirm visibility."
      - "Validate compliance with functional & non-functional requirements."
      - "Ensure content remains beginner-friendly."

  - id: PH4-7
    title: "Final Integration & RAG Indexing"
    description: "Integrate Chapter 4 into the overall curriculum."
    tasks:
      - "Add Chapter 4 sidebar entries to Docusaurus."
      - "Add embeddings metadata for RAG."
      - "Link Chapter 4 with previous and upcoming chapters."
      - "Mark implementation as complete."

deliverables:
  - "docs/04-sensors/ (full chapter content)"
  - "Webots world: sensor_demo.wbt"
  - "ROS 2 Python nodes for sensor subscribers"
  - "RViz2 configuration"
  - "Screenshots/GIFs"
  - "Troubleshooting documentation"
  - "RAG metadata entries"

success_criteria:
  - "Student can open Webots world and see camera/IMU working."
  - "Student can visualize sensor topics in RViz2."
  - "Python subscriber examples run without errors."
  - "Documentation is clear and beginner-friendly."
  - "All FR/NFR/AC conditions from the specification are satisfied."

notes:
  - "Camera will be used as the primary demo unless you choose LiDAR."
  - "LiDAR example included but optional."
  - "All examples compatible with ROS 2 Jazzy."
```

### RESPONSE_TEXT

```text
The implementation plan for "Chapter 4 — Sensors & Perception" has been generated and is available at `specs/005-sensors-perception/plan.md`.

The process involved:
1.  Correcting the local git branch to `005-sensors-perception`.
2.  Running the setup script to prepare the planning files.
3.  Reading the feature specification and project constitution.
4.  Populating the implementation plan with technical context, project structure, and detailed implementation phases based on the user's input and the feature spec.
5.  Generating the following design artifacts:
    - `specs/005-sensors-perception/research.md`: Summarizing key decisions.
    - `specs/005-sensors-perception/data-model.md`: Documenting the ROS 2 message types.
    - `specs/005-sensors-perception/contracts/ros2-topics.md`: Defining the ROS 2 topic contracts.
    - `specs/005-sensors-perception/quickstart.md`: Providing a quickstart guide.
6.  Updating the agent's context with the technologies for this feature.
```

## Outcome

- ✅ Impact: {{OUTCOME_IMPACT}}
- 🧪 Tests: {{TESTS_SUMMARY}}
- 📁 Files: {{FILES_SUMMARY}}
- 🔁 Next prompts: {{NEXT_PROMPTS}}
- 🧠 Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
