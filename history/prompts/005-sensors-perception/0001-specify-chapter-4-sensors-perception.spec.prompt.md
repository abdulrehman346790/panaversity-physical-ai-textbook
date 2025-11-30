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
id: "chapter-4-sensors-and-perception" title: "Chapter 4 — Sensors & Perception (Camera, LiDAR, IMU)" owner: "panaversity-physical-ai" contributors: - "Abdul Rehman" - "AI Agent (SpecifyPlus)" status: "draft" iteration: "v1" objectives: - Teach students how robots sense and perceive the environment. - Introduce the main types of robotic sensors: Camera, LiDAR, IMU. - Show students how ROS 2 handles sensor data using topics. - Enable students to visualize sensor streams in RViz2. - Provide simulation-based demos for camera, IMU, and LiDAR. - Prepare students for later chapters on robot navigation and AI agent integration. audience: "Beginner → Intermediate" scope: in_scope: - Overview of robot sensors (Camera, LiDAR, IMU) - ROS 2 sensor messages (Image, LaserScan, Imu) - Visualizing sensor data in RViz2 - Running simulated sensors in Webots - Running real or mock ROS 2 sensor nodes - Simple Python examples for reading sensor data - Understanding sensor noise & limitations (basic) out_of_scope: - Sensor fusion algorithms (EKF, UKF) - SLAM or Navigation algorithms - Deep learning on camera data - Real hardware sensor calibration user_stories: - id: US-4-01 story: "As a beginner, I want to understand how robots perceive the world." - id: US-4-02 story: "As a student, I want to visualize camera and LiDAR data to understand sensors better." - id: US-4-03 story: "As a ROS 2 learner, I want to subscribe to sensor topics using Python." - id: US-4-04 story: "As an AI student, I want to simulate sensor input before training/control." - id: US-4-05 story: "As a beginner, I want troubleshooting help if sensor data is not appearing." functional_requirements: - id: FR-4-001 requirement: "Must explain Camera, LiDAR, and IMU with diagrams or screenshots." - id: FR-4-002 requirement: "Must provide runnable ROS 2 example nodes for reading sensor topics." - id: FR-4-003 requirement: "Must include at least one Webots world using camera or LiDAR." - id: FR-4-004 requirement: "Must demonstrate viewing sensor data in RViz2." - id: FR-4-005 requirement: "Must include code for subscribing to Image, LaserScan, or Imu messages." - id: FR-4-006 requirement: "Must describe common sensor issues (FPS drops, noise, missing drivers)." - id: FR-4-007 requirement: "Must connect Webots simulated sensors to ROS 2 Jazzy topics." - id: FR-4-008 requirement: "Must include at least 3 troubleshooting examples with fixes." non_functional_requirements: - id: NFR-4-001 requirement: "Content must be beginner-friendly." - id: NFR-4-002 requirement: "Examples must run within 3–5 minutes on a moderate laptop." - id: NFR-4-003 requirement: "Sensor demos must be fully compatible with ROS 2 Jazzy." - id: NFR-4-004 requirement: "Simulated sensors must use Webots as the primary simulator." acceptance_criteria: - AC-4-001: Student can open Webots world with at least one working sensor. - AC-4-002: Student can run a Python script to read Camera or IMU or LiDAR data. - AC-4-003: Sensor data must appear in RViz2. - AC-4-004: At least one diagram or screenshot is provided. - AC-4-005: Troubleshooting section is included. open_questions: - OQ-4-001: Should we use Camera or LiDAR as the primary demo sensor? - OQ-4-002: Should the example include IMU filtering or keep it basic? - OQ-4-003: Should we provide 2 Webots worlds (basic + advanced) or only 1? assumptions: - Students have completed Chapter 3 (Simulation basics). - Students know how to run ROS 2 nodes. - Students have RViz2 installed. notes: - Camera is easy to demonstrate visually. - LiDAR is essential for later navigation chapters. - IMU data is required for robot stabilization and locomotion.
```

### RESPONSE_TEXT

```text
The feature specification for "Chapter 4 — Sensors & Perception (Camera, LiDAR, IMU)" has been successfully generated as `specs/005-sensors-perception/spec.md`.
A new branch `005-sensors-perception` was created.
Initial clarification questions were asked and resolved, updating the spec with the user's choices:
- Primary demo sensor: LiDAR
- IMU example complexity: Basic
- Number of Webots worlds: Two worlds (basic and advanced)
The specification quality checklist `specs/005-sensors-perception/checklists/requirements.md` has also been created and updated to reflect that all clarifications are resolved.
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
