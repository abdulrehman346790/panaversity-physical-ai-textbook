# Implementation Plan: Chapter 4 — Sensors & Perception (Camera, LiDAR, IMU)

**Branch**: `005-sensors-perception` | **Date**: 2025-11-30 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-sensors-perception/spec.md`

## Summary

This plan outlines the implementation for Chapter 4, which introduces students to robotic sensors (Camera, LiDAR, IMU) and their integration with ROS 2. The primary goal is to teach students how to work with sensor data, from simulation in Webots to visualization in RViz2, using Python for subscribing to sensor topics. The plan follows the guidelines and requirements established in the feature specification, prioritizing a beginner-friendly, hands-on learning experience.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: ROS 2 Jazzy, Webots, RViz2, Docusaurus
**Storage**: FileSystem (for documentation, Webots worlds, RViz2 configs)
**Testing**: `pytest` for ROS 2 nodes, manual testing for simulations and visualizations.
**Target Platform**: Linux, Windows
**Project Type**: Documentation and Examples (for an existing Docusaurus-based project)
**Performance Goals**: ROS 2 subscriber examples should have < 3 second startup time.
**Constraints**: All examples must be runnable within 3-5 minutes on a moderate laptop.
**Scale/Scope**: One educational chapter with two simulated labs and corresponding example code.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The project constitution at `.specify/memory/constitution.md` is currently a template. The following checks are placeholders and will need to be validated against a concrete constitution.

- **[ ] Principle Adherence**: Does the plan follow core project principles? (e.g., Test-First, Library-First)
- **[ ] Technology Stack**: Does the plan use approved technologies?
- **[ ] Process Compliance**: Does the workflow align with the defined development process?

## Project Structure

### Documentation & Examples (this feature)

The implementation will add the following structure to the project:

```text
docusaurus/book/
└── docs/
    └── 04-sensors/
        ├── index.md
        ├── camera.md
        ├── lidar.md
        ├── imu.md
        ├── rviz-visualization.md
        └── troubleshooting.md
examples/
└── ros2/
    └── sensors/
        ├── camera_subscriber.py
        ├── imu_subscriber.py
        ├── lidar_subscriber.py
        └── launch/
            └── sensor_subscribers.launch.py
webots/
└── worlds/
    ├── basic_sensor_demo.wbt
    └── advanced_sensor_demo.wbt
rviz/
└── 04-sensors.rviz
```

**Structure Decision**: The structure is based on the existing project layout, separating Docusaurus documentation from ROS 2 examples, Webots worlds, and RViz configurations. This maintains a clear separation of concerns.

## Implementation Phases

### Phase 0: Preparation & Environment Setup
- **Description**: Ensure the environment is ready to run sensor examples and RViz2 visualizations.
- **Tasks**:
  - Verify ROS 2 Jazzy installation.
  - Install RViz2 if missing.
  - Confirm Webots installation.
  - Create the directory structures defined in the "Project Structure" section.

### Phase 1: Documentation Structure Setup (Docusaurus)
- **Description**: Create the main docs layout for Chapter 4.
- **Tasks**:
  - Populate `docs/04-sensors/index.md` with an introduction.
  - Write content for `camera.md`, `lidar.md`, and `imu.md`.
  - Create the RViz2 visualization guide in `rviz-visualization.md`.
  - Develop the `troubleshooting.md` section.
  - Add a sensor comparison table in the index.

### Phase 2: Webots Simulation Examples
- **Description**: Build runnable Webots sensor demos.
- **Tasks**:
  - Create `webots/worlds/basic_sensor_demo.wbt` with a robot and a LiDAR.
  - Create `webots/worlds/advanced_sensor_demo.wbt` with a robot featuring Camera, LiDAR, and IMU.
  - Configure the Webots -> ROS 2 Jazzy bridge for all sensors.
  - Write a README for running the simulations.
  - Capture screenshots for the documentation.

### Phase 3: ROS 2 Python Examples (Sensor Subscribers)
- **Description**: Develop simple Python scripts for reading sensor data.
- **Tasks**:
  - Implement `camera_subscriber.py`, `imu_subscriber.py`, and `lidar_subscriber.py`.
  - Create a launch file to start all subscribers.
  - Add extensive inline comments to the Python scripts for educational purposes.

### Phase 4: RViz2 Visualization Workflow
- **Description**: Show students how to visualize sensor topics.
- **Tasks**:
  - Create the `rviz/04-sensors.rviz` configuration file.
  - Document the process for visualizing `LaserScan`, `Image`, and `Imu` topics.
  - Record a GIF or screenshots for the documentation.

### Phase 5: Troubleshooting & Educational Material
- **Description**: Add the beginner-friendly debugging section.
- **Tasks**:
  - Document common sensor issues and their fixes.
  - Create a diagram illustrating "How sensor data flows in ROS 2".

### Phase 6: Quality Assurance & Pedagogy Review
- **Description**: Ensure content works and meets non-functional requirements.
- **Tasks**:
  - Test simulations and examples on both Linux and Windows.
  - Verify that all functional and non-functional requirements from the spec are met.
  - Review all content for clarity and beginner-friendliness.

### Phase 7: Final Integration & RAG Indexing
- **Description**: Integrate Chapter 4 into the overall curriculum.
- **Tasks**:
  - Add Chapter 4 to the Docusaurus sidebar.
  - Add embeddings metadata for the RAG chatbot.
  - Link to and from other chapters.