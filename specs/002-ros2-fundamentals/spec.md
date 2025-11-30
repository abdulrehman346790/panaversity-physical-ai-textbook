# Feature Specification: Chapter 2 - ROS 2 Fundamentals

**Feature Branch**: `002-ros2-fundamentals`  
**Created**: 2025-11-30  
**Status**: Draft  
**Input**: User description: "This specification defines the content and structure for Chapter 2 of the Physical AI & Humanoid Robotics textbook. Focus is on ROS 2 architecture, nodes, topics, services, Python integration, and robot control."

## Summary
This feature adds Chapter 2 — "ROS 2 Fundamentals" — to the Physical AI & Humanoid Robotics textbook. The chapter will introduce ROS 2 concepts, architecture, and developer workflows with a focus on Python integration and using ROS 2 as middleware to connect AI agents for robot control. It targets students who have completed Chapter 1 and are new to ROS 2.

## Objectives
- Introduce ROS 2 as middleware for robot control
- Explain ROS 2 architecture: nodes, topics, services, and actions
- Demonstrate building ROS 2 packages with Python
- Show how to manage launch files and parameters
- Prepare students for connecting Python agents to ROS controllers

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Basic ROS 2 node communication (Priority: P1)
A student follows the chapter to create a simple ROS 2 package, write two Python nodes (publisher and subscriber), and validate that messages are exchanged.

**Why this priority**: This demonstrates the core communication pattern of ROS 2 and is foundational for all other content.

**Independent Test**: Create a vanilla ROS 2 package with a publisher and subscriber in Python and verify the subscriber receives messages published by the publisher.

**Acceptance Scenarios**:
1. **Given** a working ROS 2 environment, **When** student runs the publisher node, **Then** the subscriber receives messages at a rate > 0 for 10 seconds.
2. **Given** a message published on a topic, **When** the subscriber logs messages, **Then** message payload matches the expected content.

---

### User Story 2 - Services & Parameters (Priority: P2)
A student creates a Python service and a client that exchanges request/response pairs, and demonstrates setting and reading parameters from a launch file.

**Why this priority**: Services and parameters are essential for request/response workflows and configurable node behavior.

**Independent Test**: Build a simple echo service that returns request content, and a launch file that sets a parameter; confirm the service returns expected value and node reads parameter correctly.

**Acceptance Scenarios**:
1. **Given** a running service node, **When** the client sends a request, **Then** the client receives the correct response within a reasonable timeout.
2. **Given** a node launched with a configured parameter, **When** the node reads the parameter, **Then** it logs or uses the parameter value in behavior.

---

### User Story 3 - Agent integration with simulated humanoid (Priority: P3)
A student connects a Python-based agent to a simulated humanoid (in a supported simulator) through a ROS 2 node bridge to command joint motions and read back sensor data. The student validates the agent can publish control commands and react to sensor feedback.

**Why this priority**: Demonstrates realistic application of ROS 2 in robotics and shows AI agent integration with robot control.

**Independent Test**: With a provided simulation configuration (simple humanoid), an agent publishes normalized control commands and receives sensor topics; the simulation shows the robot moving according to commands.

**Acceptance Scenarios**:
1. **Given** the simulation and agent bridge are running, **When** the agent sends a movement command, **Then** the simulation receives the command and the robot model moves.
2. **Given** sensor topics in simulation, **When** agent subscribes to topic, **Then** agent receives sensor messages at expected frequency and logs or acts on them.

---

### Edge Cases
- Incorrect parameter data type passed from launch file
- Node starts but fails due to missing dependencies
- Topic name mismatch prevents messages from being received
- Network partition or firewall blocks DDS traffic in distributed setups

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: Chapter must describe core ROS 2 concepts (nodes, topics, services, actions) clearly and with diagrams.
- **FR-002**: Chapter must include a step-by-step lab for creating a ROS 2 package and Python nodes (publisher/subscriber).
- **FR-003**: Chapter must include a lab for creating and testing a service and setting node parameters via a launch file.
- **FR-004**: Chapter must include an example and guidance to connect a Python agent to ROS 2, including a simple simulated humanoid example.
- **FR-005**: Chapter must include troubleshooting guidance and common errors with suggested fixes.
- **FR-006**: All example code should be beginner-friendly, commented, and accompanied by quick test steps for verification.
- **FR-007**: Chapter content must be platform-agnostic for learning, but include a note on compatibility with common ROS 2 LTS versions.  

*FR-008: [NEEDS CLARIFICATION: ROS 2 LTS version for examples (e.g., Humble, Iron) should be specified to ensure code compatibility.]*

### Key Entities
- **Node**: A process that performs computation, publishes/subscribes to topics, serves services, or executes actions.
- **Topic**: Asynchronous publish/subscribe communication channel carrying typed messages.
- **Service**: Synchronous request/response interaction enabling RPC-like behavior.
- **Action**: Long-running goal-oriented interaction supporting feedback and cancellation.
- **Package**: Container for code, resources, and metadata for distribution.
- **Launch file**: Declarative structure to run multiple nodes with parameters.
- **Parameter**: Configurable value for node behavior.
- **Agent bridge**: A lightweight adapter that connects an AI agent to ROS 2 topics/services/actions.

## Success Criteria *(mandatory)*

### Measurable Outcomes
- **SC-001**: At least 80% of students following the chapter exercises can complete the P1 lab (publisher + subscriber) successfully.
- **SC-002**: At least 75% of students can implement the P2 lab (service + parameter) and observe correct request/response or parameter-driven behavior.
- **SC-003**: At least 60% of students can connect a simple agent to a simulated humanoid as described and observe expected movement or feedback in the simulator.
- **SC-004**: Chapter includes at least three diagrams covering ROS 2 architecture, inter-node communications, and an agent bridge flow, with accuracy validated by subject matter experts.

## Content Sections (Detailed)
- **Introduction to ROS 2**
  - Why ROS 2 is used for robotics and the concept of a robotic nervous system
  - Difference between ROS 1 and ROS 2 at a conceptual level
  - Minimal system architecture diagram and core concepts

- **ROS 2 Architecture**
  - Detailed explanation of nodes, topics, services, actions, QoS basics, and DDS role (conceptual)
  - Diagrams: DDS vs ROS 2 node interaction, topic flows
  - Python examples: simple publisher and subscriber in Python (educational code blocks with comments)

- **Building ROS 2 Packages**
  - Step-by-step: package skeleton, node files, resource files, package metadata (explain but do not force a specific toolchain)
  - Directory structure example and quick build/verify steps (as educational guidance)

- **Launch Files and Parameters**
  - Demonstrate how to define a launch file to run multiple nodes and set parameters
  - Example: launch file that configures node parameters and sets namespace
  - Python: how to read parameters in a node and use them

- **Connecting Python Agents**
  - How Python-based AI agents can publish/subscribe to ROS 2 topics and call services
  - Example use case: agent commanding a humanoid simulation to walk via joint or velocity commands
  - Considerations: message serialization, safety checks, frequency and command filtering

## Dependencies
- Docusaurus project must be initialized for textbook chapter (UI and docs site)
- Chapter 1 content must be completed and linked for prerequisite concepts
- RAG chatbot indexing mechanism for Chapter 2 must be in place (metadata expected)

## Assumptions
- The target audience has basic programming literacy and completed Chapter 1.
- The Docusaurus site will render markdown with code blocks and images.
- ROS 2 environment and a compatible simulator will be available in lab instructions or optional remote lab.
- Students have Python 3.10+ (or the minimum ROS 2-supported Python version) available to use with provided examples.

## Out of Scope
- Deep coverage of DDS internals, ROS 2 core implementation details, or advanced optimization patterns
- Platform-specific deployments beyond demonstrating compatibility notes (e.g., vendor-locked middleware)

## Notes & Deliverables
- Provide diagrams (SVG or PNG) where applicable and include alt text in the doc for accessibility
- Include beginner-friendly commented Python examples
- Provide quick troubleshooting tips and likely error patterns with fixes

## [NEEDS CLARIFICATION] Markers
- A single marker remains for ROS 2 LTS version selection: *FR-008* (see above)

## Next steps
- Confirm target ROS 2 LTS (Q1) and simulator family (Q2), then proceed to `/sp.plan` for chapter content drafting and developer tasks.

---

*Document prepared based on the user description and chapter draft inputs.*
