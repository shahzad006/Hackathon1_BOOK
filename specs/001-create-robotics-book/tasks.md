# Tasks: Create Robotics Book

**Input**: Design documents from `/specs/001-create-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `- [ ] [TaskID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Book Content**: `book/`
- **Docusaurus Project**: `docusaurus/`

---

## Phase 1: Setup (Docusaurus Initialization)

**Purpose**: Project initialization and basic Docusaurus setup.

- [x] T001 Initialize a new Docusaurus project in the `docusaurus/` directory.
- [x] T002 Configure `docusaurus/docusaurus.config.js` with the book title "Physical AI & Humanoid Robotics", navbar, and footer.
- [x] T003 Create the `book/` directory for the book's content.
- [x] T004 Create the chapter directory `book/01-ros2/`.
- [x] T005 Update `docusaurus/sidebars.js` to include the structure for Chapter 1.

---

## Phase 2: Chapter 1 - The Robotic Nervous System (ROS 2) (Priority: P1) 🎯

**Goal**: Introduce readers to the fundamental concepts of ROS 2.

**Independent Test**: A reader can create a simple ROS 2 publisher and subscriber, and understand how they communicate.

### [US1] Lesson 1: Introduction to ROS 2
- [x] T101 [US1] Write the content for the introduction to ROS 2 in `book/01-ros2/01-introduction.md`. Explain what ROS 2 is and its importance in robotics.

### [US1] Lesson 2: ROS 2 Nodes
- [x] T102 [US1] Write the content for ROS 2 nodes in `book/01-ros2/02-ros2-nodes.md`. Include a Python code example of a simple ROS 2 node using `rclpy`.

### [US1] Lesson 3: ROS 2 Topics
- [x] T103 [US1] Write the content for ROS 2 topics in `book/01-ros2/03-ros2-topics.md`. Include Python code examples for a publisher and a subscriber.

### [US1] Lesson 4: ROS 2 Services
- [x] T104 [US1] Write the content for ROS 2 services in `book/01-ros2/04-ros2-services.md`. Include Python code examples for a service server and client.

### [US1] Lesson 5: ROS 2 Actions
- [x] T105 [US1] Write the content for ROS 2 actions in `book/01-ros2/05-ros2-actions.md`. Include Python code examples for an action server and client.

### [US1] Review and Polish
- [x] T106 [US1] Create diagrams (Mermaid/SVG) for Chapter 1 concepts and add them to `docusaurus/static/img/chapter1/`.
- [x] T107 [US1] Review all content and code examples in Chapter 1 for technical accuracy and clarity.

---

## Phase 3: Polish & Cross-Cutting Concerns

**Purpose**: Final touches and deployment preparation.

- [x] T201 Update `docusaurus/sidebars.js` to ensure all Chapter 1 lessons are correctly linked.
- [x] T202 Validate all internal and external links in the book.
- [x] T203 Run a full Docusaurus build to check for errors using `npm run build` in the `docusaurus/` directory.
- [x] T204 Configure GitHub Actions for deploying the Docusaurus site to GitHub Pages.

---

## Dependencies

- **Chapter 1** depends on **Phase 1: Setup**.

## Parallel Execution

- Within Chapter 1, the lessons (T101-T105) can be written in parallel to some extent, although they follow a logical sequence. I will mark them as not parallel for now.

## Implementation Strategy

1.  **Complete Phase 1**: Set up the Docusaurus project.
2.  **Complete Phase 2**: Implement all tasks for Chapter 1.
3.  **Validate**: Ensure Chapter 1 builds correctly and meets the success criteria from `spec.md`.
4.  **Complete Phase 3**: Polish the book and set up deployment.
