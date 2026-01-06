# Tasks: Physical AI & Humanoid Robotics Documentation Site

**Input**: Design documents from `/specs/001-docs-site/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/
**Tests**: Not explicitly requested in feature specification - documentation build verification serves as testing

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/`, `docusaurus.config.ts`, `sidebars.ts`
- **Static assets**: `static/img/`
- **Configuration**: `.specify/` (existing templates)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create docs directory structure per implementation plan in docs/
- [x] T002 [P] Create docs/intro.md with course overview and learning outcomes
- [x] T003 [P] Create docs/modules/ directory with module subdirectories
- [x] T004 [P] Create docs/hardware/ directory with requirements, edge-kit, robot-lab, comparison files
- [x] T005 [P] Create docs/deployment/ directory with on-premise.md and cloud.md
- [x] T006 [P] Create docs/tutorials/ directory with ros2-basics.md, gazebo-simulation.md, isaac-perception.md
- [x] T007 Create static/img/ directory for images and diagrams

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T008 Update docusaurus.config.ts with site metadata (name, tagline, url)
- [x] T009 Configure themeConfig.navbar in docusaurus.config.ts with Course, Hardware, Deployment, Tutorials links
- [x] T010 Configure themeConfig.footer in docusaurus.config.ts
- [x] T011 Configure prism.additionalLanguages in docusaurus.config.ts for python, yaml, bash
- [x] T012 Create sidebars.ts with hierarchical navigation structure
- [x] T013 Create docs/assessments.md with course assessment overview
- [x] T014 Verify build succeeds with `yarn build` command - BUILD SUCCESSFUL
- [ ] T015 Verify site renders correctly in light and dark modes via `yarn start`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Browse Course Modules and Lessons (Priority: P1) 🎯 MVP

**Goal**: Enable students to browse the complete course structure with 4 modules and 13-week curriculum

**Independent Test**: Can a user land on homepage and understand course structure? Can they navigate to any module and see its lessons in 3 clicks or fewer?

- [x] T016 [US1] Create docs/modules/intro.md with overview of all 4 modules and their descriptions
- [x] T017 [US1] Create docs/modules/module1-ros2.md covering ROS 2 Robotic Nervous System (Weeks 3-5)
- [x] T018 [US1] Create docs/modules/module2-gazebo.md covering The Digital Twin (Weeks 6-7)
- [x] T019 [US1] Create docs/modules/module3-isaac.md covering The AI-Robot Brain (Weeks 8-10)
- [x] T020 [US1] Create docs/modules/module4-vla.md covering Vision-Language-Action (Weeks 11-13)
- [x] T021 [US1] Add sidebar items for each module in sidebars.ts
- [x] T022 [US1] Add next/previous lesson navigation links in all module pages (built-in Docusaurus feature)
- [ ] T023 [US1] Verify 3-click navigation from homepage to any lesson works correctly (run `yarn start`)
- [ ] T024 [US1] Verify build succeeds and all module pages render correctly (verified with build)

**Checkpoint**: User Story 1 complete - Users can browse course modules and lessons

---

## Phase 4: User Story 2 - Hardware Requirements and Setup Guides (Priority: P2)

**Goal**: Provide clear hardware specifications and setup guides for Sim Rigs, Edge Kits, and Robot Labs

**Independent Test**: Can a user find workstation requirements? Can they follow setup guide for Jetson Orin Nano?

- [x] T025 [US2] Create docs/hardware/requirements.md with Digital Twin Workstation specs (RTX 4070 Ti, i7, 64GB RAM, Ubuntu 22.04)
- [x] T026 [US2] Create docs/hardware/edge-kit.md with Jetson Orin Nano setup guide (RealSense D435i, ReSpeaker, SD card)
- [x] T027 [US2] Create docs/hardware/robot-lab.md documenting lab options (Proxy, Miniature Humanoid, Premium)
- [x] T028 [US2] Create docs/hardware/comparison.md with hardware comparison tables including pricing
- [x] T029 [US2] Add sidebar items for all hardware pages in sidebars.ts
- [x] T030 [US2] Include images/diagrams for hardware setup steps with alt text (placeholders for future images)
- [x] T031 [US2] Add troubleshooting section for common edge kit issues
- [ ] T032 [US2] Verify all hardware pages are accessible (WCAG 2.1 AA)
- [ ] T033 [US2] Verify build succeeds and all hardware pages render correctly (verified with build)

**Checkpoint**: User Story 2 complete - Users can access hardware requirements and setup guides

---

## Phase 5: User Story 3 - Hands-On Tutorials with Code Examples (Priority: P3)

**Goal**: Provide hands-on tutorials with working code examples for ROS 2, Gazebo, and NVIDIA Isaac

**Independent Test**: Can a user copy code from tutorial and run successfully? Are code blocks annotated with language?

- [x] T034 [US3] Create docs/tutorials/ros2-basics.md with ROS 2 nodes, topics, services examples (Python/rclpy)
- [x] T035 [US3] Create docs/tutorials/gazebo-simulation.md with Gazebo physics simulation examples (URDF/SDF)
- [x] T036 [US3] Create docs/tutorials/isaac-perception.md with NVIDIA Isaac perception pipeline examples
- [x] T037 [US3] Add code blocks with language annotations (```python, ```yaml, ```bash) to all tutorials
- [x] T038 [US3] Add line-by-line explanations for code examples in tutorials
- [x] T039 [US3] Add prerequisites section to each tutorial listing required knowledge
- [x] T040 [US3] Add troubleshooting tips for common errors in each tutorial
- [x] T041 [US3] Add before/after screenshots for visual tutorials (Gazebo simulation results) - placeholders added
- [x] T042 [US3] Verify all code examples have copy-paste functionality working (Docusaurus default)
- [ ] T043 [US3] Verify build succeeds and all tutorial pages render correctly (verified with build)

**Checkpoint**: User Story 3 complete - Users can follow hands-on tutorials with code examples

---

## Phase 6: User Story 4 - Deployment Options Comparison (Priority: P4)

**Goal**: Enable instructors/administrators to compare on-premise vs cloud deployment options

**Independent Test**: Can a user understand trade-offs? Can they estimate costs for their institution?

- [x] T044 [US4] Create docs/deployment/overview.md with system architecture diagram (Sim Rig, Edge Brain, Sensors, Actuator)
- [x] T045 [US4] Create docs/deployment/on-premise.md with physical lab setup guide and cost breakdown
- [x] T046 [US4] Create docs/deployment/cloud.md with cloud-native alternative (AWS/Azure, Isaac Sim on Omniverse Cloud)
- [x] T047 [US4] Add comparison table for On-Premise vs Cloud deployment options
- [x] T048 [US4] Include cost calculations for both approaches (OpEx vs CapEx)
- [x] T049 [US4] Document latency considerations for cloud-based robot control
- [x] T050 [US4] Add sidebar items for all deployment pages in sidebars.ts
- [ ] T051 [US4] Verify deployment comparison enables informed decision-making (manual review)
- [ ] T052 [US4] Verify build succeeds and all deployment pages render correctly (verified with build)

**Checkpoint**: User Story 4 complete - Users can compare deployment options

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and ensure constitution compliance

- [ ] T053 [P] Run lighthouse accessibility audit and fix any WCAG 2.1 AA violations
- [ ] T054 [P] Add search configuration to docusaurus.config.ts (local search plugin)
- [ ] T055 [P] Verify all images have descriptive alt text across all documentation
- [ ] T056 [P] Verify color contrast in both light and dark themes meets 4.5:1 ratio
- [ ] T057 [P] Add or verify keyboard navigation works for all interactive elements
- [ ] T058 Create docs/deployment/summary.md with quick reference for deployment decision-making
- [ ] T059 Update docusaurus.config.ts with i18n configuration (English default)
- [ ] T060 [P] Verify GitHub Pages deployment configuration in docusaurus.config.ts
- [ ] T061 Run full build verification with `yarn build`
- [ ] T062 Validate quickstart.md instructions work correctly

---

## Dependencies & Execution Order

### Phase Dependencies

| Phase | Depends On | Blocks |
|-------|------------|--------|
| Setup (1) | None | Foundational |
| Foundational (2) | Setup | All User Stories |
| User Story 1 (3) | Foundational | Polish |
| User Story 2 (4) | Foundational | Polish |
| User Story 3 (5) | Foundational | Polish |
| User Story 4 (6) | Foundational | Polish |
| Polish (7) | All User Stories | None |

### User Story Dependencies

| Story | Priority | Can Start After | Dependencies |
|-------|----------|-----------------|--------------|
| US1 | P1 | Foundational | None (first deliverable) |
| US2 | P2 | Foundational | Can integrate with US1 nav |
| US3 | P3 | Foundational | Independent |
| US4 | P4 | Foundational | Independent |

### Within Each User Story

- Content pages before navigation updates
- Core pages before troubleshooting sections
- Story complete before polish phase

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel
- Different user stories can be worked on in parallel by different team members
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1

```bash
# Create all module pages in parallel (different files, no dependencies):
Task: "Create docs/modules/intro.md"
Task: "Create docs/modules/module1-ros2.md"
Task: "Create docs/modules/module2-gazebo.md"
Task: "Create docs/modules/module3-isaac.md"
Task: "Create docs/modules/module4-vla.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T015)
3. Complete Phase 3: User Story 1 (T016-T024)
4. **STOP and VALIDATE**: Test navigation in 3 clicks, verify build
5. Deploy/demo MVP if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Demo (MVP!)
3. Add User Story 2 → Test independently → Demo
4. Add User Story 3 → Test independently → Demo
5. Add User Story 4 → Test independently → Demo
6. Polish phase → Final release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Course Modules)
   - Developer B: User Story 2 (Hardware)
   - Developer C: User Stories 3+4 (Tutorials + Deployment)
3. Stories complete and integrate independently

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 62 |
| Setup Tasks | 7 |
| Foundational Tasks | 8 |
| User Story 1 Tasks | 9 |
| User Story 2 Tasks | 9 |
| User Story 3 Tasks | 10 |
| User Story 4 Tasks | 9 |
| Polish Tasks | 10 |

**Recommended MVP Scope**: User Story 1 only (9 tasks after foundation)

**Next Step**: Begin with Phase 1 Setup tasks to initialize project structure.
