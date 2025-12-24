---
description: "Task list for Dark Mode, Structure & UI Polish feature implementation"
---

# Tasks: Dark Mode, Structure & UI Polish

**Input**: Design documents from `/specs/006-dark-mode-ui-polish/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No explicit tests requested in feature specification - tests are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `book_frontend/src/`, `book_frontend/docs/`, `book_frontend/docusaurus.config.ts`

<!--
  ============================================================================
  IMPORTANT: These tasks are generated based on:
  - User stories from spec.md (with their priorities P1, P1, P2, P2)
  - Feature requirements from plan.md
  - Tasks are organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan
- [X] T002 Verify Docusaurus project is properly configured in book_frontend/
- [X] T003 [P] Set up development environment for Docusaurus

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Verify current Docusaurus build process works without errors
- [X] T005 [P] Backup current custom.css file before modifications
- [X] T006 [P] Identify all chapter files that need title updates in Modules 2 and 3

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Dark Mode UI Enhancement (Priority: P1) 🎯 MVP

**Goal**: Implement enhanced dark mode with proper contrast ratios and readability improvements

**Independent Test**: Can be fully tested by enabling dark mode and verifying that all text has proper contrast ratios, colors are consistent across components, and the UI looks professional in both light and dark modes.

### Implementation for User Story 1

- [X] T007 [P] [US1] Update dark mode color variables in book_frontend/src/css/custom.css for better contrast
- [X] T008 [P] [US1] Enhance --ifm-color-content and --ifm-color-content-secondary for dark mode
- [X] T009 [P] [US1] Improve --ifm-background-color and --ifm-background-surface-color in dark mode
- [X] T010 [US1] Update code highlighting background in dark mode for better readability
- [X] T011 [US1] Test contrast ratios using accessibility tools to ensure WCAG 2.1 AA compliance
- [X] T012 [US1] Verify dark mode styling remains consistent across all components

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Module Chapter Numbering Consistency (Priority: P1)

**Goal**: Update Modules 2 and 3 to include explicit "Chapter X:" prefix in titles for consistency with Modules 1 and 4

**Independent Test**: Can be fully tested by verifying that Modules 2 and 3 explicitly show chapter numbers in the sidebar, page titles, and navigation structure, matching the expected format.

### Implementation for User Story 2

- [X] T013 [P] [US2] Update title in book_frontend/docs/modules/digital-twin/chapter-1-physics-simulation.md to include "Chapter 1:"
- [X] T014 [P] [US2] Update title in book_frontend/docs/modules/digital-twin/chapter-2-sensor-simulation.md to include "Chapter 2:"
- [X] T015 [P] [US2] Update title in book_frontend/docs/modules/digital-twin/chapter-3-unity-interaction.md to include "Chapter 3:"
- [X] T016 [P] [US2] Update title in book_frontend/docs/modules/ai-robot-brain/chapter-1-isaac-sim.md to include "Chapter 1:"
- [X] T017 [P] [US2] Update title in book_frontend/docs/modules/ai-robot-brain/chapter-2-isaac-ros.md to include "Chapter 2:"
- [X] T018 [P] [US2] Update title in book_frontend/docs/modules/ai-robot-brain/chapter-3-nav2-navigation.md to include "Chapter 3:"
- [X] T019 [US2] Update main H1 headings in Module 2 files to match the new title format
- [X] T020 [US2] Update main H1 headings in Module 3 files to match the new title format
- [X] T021 [US2] Verify sidebar navigation remains intact and properly displays new chapter numbering
- [X] T022 [US2] Test that navigation order remains unchanged and links work properly

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Improved Navbar Interaction (Priority: P2)

**Goal**: Remove black focus/outline borders from navbar links and implement smooth, accessible hover effects

**Independent Test**: Can be fully tested by clicking on navbar links and verifying that no black focus/outline borders appear, and hover effects are smooth and consistent.

### Implementation for User Story 3

- [X] T023 [US3] Modify focus outline behavior in book_frontend/src/css/custom.css for navbar links
- [X] T024 [US3] Update a:focus, button:focus, input:focus, select:focus, textarea:focus selector to use more appropriate colors
- [X] T025 [US3] Enhance hover effects for .navbar__item:hover with better transitions
- [X] T026 [US3] Implement smooth hover effects with transition under 300ms
- [X] T027 [US3] Ensure hover effects maintain accessible contrast ratios in both light and dark modes
- [X] T028 [US3] Test accessibility compliance for keyboard navigation

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Professional Footer UI (Priority: P2)

**Goal**: Create a professional-looking footer that matches the overall site theme with improved spacing, typography, and visual consistency

**Independent Test**: Can be fully tested by viewing the footer in both light and dark modes and verifying that it looks clean, professional, and visually aligned with the site theme.

### Implementation for User Story 4

- [X] T029 [US4] Update footer styling in book_frontend/src/css/custom.css for improved spacing and typography
- [X] T030 [US4] Enhance footer configuration in book_frontend/docusaurus.config.ts if needed for better structure
- [X] T031 [US4] Implement professional footer styling with proper spacing
- [X] T032 [US4] Ensure footer looks professional in both light and dark modes
- [X] T033 [US4] Verify responsive behavior and proper typography hierarchy
- [X] T034 [US4] Test footer readability and visual consistency in both modes

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T035 [P] Test Docusaurus build to ensure no errors or warnings related to UI improvements
- [X] T036 Run accessibility testing with tools like axe-core
- [X] T037 [P] Validate contrast ratios using WebAIM contrast checker across all components
- [X] T038 Manual testing across different browsers (Chrome, Firefox, Safari)
- [X] T039 Responsive design testing across different screen sizes
- [X] T040 Dark/light mode switching validation
- [X] T041 Final verification that all requirements from spec.md are met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P1 → P2 → P2)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members
- Tasks within User Stories 1 and 2 marked [P] can run in parallel

---

## Parallel Example: User Story 1 and 2

```bash
# Launch all dark mode tasks together:
Task: "Update dark mode color variables in book_frontend/src/css/custom.css for better contrast"
Task: "Enhance --ifm-color-content and --ifm-color-content-secondary for dark mode"
Task: "Improve --ifm-background-color and --ifm-background-surface-color in dark mode"

# Launch all chapter numbering tasks together:
Task: "Update title in book_frontend/docs/modules/digital-twin/chapter-1-physics-simulation.md to include 'Chapter 1:'"
Task: "Update title in book_frontend/docs/modules/digital-twin/chapter-2-sensor-simulation.md to include 'Chapter 2:'"
Task: "Update title in book_frontend/docs/modules/digital-twin/chapter-3-unity-interaction.md to include 'Chapter 3:'"
```

---

## Implementation Strategy

### MVP First (User Stories 1 and 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Dark Mode Enhancement)
4. Complete Phase 4: User Story 2 (Chapter Numbering)
5. **STOP and VALIDATE**: Test User Stories 1 and 2 independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Dark Mode)
   - Developer B: User Story 2 (Chapter Numbering)
   - Developer C: User Story 3 (Navbar)
   - Developer D: User Story 4 (Footer)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Each user story delivers independent value to users