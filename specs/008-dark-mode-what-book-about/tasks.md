---
description: "Task list for Dark Mode UI Upgrade for 'What This Book Is About' Section feature implementation"
---

# Tasks: Dark Mode UI Upgrade for "What This Book Is About" Section

**Input**: Design documents from `/specs/008-dark-mode-what-book-about/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Contrast ratio validation, readability assessment, cross-browser compatibility, responsive design verification

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `book_frontend/src/`, `book_frontend/src/css/`, `book_frontend/src/pages/`

<!-- ============================================================================
  IMPORTANT: These tasks are generated based on:
  - User stories from spec.md (with their priorities P1, P1, P2)
  - Feature requirements from plan.md
  - Tasks are organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment
  ============================================================================ -->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan
- [X] T002 Verify Docusaurus project is properly configured in book_frontend/
- [X] T003 Set up development environment for Docusaurus

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Verify current Docusaurus build process works without errors
- [X] T005 Backup current custom.css file before modifications
- [X] T006 Identify all section components that need dark mode improvements

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Professional Section Layout in Dark Mode (Priority: P1) 🎯 MVP

**Goal**: Implement professional and well-structured "What This Book Is About" section in dark mode with proper spacing, alignment, visual hierarchy, and readable content

**Independent Test**: Can be fully tested by viewing the section in dark mode and verifying that the layout has proper spacing, alignment, visual hierarchy, and the content is clearly readable with appropriate contrast.

### Implementation for User Story 1

- [X] [US1] T007 Update "What This Book Is About" section background color in dark mode in book_frontend/src/css/custom.css
- [X] [US1] T008 Enhance section text contrast with --ifm-color-content and --ifm-color-content-secondary in dark mode
- [X] [US1] T009 Improve section heading typography and spacing in book_frontend/src/css/custom.css
- [X] [US1] T010 Update section content element styling with proper contrast ratios in dark mode
- [X] [US1] T011 Test section contrast ratios using accessibility tools to ensure WCAG 2.1 AA compliance
- [X] [US1] T012 Verify section styling remains consistent with overall site theme in dark mode

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

### Validation for User Story 1

- [X] [US1] T013 Validate section readability in dark mode with accessibility tools
- [X] [US1] T014 Verify content element visibility and contrast ratios in the section
- [X] [US1] T015 Test section responsive behavior across different screen sizes
- [X] [US1] T016 Confirm section styling doesn't affect light mode

---

## Phase 4: User Story 2 - Readable Content Hierarchy (Priority: P1)

**Goal**: Create clear visual separation between section title, description, learning outcomes, and concept bridge in dark mode with appropriate spacing and contrast

**Independent Test**: Can be fully tested by viewing the section in dark mode and verifying that the title, description, lists, and concept bridge are clearly separated with appropriate spacing and contrast.

### Implementation for User Story 2

- [X] [US2] T017 Update section title styling for better hierarchy in dark mode
- [X] [US2] T018 Enhance description paragraph contrast and spacing in the section
- [X] [US2] T019 Improve learning outcomes list styling with proper spacing
- [X] [US2] T020 Ensure concept bridge elements remain readable in dark mode
- [X] [US2] T021 Test contrast ratios in the section using accessibility tools
- [X] [US2] T022 Verify section styling consistency with overall site theme in dark mode

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

### Validation for User Story 2

- [X] [US2] T023 Validate section content hierarchy clarity in dark mode
- [X] [US2] T024 Verify visual separation between content elements in the section
- [X] [US2] T025 Test section responsive behavior across different screen sizes
- [X] [US2] T026 Confirm section styling doesn't affect light mode

---

## Phase 5: User Story 3 - Optimized Typography & Readability (Priority: P2)

**Goal**: Implement optimized font sizes, line height, and contrast that make the section content comfortable to read in dark mode without eye fatigue

**Independent Test**: Can be fully tested by reading through the section content in dark mode and verifying that font sizes, line height, and contrast ratios meet accessibility standards and provide comfortable reading experience.

### Implementation for User Story 3

- [X] [US3] T027 Update font sizes for better readability in the section
- [X] [US3] T028 Improve line height for comfortable reading experience
- [X] [US3] T029 Enhance contrast ratios for typography elements
- [X] [US3] T030 Ensure emojis/icons remain readable with proper contrast
- [X] [US3] T031 Test typography contrast ratios using accessibility tools
- [X] [US3] T032 Verify typography consistency with overall site theme in dark mode

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

### Validation for User Story 3

- [X] [US3] T033 Validate typography comfort in dark mode for extended reading
- [X] [US3] T034 Verify font sizing and line height consistency
- [X] [US3] T035 Test section responsive behavior across different screen sizes
- [X] [US3] T036 Confirm typography styling doesn't affect light mode

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall consistency

- [X] [P] T037 Test Docusaurus build to ensure no errors or warnings related to UI improvements
- [X] T038 Run accessibility testing with tools like axe-core across all sections
- [X] [P] T039 Validate contrast ratios using WebAIM contrast checker across all components
- [X] T040 Manual testing across different browsers (Chrome, Firefox, Safari)
- [X] [P] T041 Responsive design testing across different screen sizes
- [X] T042 Dark/light mode switching validation
- [X] T043 Final verification that all requirements from spec.md are met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P1 → P2)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members
- Tasks within User Stories marked [P] can run in parallel

---

## Parallel Example: User Story 1 and 2

```bash
# Launch all section layout tasks together:
Task: "Update section background color in dark mode in book_frontend/src/css/custom.css"
Task: "Enhance section text contrast with --ifm-color-content and --ifm-color-content-secondary in dark mode"
Task: "Improve section heading typography and spacing in book_frontend/src/css/custom.css"

# Launch all content hierarchy tasks together:
Task: "Update section title styling for better hierarchy in dark mode"
Task: "Enhance description paragraph contrast and spacing in the section"
Task: "Improve learning outcomes list styling with proper spacing"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Professional Section Layout)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Section Layout)
   - Developer B: User Story 2 (Content Hierarchy)
   - Developer C: User Story 3 (Typography)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Each user story delivers independent value to users