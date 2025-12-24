---
description: "Task list for dark mode UI fix implementation"
---

# Tasks: Dark Mode UI Fix (Homepage Sections)

**Input**: Design documents from `/specs/[009-dark-mode-homepage-fix]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Research current Docusaurus homepage structure and dark mode implementation
- [x] T002 Identify all files related to homepage sections (Hero, What This Book Is About, Course Structure)
- [x] T003 [P] Document current color variables and contrast ratios in existing CSS

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [x] T004 Define dark mode color palette with WCAG AA compliant contrast ratios
- [x] T005 [P] Create CSS variables for dark mode colors in theme
- [x] T006 [P] Set up Docusaurus theme customization files for dark mode
- [x] T007 Research Docusaurus dark mode best practices and implementation patterns
- [x] T008 Configure development environment for testing dark mode changes
- [x] T009 [P] Create backup of current homepage files before modifications

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Dark Mode Homepage Sections (Priority: P1) 🎯 MVP

**Goal**: Enable users to view the homepage sections (Hero, What This Book Is About, Course Structure) in dark mode with proper styling and readability

**Independent Test**: Enable dark mode and verify that all three specified sections are properly styled with good contrast and readability.

### Implementation for User Story 1

- [x] T010 [P] [US1] Update Hero section with dark mode styling in src/pages/index.js
- [x] T011 [P] [US1] Update "What This Book Is About" section with dark mode styling in src/pages/index.js
- [ ] T012 [P] [US1] Update "Course Structure" section with dark mode styling in src/pages/index.js
- [ ] T013 [US1] Implement proper text contrast ratios (minimum 4.5:1) for all sections in dark mode
- [ ] T014 [US1] Test dark mode functionality across all three sections
- [ ] T015 [US1] Validate that text remains readable in dark mode without eye strain

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Professional Dark Mode Appearance (Priority: P1)

**Goal**: Ensure the dark mode looks professional and visually balanced to build trust and credibility with users

**Independent Test**: Visually inspect the dark mode sections to ensure they meet professional design standards.

### Implementation for User Story 2

- [ ] T016 [P] [US2] Improve visual hierarchy in Hero section for professional appearance in dark mode
- [ ] T017 [P] [US2] Enhance typography and spacing in "What This Book Is About" section for readability
- [ ] T018 [P] [US2] Improve visual grouping and organization in "Course Structure" section
- [ ] T019 [US2] Ensure consistent spacing and alignment across all sections in dark mode
- [ ] T020 [US2] Optimize CTA button styling in Hero section for dark mode visibility
- [ ] T021 [US2] Ensure icons/emojis in "What This Book Is About" section remain readable in dark mode
- [ ] T022 [US2] Verify headings and lists in "Course Structure" section remain clear in dark mode

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Consistent Dark Mode Experience (Priority: P2)

**Goal**: Maintain consistent dark mode styling across all homepage sections for a cohesive user experience

**Independent Test**: Navigate between sections and verify visual consistency.

### Implementation for User Story 3

- [ ] T023 [P] [US3] Ensure consistent color scheme across Hero, "What This Book Is About", and "Course Structure" sections
- [ ] T024 [P] [US3] Apply consistent typography styles across all three sections in dark mode
- [ ] T025 [P] [US3] Standardize spacing and padding across all sections in dark mode
- [ ] T026 [US3] Test consistent styling across different screen sizes and responsive layouts
- [ ] T027 [US3] Validate that theme switching works consistently across all sections
- [ ] T028 [US3] Ensure no visual regressions occur in light mode after dark mode implementation

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T029 [P] Documentation updates for dark mode implementation approach
- [ ] T030 Code cleanup and refactoring of CSS variables and theme files
- [ ] T031 Performance optimization for theme switching
- [ ] T032 [P] Additional accessibility improvements for dark mode
- [ ] T033 Security hardening (if applicable to styling)
- [ ] T034 Run validation to ensure clean Docusaurus build with zero warnings

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all styling tasks for User Story 1 together:
Task: "Update Hero section with dark mode styling in src/pages/index.js"
Task: "Update 'What This Book Is About' section with dark mode styling in src/pages/index.js"
Task: "Update 'Course Structure' section with dark mode styling in src/pages/index.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence