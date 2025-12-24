---
description: "Task list for Homepage Theme & Section Redesign feature implementation"
---

# Tasks: Homepage Theme & Section Redesign

**Input**: Design documents from `/specs/001-homepage-theme-redesign/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: No explicit test requirements in the specification, so tests are omitted.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `book_frontend/src/` for source files
- **CSS**: `book_frontend/src/css/` for styling
- **Pages**: `book_frontend/src/pages/` for homepage
- **Components**: `book_frontend/src/components/` for reusable components

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Navigate to book_frontend directory and verify project structure
- [x] T002 [P] Verify Node.js and npm are available with required versions
- [x] T003 [P] Install project dependencies if needed using npm install

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Verify current Docusaurus theme variables in book_frontend/src/css/custom.css
- [x] T005 [P] Verify homepage structure in book_frontend/src/pages/index.tsx
- [x] T006 [P] Verify HomepageFeatures component in book_frontend/src/components/HomepageFeatures/index.tsx
- [x] T007 Test current build process with npm run build
- [x] T008 Verify development server starts with npm run start

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Professional Homepage with Navy Blue Theme (Priority: P1) 🎯 MVP

**Goal**: Apply navy blue theme across the entire site with proper contrast and readability

**Independent Test**: The website displays with navy blue as the primary color across the navigation bar, buttons, and key sections, with proper contrast and readability maintained throughout.

### Implementation for User Story 1

- [x] T009 [P] [US1] Update primary color variables in book_frontend/src/css/custom.css to navy blue theme
- [x] T010 [P] [US1] Add complementary secondary color in book_frontend/src/css/custom.css
- [x] T011 [P] [US1] Update dark mode theme variables in book_frontend/src/css/custom.css
- [x] T012 [US1] Verify WCAG AA compliance for contrast ratios in custom.css
- [x] T013 [US1] Test theme application on development server (npm run start)
- [x] T014 [US1] Test responsive behavior across different screen sizes with new theme
- [x] T015 [US1] Verify build succeeds with new theme (npm run build)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Custom Content Sections (Priority: P1)

**Goal**: Replace default Docusaurus section with two custom content sections explaining the book and its structure

**Independent Test**: The homepage displays two custom content sections with the specified content: one explaining what the book is about and another outlining the course structure.

### Implementation for User Story 2

- [x] T016 [US2] Remove HomepageFeatures component from book_frontend/src/pages/index.tsx
- [x] T017 [US2] Create "What This Book Is About" section in book_frontend/src/pages/index.tsx
- [x] T018 [US2] Create "Course Structure" section in book_frontend/src/pages/index.tsx
- [x] T019 [US2] Add proper styling classes to new sections in book_frontend/src/pages/index.tsx
- [x] T020 [US2] Test content sections display correctly on development server
- [x] T021 [US2] Verify responsive behavior of new content sections
- [x] T022 [US2] Verify build succeeds with new content sections

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Clean Homepage Layout (Priority: P2)

**Goal**: Ensure clean, uncluttered homepage without default Docusaurus promotional content

**Independent Test**: The default Docusaurus promotional section is removed from the homepage, leaving only book-specific content.

### Implementation for User Story 3

- [x] T023 [US3] Verify default Docusaurus promotional section is completely removed
- [x] T024 [US3] Confirm no remnants of HomepageFeatures remain in book_frontend/src/pages/index.tsx
- [x] T025 [US3] Verify clean layout with proper spacing between sections
- [x] T026 [US3] Test that no generic Docusaurus promotional elements appear
- [x] T027 [US3] Verify all content is book-specific with no default placeholders
- [x] T028 [US3] Test final layout on development server
- [x] T029 [US3] Run final build to ensure no errors (npm run build)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T030 [P] Update documentation in docs/ if needed
- [x] T031 Code cleanup and formatting in modified files
- [x] T032 Performance verification across different devices and browsers
- [x] T033 [P] Run accessibility audit with browser dev tools
- [x] T034 Final build verification with npm run build
- [x] T035 Run quickstart.md validation steps

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
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all theme-related tasks for User Story 1 together:
Task: "Update primary color variables in book_frontend/src/css/custom.css to navy blue theme"
Task: "Add complementary secondary color in book_frontend/src/css/custom.css"
Task: "Update dark mode theme variables in book_frontend/src/css/custom.css"
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
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence