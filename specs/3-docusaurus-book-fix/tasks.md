---
description: "Task list for Docusaurus Book Update & Fix implementation"
---

# Tasks: Docusaurus Book Update & Fix

**Input**: Design documents from `/specs/3-docusaurus-book-fix/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit tests requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `book_frontend/` at repository root
- Paths shown below follow the Docusaurus project structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Navigate to book_frontend directory and verify project structure
- [X] T002 Install dependencies with `npm install` in book_frontend/
- [X] T003 [P] Verify current Docusaurus site runs with `npm run start`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 [P] Update site title to "Physical AI & Humanoid Robotics" in book_frontend/docusaurus.config.ts
- [X] T005 [P] Update navbar title to "Physical AI & Humanoid Robotics" in book_frontend/docusaurus.config.ts
- [X] T006 [P] Update favicon and site metadata in book_frontend/docusaurus.config.ts
- [X] T007 [P] Remove Blog-related items from navbar configuration in book_frontend/docusaurus.config.ts
- [X] T008 [P] Configure root URL (/) to serve as book homepage in book_frontend/docusaurus.config.ts

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Homepage Access (Priority: P1) 🎯 MVP

**Goal**: Update the homepage to display "Physical AI & Humanoid Robotics" as both the page title and browser tab title

**Independent Test**: Can be fully tested by visiting the root URL (/) and verifying that the page displays "Physical AI & Humanoid Robotics" as the title and provides clear navigation to the book content.

### Implementation for User Story 1

- [X] T009 [US1] Update homepage content to properly display book title in book_frontend/src/pages/index.tsx or equivalent
- [X] T010 [US1] Verify site title appears in browser tab as "Physical AI & Humanoid Robotics"
- [X] T011 [US1] Test that root URL (/) correctly routes to the book homepage
- [X] T012 [US1] Validate that homepage provides clear navigation to book content

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Clean Navigation Experience (Priority: P1)

**Goal**: Clean up the navigation to include only essential items (Home and Book Content) to provide a focused learning experience

**Independent Test**: Can be fully tested by examining the navbar and verifying only the required items (Home and Book Content) are present.

### Implementation for User Story 2

- [X] T013 [US2] Add "Home" navigation item pointing to root URL in book_frontend/docusaurus.config.ts
- [X] T014 [US2] Add "Book Content" navigation item pointing to docs root in book_frontend/docusaurus.config.ts
- [X] T015 [US2] Remove all Blog and Tutorial links from navbar in book_frontend/docusaurus.config.ts
- [X] T016 [US2] Verify navbar contains exactly 2 items: "Home" and "Book Content"
- [X] T017 [US2] Test that no Blog or Tutorials links appear in the navbar

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Structured Book Content Hierarchy (Priority: P1)

**Goal**: Enforce the exact Module → Chapter hierarchy with 3 modules containing 3 chapters each, all properly numbered

**Independent Test**: Can be fully tested by verifying the sidebar navigation displays the exact Module/Chapter structure as specified.

### Implementation for User Story 3

- [X] T018 [US3] Restructure VLA module to have exactly 3 chapters by grouping content in book_frontend/docs/modules/vla/
- [X] T019 [US3] Update sidebar configuration to reflect exact Module → Chapter hierarchy in book_frontend/sidebars.ts
- [X] T020 [US3] Ensure all modules and chapters are explicitly numbered (Module 1, Chapter 1, etc.) in book_frontend/sidebars.ts
- [X] T021 [US3] Verify sidebar shows exactly 3 modules with 3 chapters each
- [X] T022 [US3] Test navigation through modules and chapters matches specified structure

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Proper Content Rendering (Priority: P2)

**Goal**: Fix all Markdown content to render correctly without formatting issues, console errors, or build warnings

**Independent Test**: Can be fully tested by viewing multiple pages and verifying that headings, code blocks, lists, and other Markdown elements render properly.

### Implementation for User Story 4

- [X] T023 [US4] Audit all Markdown files for proper heading levels in book_frontend/docs/
- [X] T024 [US4] Verify correct code block fencing with proper language tags in book_frontend/docs/
- [X] T025 [US4] Check for valid frontmatter in all content files in book_frontend/docs/
- [X] T026 [US4] Ensure lists and other Markdown elements render correctly in book_frontend/docs/
- [X] T027 [US4] Run Docusaurus build process and fix any warnings in book_frontend/
- [X] T028 [US4] Test that no rendering errors or console warnings occur in browser

**Checkpoint**: All user stories should now be fully functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T029 [P] Update GitHub Pages deployment configuration in book_frontend/docusaurus.config.ts
- [X] T030 [P] Run full build test with `npm run build` to ensure no errors
- [X] T031 [P] Test production build locally with `npm run serve`
- [X] T032 [P] Verify all links and navigation work properly in built site
- [X] T033 [P] Run quickstart validation as per quickstart.md
- [X] T034 Update README.md with new project information

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
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 2

```bash
# Launch all navigation updates for User Story 2 together:
Task: "Add Home navigation item pointing to root URL in book_frontend/docusaurus.config.ts"
Task: "Add Book Content navigation item pointing to docs root in book_frontend/docusaurus.config.ts"
Task: "Remove all Blog and Tutorial links from navbar in book_frontend/docusaurus.config.ts"
```

---

## Implementation Strategy

### MVP First (User Stories 1-3 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. Complete Phase 4: User Story 2
5. Complete Phase 5: User Story 3
6. **STOP and VALIDATE**: Test User Stories 1-3 independently
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Stories 1-3 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 4 → Test independently → Deploy/Demo
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence