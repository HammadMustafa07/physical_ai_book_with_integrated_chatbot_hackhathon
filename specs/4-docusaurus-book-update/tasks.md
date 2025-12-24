# Testable Tasks: Docusaurus Book Update & Fix

**Feature**: Docusaurus Book Update & Fix
**Branch**: `004-docusaurus-book-update`
**Spec**: specs/4-docusaurus-book-update/spec.md
**Plan**: specs/4-docusaurus-book-update/plan.md
**Date**: 2025-12-22

## Phase 1: Setup Tasks

- [ ] T001 Create tasks.md file based on spec and plan documents

## Phase 2: Foundational Tasks

- [X] T002 [P] Update docusaurus.config.ts to set correct site title and metadata
- [X] T003 [P] Verify all existing content files render without console errors
- [X] T004 [P] Create Module 4 content directory for VLA content

## Phase 3: User Story 1 - Book Homepage Access (P1)

- [X] T005 [US1] Update docusaurus.config.ts to ensure site title is "Physical AI & Humanoid Robotics"
- [X] T006 [US1] Configure root URL (/) to route to book homepage with correct title
- [X] T007 [US1] Verify browser tab shows "Physical AI & Humanoid Robotics" as title

## Phase 4: User Story 2 - Clean Navigation Experience (P1)

- [X] T008 [US2] Update navbar configuration in docusaurus.config.ts to have only "Physical AI And Humanoid Robotics" and "Book" items
- [X] T009 [US2] Remove "Home" navbar item from docusaurus.config.ts
- [X] T010 [US2] Replace "Book Content" with "Book" navbar item in docusaurus.config.ts
- [X] T011 [US2] Remove any other navigation items from navbar configuration

## Phase 5: User Story 3 - Structured Book Content Hierarchy (P1)

- [X] T012 [US3] Restructure VLA content into 3 chapters for Module 4 in docs/modules/vla/
- [X] T013 [US3] Create Module 4 directory structure with 3 chapters
- [X] T014 [US3] Update sidebars.ts to include Module 4 with 3 chapters
- [X] T015 [US3] Verify sidebar displays exact Module → Chapter hierarchy (4 modules with 3 chapters each)
- [X] T016 [US3] Ensure all modules and chapters are properly numbered in sidebar

## Phase 6: User Story 4 - Direct Book Access (P2)

- [X] T017 [US4] Configure "Book" navbar item to redirect directly to Module 1 → Chapter 1
- [X] T018 [US4] Update navbar item type to point to first chapter instead of sidebar
- [X] T019 [US4] Test that clicking "Book" redirects to Module 1 → Chapter 1

## Phase 7: Content Cleanup Tasks

- [X] T020 Remove Tutorial Intro page (docs/intro.md) as specified in requirements
- [X] T021 Update sidebar configuration to no longer reference intro page
- [X] T022 [P] Verify all Markdown files have proper heading levels (h1, h2, h3, etc.)
- [X] T023 [P] Check all Markdown files for valid frontmatter
- [X] T024 [P] Ensure code blocks have proper fencing with syntax highlighting
- [X] T025 [P] Verify lists and other Markdown elements render correctly

## Phase 8: Quality Assurance & Testing

- [X] T026 Run Docusaurus build process to ensure no build errors occur
- [X] T027 Test development server to verify all navigation works correctly
- [X] T028 Verify no hydration errors occur in the Docusaurus application
- [X] T029 Test that all 4 modules with 3 chapters each are accessible
- [X] T030 Run production build and serve locally to test final output

## Phase 9: Polish & Cross-Cutting Concerns

- [X] T031 Update GitHub Pages deployment configuration if needed
- [X] T032 Final verification that all functional requirements are met (FR-001 through FR-012)
- [X] T033 Verify success criteria are met (SC-001 through SC-008)
- [X] T034 Clean up any temporary files or unused configurations

## Dependencies

User Story 3 (Structured Book Content Hierarchy) must be completed before User Story 4 (Direct Book Access) can be fully tested, as the direct access needs to point to an existing Module 1 → Chapter 1.

## Parallel Execution Opportunities

- Tasks T002, T003, and T004 can be executed in parallel as they work with different aspects of the codebase
- Tasks T022, T023, T024, and T025 can be executed in parallel as they involve checking different aspects of Markdown files
- Tasks in Phase 8 can be executed after all content and configuration changes are complete

## Implementation Strategy

1. **MVP Scope**: Complete User Story 1 (Book Homepage Access) and User Story 2 (Clean Navigation) first to establish the basic structure
2. **Incremental Delivery**: Add Module 4 content and update sidebar in Phase 3, then implement direct book access in Phase 4
3. **Quality Assurance**: Perform comprehensive testing in Phase 8 to ensure all requirements are met
4. **Final Polish**: Complete cross-cutting concerns and final verification in Phase 9