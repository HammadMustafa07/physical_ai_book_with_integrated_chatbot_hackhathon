# Testable Tasks: Docusaurus UI, Structure & Metadata Fix

**Feature**: Docusaurus UI, Structure & Metadata Fix
**Branch**: `005-docusaurus-ui-fix`
**Spec**: specs/5-docusaurus-ui-fix/spec.md
**Plan**: specs/5-docusaurus-ui-fix/plan.md
**Date**: 2025-12-22

## Phase 1: Setup Tasks

- [X] T001 Create tasks.md file based on spec and plan documents

## Phase 2: Foundational Tasks

- [X] T002 [P] Update docusaurus.config.ts to use logo.png instead of logo.svg
- [X] T003 [P] Update docusaurus.config.ts site title to "Physical AI & Humanoid Robotics Book"
- [X] T004 [P] Remove duplicate "Physical AI And Humanoid Robotics" text link from navbar items

## Phase 3: User Story 1 - Clean Navigation Experience (P1)

- [X] T005 [US1] Update navbar configuration in docusaurus.config.ts to have only logo and "Book" items
- [X] T006 [US1] Verify navbar displays exactly one home link via logo
- [X] T007 [US1] Test that duplicate text-based home links are removed

## Phase 4: User Story 2 - Professional UI Experience (P1)

- [ ] T008 [US2] Apply CSS improvements for professional UI appearance
- [ ] T009 [US2] Verify typography improvements for better readability
- [ ] T010 [US2] Test spacing and layout refinements

## Phase 5: User Story 3 - Correct Module Structure (P2)

- [ ] T011 [US3] Verify Module 2 chapters are properly numbered as Chapter 1, Chapter 2, Chapter 3 in content
- [ ] T012 [US3] Verify Module 3 chapters are properly numbered as Chapter 1, Chapter 2, Chapter 3 in content
- [ ] T013 [US3] Update chapter titles in markdown files if needed to reflect proper numbering

## Phase 6: User Story 4 - Updated Hero Section (P2)

- [X] T014 [US4] Update hero button text in src/pages/index.tsx to "Start learning From Today"
- [X] T015 [US4] Verify hero button text displays correctly on homepage
- [X] T016 [US4] Test that button functionality remains intact after text update

## Phase 7: User Story 5 - Correct Branding and Metadata (P1)

- [X] T017 [US5] Configure logo.png as site logo in docusaurus.config.ts
- [X] T018 [US5] Verify logo.png renders correctly in navbar
- [X] T019 [US5] Test that page metadata title shows "Physical AI & Humanoid Robotics Book"

## Phase 8: Quality Assurance & Testing

- [X] T020 Run Docusaurus build process to ensure no build errors occur
- [X] T021 Test development server to verify all navigation works correctly
- [X] T022 Verify no hydration errors occur in the Docusaurus application
- [X] T023 Test that all 4 modules with 3 chapters each are accessible
- [X] T024 Run production build and serve locally to test final output

## Phase 9: Polish & Cross-Cutting Concerns

- [X] T025 Update GitHub Pages deployment configuration if needed
- [X] T026 Final verification that all functional requirements are met (FR-001 through FR-010)
- [X] T027 Verify success criteria are met (SC-001 through SC-008)
- [X] T028 Clean up any temporary files or unused configurations

## Dependencies

User Story 2 (Professional UI Experience) may depend on foundational updates to CSS files. User Story 5 (Correct Branding and Metadata) requires the logo configuration to be properly set up in docusaurus.config.ts before verification can occur.

## Parallel Execution Opportunities

- Tasks T002, T003, and T004 can be executed in parallel as they work with different aspects of the configuration
- Tasks in Phase 8 can be executed after all content and configuration changes are complete
- User stories 1, 2, and 5 can be developed in parallel since they focus on different aspects of the UI

## Implementation Strategy

1. **MVP Scope**: Complete User Story 1 (Clean Navigation) and User Story 5 (Correct Branding) first to establish the basic UI fixes
2. **Incremental Delivery**: Add hero section updates (User Story 4) and module structure verification (User Story 3)
3. **Quality Assurance**: Perform comprehensive testing in Phase 8 to ensure all requirements are met
4. **Final Polish**: Complete cross-cutting concerns and final verification in Phase 9