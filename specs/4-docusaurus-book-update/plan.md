# Implementation Plan: Docusaurus Book Update & Fix

**Branch**: `004-docusaurus-book-update` | **Date**: 2025-12-22 | **Spec**: specs/4-docusaurus-book-update/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Update and fix the Docusaurus-based Physical AI & Humanoid Robotics book to make it production-ready with correct structure. This involves updating the navbar to have only "Physical AI And Humanoid Robotics" and "Book" items, implementing the Book link to redirect to Module 1 → Chapter 1, restructuring content to have exactly 4 modules with 3 chapters each, and removing the Tutorial Intro page. The implementation will maintain the existing content while reorganizing the structure to match the specification.

## Technical Context

**Language/Version**: TypeScript 5.6.2, Node.js >=20.0
**Primary Dependencies**: Docusaurus 3.9.2, React 19.0.0, Node.js ecosystem
**Storage**: N/A (static site generator)
**Testing**: Docusaurus build process, manual browser testing
**Target Platform**: Web (GitHub Pages deployment)
**Project Type**: Web/static site
**Performance Goals**: Fast loading times, proper SEO, responsive design
**Constraints**: Static site limitations, GitHub Pages deployment constraints
**Scale/Scope**: Educational book with 4 modules, 3 chapters each, plus supporting content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [X] Spec-first development: Implementation follows spec.md requirements
- [X] Production-ready architecture: Docusaurus provides production-ready static site
- [X] Clear API contracts: Docusaurus configuration provides clear interface contracts
- [X] Technology Stack Requirements: Uses Docusaurus, Markdown, GitHub Pages as specified in constitution

## Project Structure

### Documentation (this feature)

```text
specs/4-docusaurus-book-update/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book_frontend/
├── docusaurus.config.ts    # Docusaurus configuration file - updates navbar items
├── sidebars.ts             # Navigation sidebar configuration - restructure to 4 modules
├── docs/                   # Book content files
│   ├── intro.md            # Tutorial Intro page (to be removed)
│   └── modules/            # Module directories
│       ├── ros2-nervous-system/
│       ├── digital-twin/
│       ├── ai-robot-brain/
│       └── vla/            # Will be added as Module 4
├── src/                    # Custom components and styling
├── static/                 # Static assets
├── package.json            # Project dependencies
└── tsconfig.json           # TypeScript configuration
```

**Structure Decision**: Web application structure selected for Docusaurus-based book frontend with static content organization.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |