# Implementation Plan: Docusaurus UI, Structure & Metadata Fix

**Branch**: `005-docusaurus-ui-fix` | **Date**: 2025-12-22 | **Spec**: specs/5-docusaurus-ui-fix/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Fix UI, navigation, structural inconsistencies, and metadata issues in the existing Docusaurus-based Physical AI & Humanoid Robotics book to make it professional, visually appealing, structurally correct, and production-ready for GitHub Pages deployment. This involves cleaning the navbar to have only the logo and "Book" link, updating the hero button text to "Start learning From Today", ensuring Module 2 and Module 3 chapters are properly numbered (Chapter 1, Chapter 2, Chapter 3), configuring logo.png as the site logo, and updating the global metadata title to "Physical AI & Humanoid Robotics Book".

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
specs/5-docusaurus-ui-fix/
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
├── docusaurus.config.ts    # Docusaurus configuration file - updates navbar, logo, title
├── sidebars.ts             # Navigation sidebar configuration - verify module structure
├── src/
│   └── pages/
│       └── index.tsx       # Homepage with hero section - updates button text
├── static/
│   └── img/
│       └── logo.png        # Site logo image file
├── docs/                   # Book content files - verify chapter numbering in modules 2 & 3
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