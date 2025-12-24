# Implementation Plan: Docusaurus Book Update & Fix

**Branch**: `003-docusaurus-book-fix` | **Date**: 2025-12-21 | **Spec**: specs/3-docusaurus-book-fix/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Update and fix the Docusaurus-based Physical AI & Humanoid Robotics book to make it production-ready with correct structure. This involves changing the site title to "Physical AI & Humanoid Robotics", cleaning up the navigation to include only essential items (Home and Book Content), enforcing the Module → Chapter hierarchy, and fixing all Markdown rendering issues.

## Technical Context

**Language/Version**: TypeScript 5.6.2, Node.js >=20.0
**Primary Dependencies**: Docusaurus 3.9.2, React 19.0.0, Node.js ecosystem
**Storage**: N/A (static site generator)
**Testing**: Docusaurus build process, manual browser testing
**Target Platform**: Web (GitHub Pages deployment)
**Project Type**: Web/static site
**Performance Goals**: Fast loading times, proper SEO, responsive design
**Constraints**: Static site limitations, GitHub Pages deployment constraints
**Scale/Scope**: Educational book with 3 modules, 3 chapters each, plus supporting content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [X] Spec-first development: Implementation follows spec.md requirements
- [X] Production-ready architecture: Docusaurus provides production-ready static site
- [X] Clear API contracts: Docusaurus configuration provides clear interface contracts
- [X] Technology Stack Requirements: Uses Docusaurus, Markdown, GitHub Pages as specified in constitution

## Project Structure

### Documentation (this feature)

```text
specs/3-docusaurus-book-fix/
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
├── docusaurus.config.ts    # Docusaurus configuration file
├── sidebars.ts             # Navigation sidebar configuration
├── docs/                   # Book content files
│   ├── intro.md
│   └── modules/            # Module directories
│       ├── ros2-nervous-system/
│       ├── digital-twin/
│       ├── ai-robot-brain/
│       └── vla/
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