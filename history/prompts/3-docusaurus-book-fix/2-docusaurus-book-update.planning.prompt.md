---
id: 2
title: "Docusaurus Book Update Planning"
stage: "plan"
date_iso: "2025-12-21"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "3-docusaurus-book-fix"
branch: "003-docusaurus-book-fix"
user: "Abro"
command: "/sp.plan ---\n\n## `/sp.plan` — Fix & Stabilize Docusaurus Book\n\n**Project:** Physical AI & Humanoid Robotics\n**Repo:** `book_frontend`\n**Framework:** Docusaurus\n**Methodology:** Spec-Driven Development (Spec-Kit Plus)\n\n---\n\n### 🎯 Goal\n\nImplement all changes defined in `/sp.specify` to make the Docusaurus book **correctly structured, stable, and production-ready**.\n\n---\n\n### 🧩 Implementation Steps\n\n#### Phase 1 — Site Configuration\n\n1. Update `docusaurus.config.js`\n\n   * Change site title to **Physical AI & Humanoid Robotics**\n   * Configure `/` as the book homepage\n2. Clean navbar\n\n   * Add **Home** (points to `/`)\n   * Add **Book Content** (docs root)\n   * Remove Blog and Tutorials\n\n---\n\n#### Phase 2 — Docs Structure Enforcement\n\n3. Refactor `/docs` folder\n\n   * Create `module-1`, `module-2`, `module-3`\n   * Ensure each module contains exactly 3 chapters\n4. Update frontmatter\n\n   * Explicit module and chapter numbering\n   * Clear, descriptive titles\n5. Align `sidebars.js`\n\n   * Match exact Module → Chapter hierarchy\n\n---\n\n#### Phase 3 — Markdown Fixes\n\n6. Audit Module 1 Markdown\n\n   * Fix headings, lists, code blocks, frontmatter\n   * Remove malformed MDX\n7. Validate rendering\n\n   * No broken layouts or formatting issues\n\n---\n\n#### Phase 4 — Validation & Build\n\n8. Run local checks\n\n   * `npm run start`\n   * `npm run build`\n9. Fix any warnings or errors\n10. Confirm GitHub Pages deploy readiness\n\n---\n\n### ✅ Exit Criteria\n\n* Homepage loads correctly at `/`\n* Navbar matches spec exactly\n* Sidebar hierarchy is correct\n* All Markdown renders cleanly\n* Docusaurus builds with zero errors\n\n---"
labels: ["docusaurus", "book", "frontend", "planning"]
links:
  spec: "specs/3-docusaurus-book-fix/spec.md"
  plan: "specs/3-docusaurus-book-fix/plan.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/3-docusaurus-book-fix/plan.md
  - specs/3-docusaurus-book-fix/research.md
  - specs/3-docusaurus-book-fix/data-model.md
  - specs/3-docusaurus-book-fix/quickstart.md
  - specs/3-docusaurus-book-fix/contracts/configuration-contracts.md
  - history/prompts/3-docusaurus-book-fix/2-docusaurus-book-update.planning.prompt.md
tests_yaml: |
  - "Implementation plan completeness"
  - "Research document validation"
  - "Data model accuracy"
  - "Quickstart guide verification"
prompt_text: "---\n\n## `/sp.plan` — Fix & Stabilize Docusaurus Book\n\n**Project:** Physical AI & Humanoid Robotics\n**Repo:** `book_frontend`\n**Framework:** Docusaurus\n**Methodology:** Spec-Driven Development (Spec-Kit Plus)\n\n---\n\n### 🎯 Goal\n\nImplement all changes defined in `/sp.specify` to make the Docusaurus book **correctly structured, stable, and production-ready**.\n\n---\n\n### 🧩 Implementation Steps\n\n#### Phase 1 — Site Configuration\n\n1. Update `docusaurus.config.js`\n\n   * Change site title to **Physical AI & Humanoid Robotics**\n   * Configure `/` as the book homepage\n2. Clean navbar\n\n   * Add **Home** (points to `/`)\n   * Add **Book Content** (docs root)\n   * Remove Blog and Tutorials\n\n---\n\n#### Phase 2 — Docs Structure Enforcement\n\n3. Refactor `/docs` folder\n\n   * Create `module-1`, `module-2`, `module-3`\n   * Ensure each module contains exactly 3 chapters\n4. Update frontmatter\n\n   * Explicit module and chapter numbering\n   * Clear, descriptive titles\n5. Align `sidebars.js`\n\n   * Match exact Module → Chapter hierarchy\n\n---\n\n#### Phase 3 — Markdown Fixes\n\n6. Audit Module 1 Markdown\n\n   * Fix headings, lists, code blocks, frontmatter\n   * Remove malformed MDX\n7. Validate rendering\n\n   * No broken layouts or formatting issues\n\n---\n\n#### Phase 4 — Validation & Build\n\n8. Run local checks\n\n   * `npm run start`\n   * `npm run build`\n9. Fix any warnings or errors\n10. Confirm GitHub Pages deploy readiness\n\n---\n\n### ✅ Exit Criteria\n\n* Homepage loads correctly at `/`\n* Navbar matches spec exactly\n* Sidebar hierarchy is correct\n* All Markdown renders cleanly\n* Docusaurus builds with zero errors\n\n---"
response_text: "Completed the planning phase for the Docusaurus Book Update & Fix feature. Created comprehensive implementation plan with technical context, research findings, data model, quickstart guide, and configuration contracts. The plan addresses all requirements from the specification including site title update, navbar cleanup, module structure enforcement, and markdown rendering fixes. All Phase 0 and Phase 1 deliverables have been created as per the Spec-Kit Plus methodology."
outcome: "Planning phase successfully completed with all required artifacts. The implementation plan is ready to proceed to the tasks generation phase (/sp.tasks)."
evaluation: "The planning process followed the Spec-Kit Plus methodology correctly, producing all required artifacts (plan.md, research.md, data-model.md, quickstart.md, contracts/). The plan comprehensively addresses all aspects of the original feature request with appropriate technical context and implementation approach."
---

# Docusaurus Book Update & Fix - Planning Phase

## Summary of Work Completed

I have completed the planning phase for updating and fixing the Docusaurus-based Physical AI & Humanoid Robotics book. The planning includes:

1. **Implementation Plan** (specs/3-docusaurus-book-fix/plan.md): Comprehensive technical approach with architecture decisions
2. **Research Findings** (specs/3-docusaurus-book-fix/research.md): Technical decisions and implementation approaches
3. **Data Model** (specs/3-docusaurus-book-fix/data-model.md): Entity definitions for the book structure
4. **Quickstart Guide** (specs/3-docusaurus-book-fix/quickstart.md): Setup and development instructions
5. **Configuration Contracts** (specs/3-docusaurus-book-fix/contracts/configuration-contracts.md): Interface specifications

## Key Technical Decisions

- Updated site title to "Physical AI & Humanoid Robotics" in docusaurus.config.ts
- Cleaned navbar to include only "Home" and "Book Content" items
- Enforced 3-module structure with 3 chapters each in sidebars.ts
- Identified Markdown rendering fixes needed across all content files

## Next Steps

The planning phase is complete and ready for the tasks generation phase (/sp.tasks). All required artifacts have been created following the Spec-Kit Plus methodology.