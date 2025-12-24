# Research: Docusaurus Book Update & Fix

## Overview
This research document captures findings and decisions made during the planning phase for updating and fixing the Docusaurus-based Physical AI & Humanoid Robotics book.

## Decision: Navbar Configuration
**Rationale**: The current navbar has "Home" and "Book Content" items, but the specification requires "Physical AI And Humanoid Robotics" and "Book" items with the "Book" link redirecting to Module 1 → Chapter 1.

**Implementation Approach**:
- Update `navbar.items` in `docusaurus.config.ts`
- Replace "Home" with "Physical AI And Humanoid Robotics" (redirects to "/")
- Replace "Book Content" with "Book" (redirects to Module 1 → Chapter 1)
- Remove any other navigation items

**Alternatives considered**:
- Keep existing navbar: Would not meet specification requirements
- Use different labels: Would not match exact specification requirements

## Decision: Module Structure Enforcement
**Rationale**: The current structure has 3 modules with 3 chapters each, but the specification requires exactly 4 modules with 3 chapters each.

**Current Structure Analysis**:
- Module 1: "Module 1: ROS 2 Nervous System" (3 chapters)
- Module 2: "Module 2: Digital Twin (Gazebo & Unity)" (3 chapters)
- Module 3: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)" (3 chapters)
- Missing: Module 4

**Implementation Approach**:
- Keep first 3 modules as they already have 3 chapters each
- Add Module 4 using content from the VLA (Vision-Language-Action) module
- Restructure VLA content into 3 chapters for Module 4
- Update sidebar configuration to reflect the exact 4-module structure

**Alternatives considered**:
- Keep existing 3 modules: Would not meet the 4-module requirement
- Create completely new modules: Would require significant new content creation

## Decision: Tutorial Intro Page Removal
**Rationale**: The specification requires removing the Tutorial Intro page entirely, which currently exists as `docs/intro.md`.

**Implementation Approach**:
- Remove `docs/intro.md` file
- Update sidebar configuration to no longer reference the intro page
- Ensure the "Book" navbar item redirects directly to Module 1 → Chapter 1

**Alternatives considered**:
- Keep intro page: Would not meet specification requirements
- Move intro content: Would complicate the navigation structure

## Decision: Direct Book Access Implementation
**Rationale**: The "Book" navbar item must redirect directly to Module 1 → Chapter 1, bypassing any introductory pages.

**Implementation Approach**:
- Update navbar configuration to point "Book" item to Module 1 → Chapter 1 path
- Use Docusaurus `to` property to specify direct route to first chapter
- Verify the path matches the actual location of Module 1 → Chapter 1

**Alternatives considered**:
- Keep current behavior: Would not meet direct access requirement
- Redirect to different location: Would not match specification

## Decision: Markdown Rendering Quality
**Rationale**: The specification requires all Markdown files to render correctly without formatting issues, console errors, or build warnings.

**Implementation Approach**:
- Audit all Markdown files for proper heading levels (h1, h2, h3, etc.)
- Verify correct code block fencing with proper language tags
- Check for valid frontmatter in all content files
- Ensure lists and other Markdown elements render correctly
- Test with Docusaurus build process to identify any warnings

**Alternatives considered**:
- Leave existing formatting issues: Would not meet quality requirements
- Use different content format: Would require significant rework and not align with Docusaurus approach

## Decision: GitHub Pages Deployment Readiness
**Rationale**: The site needs to be deployable to GitHub Pages as specified in the constitution.

**Implementation Approach**:
- Verify baseUrl is set to '/' for GitHub Pages
- Ensure organizationName and projectName are correctly configured
- Test build process locally before deployment
- Update GitHub Pages settings as needed

**Alternatives considered**:
- Use different deployment method: Would not align with constitution requirements