# Research: Docusaurus Book Update & Fix

## Overview
This research document captures findings and decisions made during the planning phase for updating and fixing the Docusaurus-based Physical AI & Humanoid Robotics book.

## Decision: Site Title and Homepage Configuration
**Rationale**: The current Docusaurus configuration has a generic title "My Site" that needs to be updated to "Physical AI & Humanoid Robotics" as specified in the requirements. The homepage needs to be configured to serve as the book landing page.

**Implementation Approach**:
- Update `title` field in `docusaurus.config.ts`
- Update `navbar.title` field in `docusaurus.config.ts`
- Configure the homepage to be the root URL (/)

**Alternatives considered**:
- Keep existing title: Would not meet requirements
- Use different title: Would not align with specified requirements

## Decision: Navbar Cleanup
**Rationale**: The current navbar includes Blog and Tutorial links that need to be removed to provide a clean navigation experience with only essential items.

**Implementation Approach**:
- Remove Blog-related items from navbar configuration
- Remove Tutorial label from sidebar type navigation
- Add "Book Content" link that points to the docs root
- Maintain GitHub link for development transparency

**Alternatives considered**:
- Keep existing navbar: Would not meet clean navigation requirements
- Add more navigation items: Would contradict the minimal navigation requirement

## Decision: Module Structure Enforcement
**Rationale**: The current structure has 4 modules with varying numbers of chapters. The specification requires exactly 3 modules with 3 chapters each, all properly numbered.

**Current Structure Analysis**:
- Module 1: "ROS 2 Nervous System" (3 chapters)
- Module 2: "Digital Twin (Gazebo & Unity)" (3 chapters)
- Module 3: "The AI-Robot Brain (NVIDIA Isaac™)" (3 chapters)
- Module 4: "Vision-Language-Action (VLA)" (7 chapters)

**Implementation Approach**:
- Keep first 3 modules as they already have 3 chapters each
- Restructure VLA module to have exactly 3 chapters by grouping content
- Update sidebar configuration to reflect the exact Module → Chapter hierarchy
- Ensure all modules and chapters are explicitly numbered

**Alternatives considered**:
- Keep existing 4 modules: Would not meet the 3-module requirement
- Create completely new modules: Would lose existing content structure

## Decision: Markdown Rendering Fixes
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