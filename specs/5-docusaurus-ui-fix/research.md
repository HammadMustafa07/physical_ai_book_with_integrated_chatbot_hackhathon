# Research: Docusaurus UI, Structure & Metadata Fix

## Overview
This research document captures findings and decisions made during the planning phase for fixing UI, navigation, structural inconsistencies, and metadata issues in the Docusaurus-based Physical AI & Humanoid Robotics book.

## Decision: Navbar Cleanup - Keep Logo-based Navigation Only
**Rationale**: The current navbar has both a logo and a text link "Physical AI And Humanoid Robotics" that both point to the home page. To comply with FR-001 and SC-001, we need to keep only the logo-based navigation item.

**Implementation Approach**:
- Remove the text-based "Physical AI And Humanoid Robotics" link from the navbar items array
- Keep the logo configuration as is, since it also serves as a home link
- This ensures only one home link exists as required

**Alternatives considered**:
- Keep text link and remove logo: Would not be as intuitive for users
- Keep both: Would violate the requirement for only one home link

## Decision: Hero Section Button Text Update
**Rationale**: The current hero button text "Start Learning Physical AI - 5min ⏱️" needs to be updated to exactly "Start learning From Today" as specified in FR-003 and SC-002.

**Implementation Approach**:
- Update the button text in `src/pages/index.tsx` from "Start Learning Physical AI - 5min ⏱️" to "Start learning From Today"
- Keep the same link destination to maintain functionality

**Alternatives considered**:
- Different button text: Would not meet the exact requirement
- No text change: Would not satisfy the requirement

## Decision: Module 2 & 3 Chapter Renaming Strategy
**Rationale**: The specification requires Module 2 and Module 3 chapters to be properly numbered as Chapter 1, Chapter 2, Chapter 3. Currently they appear to follow a different naming pattern.

**Current Structure Analysis**:
- Module 2: "Digital Twin (Gazebo & Unity)" with chapters: chapter-1-physics-simulation, chapter-2-sensor-simulation, chapter-3-unity-interaction
- Module 3: "The AI-Robot Brain (NVIDIA Isaac™)" with chapters: chapter-1-isaac-sim, chapter-2-isaac-ros, chapter-3-nav2-navigation

**Implementation Approach**:
- Keep the existing file names and paths to maintain existing links
- Update the chapter titles within the markdown files to start with "Chapter 1", "Chapter 2", "Chapter 3" as appropriate
- This maintains file structure while meeting the requirement for proper chapter numbering in content

**Alternatives considered**:
- Rename the actual files: Would break existing links and require updating all references
- Only update sidebar: Would not address the content itself

## Decision: Logo Configuration
**Rationale**: The site currently uses "img/logo.svg" but the requirement specifies using "logo.png" as mentioned in FR-005 and SC-004.

**Current State Analysis**:
- Current config: `src: 'img/logo.svg'`
- Available logo: `static/img/logo.png` exists
- Need to update config to use logo.png

**Implementation Approach**:
- Update the logo source in `docusaurus.config.ts` from 'img/logo.svg' to 'img/logo.png'
- Ensure the logo appears correctly in the navbar and links to home page

**Alternatives considered**:
- Keep existing logo.svg: Would not meet the specific requirement for logo.png
- Create new logo: Unnecessary when existing logo.png is available

## Decision: Metadata Title Update
**Rationale**: The current site title is "Physical AI & Humanoid Robotics" but needs to be updated to "Physical AI & Humanoid Robotics Book" as specified in FR-006 and SC-005.

**Implementation Approach**:
- Update the title field in `docusaurus.config.ts` from 'Physical AI & Humanoid Robotics' to 'Physical AI & Humanoid Robotics Book'
- This will update the page metadata and browser tab title

**Alternatives considered**:
- Only update specific pages: Would not provide site-wide consistency
- Keep existing title: Would not meet the requirement

## Decision: UI Refinement Boundaries
**Rationale**: The requirement allows "Layout refinement, Typography improvements, Spacing and readability enhancements" but prohibits "Full theme replacement" and "Branding changes unrelated to this spec".

**Implementation Approach**:
- Focus on CSS improvements in existing custom CSS files
- Make typography and spacing improvements without changing the overall Docusaurus theme
- Enhance readability without major visual redesign

**Constraints**:
- Do not replace the Docusaurus theme
- Do not make significant branding changes beyond specified requirements
- Maintain existing color scheme and overall design aesthetic