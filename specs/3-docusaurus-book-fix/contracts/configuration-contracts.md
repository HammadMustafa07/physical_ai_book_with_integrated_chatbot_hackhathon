# Docusaurus Configuration Contracts

## Overview
This document defines the configuration contracts for the Docusaurus-based Physical AI & Humanoid Robotics book.

## Site Configuration Contract

### Site Title Configuration
- **Property**: `title`
- **Location**: `docusaurus.config.ts`
- **Type**: string
- **Required**: Yes
- **Value**: "Physical AI & Humanoid Robotics"

### Navigation Configuration
- **Property**: `themeConfig.navbar.items`
- **Location**: `docusaurus.config.ts`
- **Type**: Array of navigation items
- **Required**: Yes
- **Expected Items**: Exactly 2 items - "Home" and "Book Content"

## Sidebar Configuration Contract

### Module Structure Contract
- **Property**: `tutorialSidebar` structure
- **Location**: `sidebars.ts`
- **Type**: Sidebar configuration object
- **Required**: Yes
- **Expected Structure**: 3 modules, each with exactly 3 chapters

## Content Rendering Contract

### Markdown Content Requirements
- **Property**: All .md files in docs/
- **Location**: `docs/` directory
- **Type**: Markdown with frontmatter
- **Required**: Yes
- **Requirements**: Valid frontmatter, proper heading hierarchy, correct code fencing