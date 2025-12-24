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
- **Expected Items**: Exactly 2 items - "Physical AI And Humanoid Robotics" and "Book"

## Sidebar Configuration Contract

### Module Structure Contract
- **Property**: `tutorialSidebar` structure
- **Location**: `sidebars.ts`
- **Type**: Sidebar configuration object
- **Required**: Yes
- **Expected Structure**: 4 modules, each with exactly 3 chapters

## Content Structure Contract

### Module Count Contract
- **Property**: Total number of modules
- **Location**: `sidebars.ts`
- **Type**: Integer
- **Required**: Yes
- **Value**: Exactly 4 modules

### Chapter Count Contract
- **Property**: Number of chapters per module
- **Location**: `sidebars.ts`
- **Type**: Integer
- **Required**: Yes
- **Value**: Exactly 3 chapters per module

## Page Structure Contract

### Tutorial Intro Removal Contract
- **Property**: Presence of intro page
- **Location**: `docs/intro.md`
- **Type**: Boolean (should not exist)
- **Required**: Yes
- **Value**: File must be removed

## Navigation Behavior Contract

### Book Redirect Contract
- **Property**: "Book" navbar item redirect
- **Location**: `docusaurus.config.ts`
- **Type**: String (URL path)
- **Required**: Yes
- **Value**: Must redirect to Module 1 → Chapter 1