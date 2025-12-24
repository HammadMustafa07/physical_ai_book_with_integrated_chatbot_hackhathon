# Data Model: Docusaurus Book Update & Fix

## Overview
This document defines the data model for the Physical AI & Humanoid Robotics book, representing the entities and relationships identified in the feature specification.

## Entities

### Book Homepage
**Description**: The main landing page that displays the book title and provides navigation to content
**Fields**:
- title: "Physical AI & Humanoid Robotics" (required, string)
- url: "/" (required, string)
- navigation: [Home, Book Content] (required, array of navigation items)

**Validation Rules**:
- Title must match exactly "Physical AI & Humanoid Robotics"
- URL must be root path "/"
- Navigation must contain exactly 2 items

### Navigation Bar
**Description**: The top navigation that contains only essential links (Home, Book Content)
**Fields**:
- items: [NavigationItem] (required, array)
- visible: true (required, boolean)

**Validation Rules**:
- Must contain exactly 2 navigation items
- Items must be "Home" and "Book Content"
- No other navigation elements allowed

### NavigationItem
**Description**: A single item in the navigation bar
**Fields**:
- label: string (required)
- url: string (required)
- position: "left" | "right" (required)

**Validation Rules**:
- Label must be one of: "Home", "Book Content"
- URL must be valid path

### Module Structure
**Description**: The hierarchical organization of content into numbered modules and chapters
**Fields**:
- id: string (required)
- title: string (required)
- chapters: [Chapter] (required, array)
- number: integer (required, 1-3)

**Validation Rules**:
- Must have exactly 3 modules
- Each module number must be 1, 2, or 3
- Each module must have exactly 3 chapters

### Chapter
**Description**: A chapter within a module
**Fields**:
- id: string (required)
- title: string (required)
- module: Module (required, reference)
- number: integer (required, 1-3)

**Validation Rules**:
- Each chapter number must be 1, 2, or 3
- Each chapter must belong to exactly one module
- Each module must have exactly 3 chapters

### Markdown Content
**Description**: The educational content files that must render correctly without formatting issues
**Fields**:
- filePath: string (required)
- frontmatter: object (required)
- content: string (required)
- headings: [Heading] (optional)
- codeBlocks: [CodeBlock] (optional)
- lists: [List] (optional)

**Validation Rules**:
- Must have valid frontmatter
- Headings must follow proper hierarchy (h1, h2, h3, etc.)
- Code blocks must have proper fencing
- No rendering errors or console warnings