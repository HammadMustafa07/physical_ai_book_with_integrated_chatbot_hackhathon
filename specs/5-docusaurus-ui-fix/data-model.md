# Data Model: Docusaurus UI, Structure & Metadata Fix

## Overview
This document defines the data model for the Docusaurus UI, Structure & Metadata Fix feature, representing the entities and relationships identified in the feature specification.

## Entities

### Navbar
**Description**: The top navigation that should contain only the logo and one "Physical AI & Humanoid Robotics" link
**Fields**:
- items: [NavigationItem] (required, array)
- logo: LogoConfiguration (required)
- position: "top" (required, string)

**Validation Rules**:
- Must contain exactly one home link (either logo or text, but not both as separate home links)
- Logo must be configured and visible
- Items must be properly positioned

### NavigationItem
**Description**: A single item in the navigation bar
**Fields**:
- label: string (required)
- to: string (required, path to navigate to)
- position: "left" | "right" (required)
- type: "link" (optional)

**Validation Rules**:
- Label must be appropriate for the navigation context
- Path must be valid and accessible
- Position must be either "left" or "right"

### LogoConfiguration
**Description**: Configuration for the site logo that appears in the navbar
**Fields**:
- alt: string (required, alt text for accessibility)
- src: string (required, path to logo image file)
- href: string (optional, where logo links to, defaults to home)

**Validation Rules**:
- src must point to a valid image file
- alt text must be descriptive
- Logo must link to home page when clicked

### HeroSection
**Description**: The main landing page section with the call-to-action button that needs updated text
**Fields**:
- title: string (required, from site config)
- subtitle: string (required, from site config)
- ctaButton: CTAButtonConfiguration (required)

**Validation Rules**:
- Must contain a clear call-to-action button
- Button text must match specified requirements
- Button must link to appropriate content

### CTAButtonConfiguration
**Description**: Configuration for the call-to-action button in the hero section
**Fields**:
- text: string (required, button label)
- to: string (required, destination path)
- className: string (required, styling classes)

**Validation Rules**:
- Text must match exactly "Start learning From Today"
- Destination path must be valid
- Button must be visible and clickable

### ModuleStructure
**Description**: The hierarchical organization of content into modules with properly numbered chapters
**Fields**:
- id: string (required)
- label: string (required, display name)
- items: [ChapterItem] (required, array)
- type: "category" (required)

**Validation Rules**:
- Must contain exactly 3 chapters per module (for modules 2 and 3)
- Chapters must be properly numbered (Chapter 1, Chapter 2, Chapter 3)
- Module structure must be consistent across all modules

### ChapterItem
**Description**: A chapter within a module
**Fields**:
- id: string (required, file path reference)
- title: string (required, display title)
- module: ModuleStructure (required, reference to parent module)

**Validation Rules**:
- Title should indicate proper chapter number (Chapter 1, Chapter 2, Chapter 3)
- Path must reference an existing content file
- Must belong to exactly one module

### PageMetadata
**Description**: The title and SEO information that should reflect the correct book title
**Fields**:
- title: string (required, site title)
- tagline: string (optional, site tagline)
- description: string (optional, meta description)

**Validation Rules**:
- Title must be "Physical AI & Humanoid Robotics Book"
- Must be consistent across all pages
- Should be properly formatted for SEO