# Architecture Plan: Dark Mode, Structure & UI Polish

**Feature**: Dark Mode, Structure & UI Polish
**Branch**: 006-dark-mode-ui-polish
**Created**: 2025-12-23

## Overview

This plan addresses the following key areas:
1. Dark mode UI improvements for better contrast and readability
2. Chapter numbering consistency for Modules 2 and 3
3. Navbar interaction fixes (remove black focus outlines, improve hover effects)
4. Footer UI enhancement for professional appearance

## Architecture Sketch

### 1. Dark Mode Improvements
**Files to modify**: `book_frontend/src/css/custom.css`

**Current state**: Basic dark mode with `[data-theme='dark']` selector
**Target state**: Enhanced dark mode with improved contrast ratios and readability

**Approach**:
- Enhance color variables in dark mode to meet WCAG 2.1 AA contrast standards
- Improve text contrast for better readability
- Adjust background colors for better visual balance
- Ensure code blocks and syntax highlighting are readable

**Key variables to update**:
- `--ifm-color-content` and `--ifm-color-content-secondary` for better text contrast
- `--ifm-background-color` and `--ifm-background-surface-color` for improved background contrast
- Code highlighting background in dark mode

### 2. Navbar Interaction Fixes
**Files to modify**: `book_frontend/src/css/custom.css`

**Current state**: Navbar has focus outline issue at lines 98-105 in custom.css
**Target state**: Clean navbar interactions without black focus outlines, with improved hover effects

**Approach**:
- Remove or modify the focus outline on navbar links
- Implement smooth, accessible hover effects
- Ensure hover states maintain proper contrast in both light and dark modes
- Apply consistent transition timing across all navbar interactions

**Specific changes**:
- Update the `a:focus, button:focus, input:focus, select:focus, textarea:focus` selector to use more appropriate colors
- Enhance hover effects for `.navbar__item:hover` with better transitions

### 3. Footer UI Enhancement
**Files to modify**: `book_frontend/docusaurus.config.ts` (footer configuration) and `book_frontend/src/css/custom.css` (styling)

**Current state**: Basic Docusaurus footer with default styling
**Target state**: Professional, clean footer with improved spacing, typography, and visual consistency

**Approach**:
- Update footer configuration in docusaurus.config.ts to potentially add more structured content
- Add custom CSS for improved footer styling
- Ensure footer looks professional in both light and dark modes
- Improve spacing and typography for better visual hierarchy

### 4. Chapter Numbering Consistency
**Files to modify**: All markdown files in `book_frontend/docs/modules/digital-twin/` and `book_frontend/docs/modules/ai-robot-brain/`

**Current state**: Modules 2 and 3 chapters don't have "Chapter X:" prefix in titles (e.g., "Digital Twins & Physics Simulation with Gazebo")
**Target state**: Modules 2 and 3 chapters have consistent "Chapter X:" prefix like Modules 1 and 4 (e.g., "Chapter 1: Digital Twins & Physics Simulation with Gazebo")

**Approach**:
- Update the `title` field in the frontmatter of all chapter files in Modules 2 and 3
- Update the main heading (H1) in the content to match the new title format
- Ensure sidebar positions remain unchanged to maintain navigation order

## Technical Implementation

### Phase 1: Dark Mode Improvements
1. Update color variables in `custom.css` for dark mode
2. Test contrast ratios using accessibility tools
3. Verify readability across different content types (text, code, tables)

### Phase 2: Navbar Fixes
1. Modify focus outline behavior in `custom.css`
2. Enhance hover effects with smooth transitions
3. Test accessibility compliance

### Phase 3: Footer Enhancement
1. Update footer styling in `custom.css`
2. Potentially enhance footer configuration in `docusaurus.config.ts`
3. Verify responsive behavior

### Phase 4: Chapter Numbering
1. Update all Module 2 chapter files (digital-twin module)
2. Update all Module 3 chapter files (ai-robot-brain module)
3. Verify sidebar navigation remains intact

## Quality Validation

### Dark Mode Validation
- [ ] Contrast ratios meet WCAG 2.1 AA standards (4.5:1 for normal text, 3:1 for large text)
- [ ] Text remains readable in all content areas
- [ ] Code blocks maintain readability
- [ ] Background colors provide good visual balance

### Navbar Validation
- [ ] No black focus/outline borders appear on navbar links
- [ ] Hover effects are smooth (transition under 300ms)
- [ ] Hover effects maintain accessible contrast ratios in both modes
- [ ] Focus indicators remain accessible for keyboard users

### Footer Validation
- [ ] Footer appears professional and consistent with site theme
- [ ] Proper spacing and typography implemented
- [ ] Readability maintained in both light and dark modes
- [ ] Responsive behavior preserved

### Chapter Numbering Validation
- [ ] Modules 2 and 3 display explicit chapter numbering in sidebar
- [ ] Page titles include "Chapter X:" prefix
- [ ] Navigation order remains unchanged
- [ ] No broken links or navigation issues

## Dependencies and Risks

### Dependencies
- Docusaurus framework (already present)
- Existing content structure (no breaking changes planned)

### Risks
- Theme updates might override CSS customizations
- Chapter title changes might affect search indexing
- Focus outline removal might impact accessibility if not handled properly

## Testing Strategy

1. Manual testing across different browsers (Chrome, Firefox, Safari)
2. Accessibility testing with tools like axe-core
3. Contrast ratio validation using WebAIM contrast checker
4. Responsive design testing across different screen sizes
5. Dark/light mode switching validation