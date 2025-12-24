# Implementation Plan: Dark Mode UI Fix (Homepage Sections)

**Feature**: Dark Mode UI Fix for Hero, "What This Book Is About", and "Course Structure" sections
**Branch**: 009-dark-mode-homepage-fix
**Created**: 2025-12-23
**Status**: Draft

## Technical Context

**Frontend Framework**: Docusaurus
**Styling Approach**: CSS variables and Docusaurus theme customization
**Target Sections**:
- Hero section (main landing area)
- "What This Book Is About" section (features/benefits)
- "Course Structure" section (outline/curriculum)

**File Locations**:
- Homepage component: `book_frontend/src/pages/index.js` or similar
- CSS/Docusaurus theme files: `book_frontend/src/css/` or `book_frontend/src/theme/`
- Custom components: `book_frontend/src/components/`

**Constraints**:
- Maintain existing light mode functionality (no regressions)
- Follow Docusaurus theming best practices
- Ensure WCAG AA compliance (4.5:1 contrast ratio for text)
- Keep changes scoped to homepage sections only

## Architecture Sketch

```
book_frontend/
├── src/
│   ├── pages/
│   │   └── index.js              # Homepage with sections
│   ├── components/
│   │   ├── HeroSection.js        # Hero section component
│   │   ├── AboutSection.js       # "What This Book Is About" section
│   │   └── StructureSection.js   # "Course Structure" section
│   ├── css/
│   │   └── custom.css            # Custom styles including dark mode
│   └── theme/
│       └── index.js              # Docusaurus theme customization
```

## Implementation Strategy

### Phase 1: Research & Analysis
- Inspect current homepage structure and styling
- Identify existing dark mode implementation
- Document current color variables and contrast ratios
- Research Docusaurus dark mode best practices

### Phase 2: Design & Preparation
- Define dark mode color palette with proper contrast ratios
- Plan CSS variable structure for theme switching
- Prepare implementation approach for each section

### Phase 3: Implementation
- Update Hero section with improved dark mode styling
- Update "What This Book Is About" section with improved dark mode styling
- Update "Course Structure" section with improved dark mode styling
- Ensure CTA buttons remain visible in dark mode
- Verify icons/emojis remain readable in dark mode

### Phase 4: Validation
- Test contrast ratios meet WCAG AA standards
- Verify no regressions in light mode
- Test responsive layouts in dark mode
- Validate build process remains clean

## Technology Stack

- **Frontend**: Docusaurus v2.x
- **Styling**: CSS variables, Docusaurus theme system
- **Standards**: WCAG 2.1 AA accessibility standards
- **Build Tool**: Node.js/npm

## Dependencies

- Docusaurus framework
- Standard web technologies (HTML, CSS, JavaScript)
- No external dependencies required

## Risk Analysis

- **Low Risk**: Pure CSS/styling changes with no functional impact
- **Mitigation**: Maintain existing light mode as fallback
- **Testing**: Visual comparison between modes

## Constitution Check

✅ **Spec-first development**: Based on approved specification
✅ **Zero hallucination tolerance**: Implementation based on actual codebase
✅ **Content-grounded responses**: N/A for UI implementation
✅ **Deterministic behavior**: CSS changes are predictable
✅ **Production-ready**: Follows Docusaurus best practices
✅ **Clear API contracts**: N/A for UI implementation

## Research Requirements

- Current dark mode implementation in Docusaurus
- WCAG contrast ratio requirements
- Docusaurus theme customization patterns
- CSS variable best practices for theming