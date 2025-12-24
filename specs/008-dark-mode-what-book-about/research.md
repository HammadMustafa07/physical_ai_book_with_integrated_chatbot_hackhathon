# Research Document: Dark Mode UI Upgrade for "What This Book Is About" Section

## Decision: WCAG 2.1 AA Compliance for Dark Mode

**Rationale**: WCAG 2.1 AA standards require a minimum contrast ratio of 4.5:1 for normal text and 3:1 for large text. This ensures accessibility for users with visual impairments.

**Alternatives Considered**:
- WCAG AA (4.5:1 for normal text, 3:1 for large text) - Chosen for compliance
- WCAG AAA (7:1 for normal text, 4.5:1 for large text) - Rejected as unnecessarily strict for this context

## Decision: Docusaurus CSS Variable System

**Rationale**: Docusaurus uses CSS variables for theme customization. The system leverages `:root` for light mode and `[data-theme='dark']` for dark mode overrides.

**Key Variables**:
- `--ifm-color-content`: Main content text color
- `--ifm-color-content-secondary`: Secondary content text color
- `--ifm-background-color`: Background color
- `--ifm-background-surface-color`: Surface background color

## Decision: Section Structure Analysis

**Rationale**: The "What This Book Is About" section is located in `book_frontend/src/pages/index.tsx` with CSS class `.aboutSection` and is styled in `book_frontend/src/pages/index.module.css`.

**Findings**:
- Section uses Docusaurus Layout and Heading components
- Contains title, description paragraph, and lists
- Current styling applies light mode colors with default dark mode behavior

## Decision: Dark Mode Typography Best Practices

**Rationale**: Dark mode typography requires careful consideration of contrast, brightness, and readability.

**Best Practices Applied**:
- Use lighter text colors (#f0f0f0 for primary, #e0e0e0 for secondary) on dark backgrounds
- Avoid pure black/white to prevent visual strain
- Ensure adequate spacing and visual hierarchy
- Maintain consistent font sizing and line height