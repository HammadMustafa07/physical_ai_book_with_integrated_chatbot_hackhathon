# Quickstart Guide: Dark Mode UI Upgrade for "What This Book Is About" Section

## Overview
This guide provides essential information for implementing the dark mode UI upgrade for the "What This Book Is About" section.

## Project Structure
```
book_frontend/
├── src/
│   ├── css/
│   │   └── custom.css          # Main CSS file for theme customization
│   └── pages/
│       ├── index.tsx           # Homepage with "What This Book Is About" section
│       └── index.module.css    # Module-specific styles
```

## Key CSS Variables
- `--ifm-color-content`: Main content text color in dark mode
- `--ifm-color-content-secondary`: Secondary text color in dark mode
- `--ifm-background-color`: Background color in dark mode
- `--ifm-background-surface-color`: Surface background color in dark mode

## Implementation Steps
1. Update CSS variables in `custom.css` for dark mode
2. Modify `.aboutSection` styles in `index.module.css` for dark mode
3. Test contrast ratios using accessibility tools
4. Verify responsive behavior across devices
5. Confirm no regressions in light mode

## Testing Commands
```bash
# Build the site to verify no errors
cd book_frontend && npm run build

# Start development server
cd book_frontend && npm run start
```

## Quality Checks
- Contrast ratio ≥ 4.5:1 for normal text, ≥ 3:1 for large text
- Section remains readable and well-structured in dark mode
- No visual regressions in light mode
- Responsive design works properly in both modes