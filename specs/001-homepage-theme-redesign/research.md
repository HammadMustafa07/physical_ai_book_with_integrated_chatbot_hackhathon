# Research: Homepage Theme & Section Redesign

## Decision: Navy Blue Theme Implementation
- **Rationale**: Navy blue (#001f3f or similar) is a professional, academic-appropriate color that conveys trust, stability, and seriousness appropriate for a technical book
- **Implementation approach**: Override Infima CSS variables in custom.css to use navy blue as primary color
- **Secondary color**: Light blue (#0074D9) or gold (#FFDC00) as complementary colors for accents

## Decision: Homepage Layout Structure
- **Rationale**: Need to replace the default Docusaurus "Features" section with two custom content sections
- **Implementation approach**: Modify the index.tsx homepage to remove HomepageFeatures component and add two new custom sections
- **Section 1**: "What This Book Is About" - explaining Physical AI and Embodied Intelligence
- **Section 2**: "Course Structure" - outlining the four modules

## Decision: Responsive Design Considerations
- **Rationale**: Must maintain responsive layout across desktop, tablet, and mobile devices
- **Implementation approach**: Use Docusaurus's built-in grid system (container, row, col classes) with appropriate padding and spacing

## Decision: Accessibility Compliance
- **Rationale**: Must maintain WCAG AA compliance for color contrast
- **Implementation approach**: Choose navy blue and complementary colors that provide sufficient contrast with background and text elements
- **Standard**: Minimum 4.5:1 contrast ratio for normal text, 3:1 for large text

## Technical Implementation Path
1. Update theme colors in src/css/custom.css
2. Modify src/pages/index.tsx to remove HomepageFeatures and add custom sections
3. Create new CSS modules for custom sections if needed
4. Test responsive behavior across different screen sizes
5. Verify build process works without errors