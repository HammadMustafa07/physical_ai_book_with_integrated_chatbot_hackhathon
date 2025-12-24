# Quickstart Guide: Homepage Theme & Section Redesign

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up in `book_frontend/`
- Git repository with current branch `001-homepage-theme-redesign`

## Setup and Installation

1. **Navigate to the project directory**:
   ```bash
   cd book_frontend
   ```

2. **Install dependencies** (if not already installed):
   ```bash
   npm install
   ```

## Implementation Steps

### Step 1: Update Theme Colors
1. Open `src/css/custom.css`
2. Replace the existing primary color variables with navy blue theme
3. Add complementary secondary color
4. Ensure WCAG AA compliance for contrast ratios

### Step 2: Modify Homepage Content
1. Open `src/pages/index.tsx`
2. Remove the `<HomepageFeatures />` component from the main layout
3. Add two new custom sections with the specified content
4. Style the sections appropriately

### Step 3: Create Custom Components (if needed)
1. Create new components for the custom sections if they require complex logic
2. Add appropriate CSS modules for styling

## Running and Testing

1. **Start development server**:
   ```bash
   npm run start
   ```

2. **Verify changes**:
   - Check that navy blue theme is applied globally
   - Verify that default Docusaurus features section is removed
   - Confirm that two new content sections are displayed correctly
   - Test responsive behavior on different screen sizes

3. **Build for production**:
   ```bash
   npm run build
   ```

4. **Verify build succeeds without errors**

## Common Issues and Solutions

- **Theme not applying**: Ensure CSS variables are properly defined in `custom.css`
- **Build errors**: Check for TypeScript/React syntax errors in modified files
- **Responsive issues**: Verify Docusaurus grid classes are used correctly
- **Color contrast**: Use browser dev tools to verify WCAG compliance