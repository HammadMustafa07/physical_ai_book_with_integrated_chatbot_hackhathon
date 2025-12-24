# Quickstart Guide: Docusaurus Book Update & Fix

## Prerequisites
- Node.js >= 20.0
- npm or yarn package manager
- Git for version control

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
cd book_frontend
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Development Server
```bash
npm run start
```
This will start the Docusaurus development server at http://localhost:3000

### 4. Verify Current Configuration
- Check that the site is running at http://localhost:3000
- Note the current navbar items and structure
- Review the current module structure in the sidebar

## Key Files to Modify

### Docusaurus Configuration
- **File**: `docusaurus.config.ts`
- **Changes needed**:
  - Update navbar items to include "Physical AI And Humanoid Robotics" and "Book"
  - Configure "Book" item to redirect to Module 1 → Chapter 1
  - Remove other navigation items

### Sidebar Configuration
- **File**: `sidebars.ts`
- **Changes needed**:
  - Restructure to have exactly 4 modules with 3 chapters each
  - Ensure proper Module → Chapter hierarchy
  - Remove reference to intro page

### Content Files
- **Directory**: `docs/`
- **Changes needed**:
  - Remove `intro.md` file
  - Restructure VLA content into Module 4 with 3 chapters
  - Ensure all Markdown files render correctly

## Testing the Changes

### 1. Development Server Test
```bash
npm run start
```
- Verify the new navbar items appear correctly
- Check that "Physical AI And Humanoid Robotics" links to homepage
- Verify that "Book" redirects to Module 1 → Chapter 1
- Navigate through the module structure

### 2. Build Test
```bash
npm run build
```
- Ensure the build completes without errors
- Check for any warnings in the build output

### 3. Serve Build Locally
```bash
npm run serve
```
- Test the production build locally at http://localhost:3000

## Common Issues and Solutions

### Build Errors
- If you encounter build errors, check the console output for specific error messages
- Verify all Markdown files have proper frontmatter
- Check that all file paths in sidebars.ts are correct

### Navigation Issues
- If navigation doesn't appear as expected, verify the navbar configuration in docusaurus.config.ts
- Ensure sidebar items in sidebars.ts point to existing files

### Rendering Problems
- If Markdown content doesn't render correctly, check for proper heading hierarchy
- Verify code blocks have proper language tags
- Ensure lists are formatted correctly

### Missing Content
- If Module 4 content is missing, ensure VLA content has been properly restructured
- Verify all 4 modules and their 3 chapters each are present in the sidebar