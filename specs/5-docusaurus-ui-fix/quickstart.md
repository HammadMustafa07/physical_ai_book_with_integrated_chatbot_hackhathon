# Quickstart Guide: Docusaurus UI, Structure & Metadata Fix

## Prerequisites
- Node.js >= 20.0
- npm or yarn package manager
- Git for version control
- Access to the `logo.png` file

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

## Key Files to Modify

### Docusaurus Configuration
- **File**: `docusaurus.config.ts`
- **Changes needed**:
  - Update site title from "Physical AI & Humanoid Robotics" to "Physical AI & Humanoid Robotics Book"
  - Update logo source from "img/logo.svg" to "img/logo.png"
  - Remove duplicate "Physical AI And Humanoid Robotics" text link from navbar (keep only logo)

### Homepage Configuration
- **File**: `src/pages/index.tsx`
- **Changes needed**:
  - Update hero button text from "Start Learning Physical AI - 5min ⏱️" to "Start learning From Today"

### Sidebar Configuration
- **File**: `sidebars.ts`
- **Changes needed**:
  - Verify Module 2 and Module 3 have proper chapter structure
  - Ensure chapters are numbered as Chapter 1, Chapter 2, Chapter 3 in content

## Testing the Changes

### 1. Development Server Test
```bash
npm run start
```
- Verify navbar has only logo and "Book" link (no duplicate text home link)
- Check that hero button text is "Start learning From Today"
- Navigate through modules to verify proper chapter numbering
- Ensure logo.png displays correctly in navbar
- Verify page title is "Physical AI & Humanoid Robotics Book"

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
- Verify all UI fixes are present in production build

## Common Issues and Solutions

### Missing Logo
- If logo doesn't appear, verify `static/img/logo.png` exists
- Check that path in config matches the actual file location

### Navbar Links
- If duplicate home links appear, double-check navbar items array in config
- Ensure only logo-based navigation remains for home access

### Build Errors
- If you encounter build errors, check the console output for specific error messages
- Verify all file paths in configurations are correct
- Ensure no syntax errors were introduced during changes

### Chapter Numbering
- If chapters don't display proper numbering, check the content files directly
- Update chapter titles within markdown files to reflect Chapter 1, Chapter 2, Chapter 3 format