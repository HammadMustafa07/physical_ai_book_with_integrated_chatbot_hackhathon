# Data Model: Dark Mode UI Upgrade for "What This Book Is About" Section

## Entities

### SectionContent
- **name**: string (e.g., "What This Book Is About")
- **title**: string (the main heading text)
- **description**: string (the descriptive paragraph)
- **contentElements**: array of ContentElement
- **styles**: object (CSS styling properties for light/dark modes)

### ContentElement
- **type**: enum (title, paragraph, list, conceptBridge)
- **content**: string (the actual content text)
- **styling**: object (CSS properties specific to this element)
- **visibility**: boolean (whether element is visible in dark mode)

### DarkModeStyling
- **backgroundColor**: string (dark mode background color)
- **textColor**: string (dark mode text color)
- **contrastRatio**: number (measured contrast ratio)
- **spacing**: object (padding, margin properties)
- **typography**: object (font size, line height, weight properties)

## Relationships
- SectionContent contains multiple ContentElement instances
- ContentElement has associated DarkModeStyling
- DarkModeStyling inherits from global theme variables

## Validation Rules
- All text elements must have contrast ratio >= 4.5:1 (normal text) or 3:1 (large text)
- Content elements must maintain proper visual hierarchy
- Styling must be responsive across screen sizes
- Light mode appearance must remain unchanged

## State Transitions
- Light Mode → Dark Mode: CSS variables change based on [data-theme='dark'] selector
- Responsive States: Layout adjusts based on media queries