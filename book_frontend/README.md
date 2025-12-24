# Physical AI & Humanoid Robotics

This book website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator, focusing on Physical AI & Humanoid Robotics concepts.

## About This Book

This educational resource covers:
- **Module 1**: ROS 2 Nervous System - Middleware for robot control
- **Module 2**: Digital Twin (Gazebo & Unity) - Physics simulation and environment building
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac™) - Advanced perception and training

## Installation

```bash
npm install
```

## Local Development

```bash
npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
