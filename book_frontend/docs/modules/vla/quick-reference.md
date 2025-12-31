---
sidebar_position: 7
title: Quick Reference
---

# Vision-Language-Action (VLA) Quick Reference

## Command Structure

### Voice-to-Action Commands
- **Basic Navigation**: "move forward", "turn left", "go to [location]"
- **Manipulation**: "pick up [object]", "place [object] on [surface]"
- **Perception**: "find [object]", "look at [location]"

### Cognitive Planning Patterns
- **Simple Tasks**: "Do [action] to [object]"
- **Complex Tasks**: "Go to [location] and [action] [object]"
- **Multi-step**: "[Task 1], then [Task 2], then [Task 3]"

## Common Parameters

### Navigation Parameters
- `distance`: Distance to move (e.g., "move forward 2 meters")
- `location`: Named location (e.g., "go to kitchen")
- `direction`: Direction to turn (left, right, around)

### Manipulation Parameters
- `object`: Object to manipulate (name or description)
- `target`: Target location or surface

## ROS 2 Integration

### Common Topics
- `/voice_command`: Input for voice commands
- `/cmd_vel`: Robot velocity commands
- `/detected_objects`: Output from perception system
- `/vla_status`: System status updates

### Message Types
- `std_msgs/String`: For voice commands and status
- `geometry_msgs/Twist`: For movement commands
- `sensor_msgs/Image`: For camera data
- `sensor_msgs/LaserScan`: For obstacle detection

## Error Handling

### Common Error Codes
- `E001`: Speech recognition failure
- `E002`: Plan infeasible
- `E003`: Navigation obstacle
- `E004`: Object not found

### Recovery Strategies
1. **Retry**: Attempt the same action again
2. **Alternative**: Use a different approach to the same goal
3. **Ask for Help**: Request human assistance
4. **Abort**: Safely stop and report failure


