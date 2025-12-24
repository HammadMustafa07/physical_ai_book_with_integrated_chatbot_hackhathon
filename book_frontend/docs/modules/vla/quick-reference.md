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
- `grip_force`: Force for grasping (light, medium, firm)

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
- `E005`: Grasp failure

### Recovery Strategies
1. **Retry**: Attempt the same action again
2. **Alternative**: Use a different approach to the same goal
3. **Ask for Help**: Request human assistance
4. **Abort**: Safely stop and report failure

## Performance Tips

### Optimizing Speech Recognition
- Use clear, consistent command vocabulary
- Speak at moderate pace and volume
- Minimize background noise
- Use confirmation prompts for critical commands

### Efficient Planning
- Break complex goals into simpler subtasks
- Use context to disambiguate commands
- Implement plan caching for common tasks
- Validate plans before execution

### Robust Execution
- Monitor execution progress continuously
- Implement timeout mechanisms
- Use feedback to adjust actions
- Plan for graceful degradation

## Debugging Commands

### System Status
- Check all nodes: `ros2 node list`
- Monitor topics: `ros2 topic list`
- View messages: `ros2 topic echo [topic_name]`

### VLA-Specific Debugging
- Voice input: `ros2 topic echo /voice_command`
- System status: `ros2 topic echo /vla_status`
- Navigation: `ros2 topic echo /cmd_vel`
- Perception: `ros2 topic echo /detected_objects`

## Best Practices

### Design Principles
1. **Safety First**: Always prioritize safe robot behavior
2. **User-Centric**: Design commands that match user expectations
3. **Robustness**: Handle errors gracefully
4. **Scalability**: Design for future feature additions

### Testing Strategies
- Test with diverse user commands
- Validate edge cases and error conditions
- Verify system behavior in various environments
- Conduct user experience testing

## Troubleshooting Quick Fixes

### Voice Not Recognized
- Check microphone connection
- Verify audio input levels
- Try speaking more clearly
- Use alternative phrasing

### Robot Not Moving
- Check battery level
- Verify navigation permissions
- Ensure clear path
- Confirm ROS 2 connections

### Object Not Found
- Improve lighting conditions
- Move closer to object
- Check camera calibration
- Verify object is in field of view

### Plan Failing
- Simplify the goal
- Check robot capabilities
- Verify environmental constraints
- Break into smaller tasks