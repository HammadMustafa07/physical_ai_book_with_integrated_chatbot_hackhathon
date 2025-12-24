# Data Model: Vision-Language-Action (VLA) Integration

## Core Entities

### Voice Command
- **Fields**:
  - id: string (unique identifier)
  - text: string (transcribed speech content)
  - timestamp: datetime (when command was received)
  - confidence: float (speech recognition confidence level)
  - intent: string (structured robot intent derived from command)
  - userId: string (optional, for multi-user scenarios)
  - status: enum (pending, processed, failed)

- **Relationships**:
  - One-to-many with Action Plan (a voice command can trigger multiple action sequences)
  - Belongs to User (optional, for tracking command history)

### Action Plan
- **Fields**:
  - id: string (unique identifier)
  - title: string (human-readable description)
  - description: string (detailed explanation of the plan)
  - steps: array (sequence of actions to execute)
  - status: enum (pending, in-progress, completed, failed)
  - createdAt: datetime
  - updatedAt: datetime
  - priority: enum (low, medium, high)
  - estimatedDuration: integer (in seconds)

- **Relationships**:
  - Belongs to Voice Command (originated from a specific command)
  - One-to-many with Robot Action (contains multiple individual actions)
  - Belongs to User (optional)

### Robot Action
- **Fields**:
  - id: string (unique identifier)
  - type: enum (navigation, perception, manipulation, communication)
  - parameters: object (action-specific parameters)
  - sequenceNumber: integer (order in the action plan)
  - status: enum (pending, executing, completed, failed)
  - startTime: datetime (when action started)
  - endTime: datetime (when action completed)
  - result: object (outcome of the action)

- **Relationships**:
  - Belongs to Action Plan (part of a specific plan)
  - Belongs to Robot (executed by a specific robot instance)

### VLA Pipeline
- **Fields**:
  - id: string (unique identifier)
  - name: string (pipeline name)
  - status: enum (idle, processing, active, error)
  - currentStage: enum (voice, planning, navigation, perception, manipulation)
  - lastActivity: datetime
  - configuration: object (pipeline settings and parameters)

- **Relationships**:
  - One-to-many with Voice Command (processes multiple commands)
  - One-to-many with Action Plan (generates multiple plans)

### Robot Intent
- **Fields**:
  - id: string (unique identifier)
  - name: string (intent name)
  - description: string (what the intent represents)
  - parameters: object (required parameters for the intent)
  - requiredCapabilities: array (robot capabilities needed)
  - fallbackIntent: string (optional, alternative intent if primary fails)

- **Relationships**:
  - One-to-many with Voice Command (multiple commands can have the same intent)

## State Transitions

### Action Plan States
- `pending` → `in-progress` (when execution begins)
- `in-progress` → `completed` (when all steps succeed)
- `in-progress` → `failed` (when a step fails)
- `in-progress` → `paused` (when manual intervention needed)

### Robot Action States
- `pending` → `executing` (when action starts)
- `executing` → `completed` (when action succeeds)
- `executing` → `failed` (when action fails)
- `executing` → `interrupted` (when interrupted by higher priority action)

## Validation Rules

1. **Voice Command**:
   - Text must not be empty
   - Confidence must be between 0.0 and 1.0
   - Intent must match a known Robot Intent

2. **Action Plan**:
   - Must have at least one step
   - Steps must have sequential sequence numbers
   - Estimated duration must be positive

3. **Robot Action**:
   - Type must be one of the defined enums
   - Sequence number must be unique within the plan
   - Parameters must match the action type requirements

4. **VLA Pipeline**:
   - Current stage must be consistent with pipeline state
   - Configuration must be valid for the pipeline type

## Relationships Summary

- Voice Command → (1 to many) → Action Plan
- Action Plan → (1 to many) → Robot Action
- VLA Pipeline → (1 to many) → Voice Command
- VLA Pipeline → (1 to many) → Action Plan
- Robot Intent → (1 to many) → Voice Command