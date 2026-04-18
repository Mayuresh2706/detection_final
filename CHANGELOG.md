# Changelog

All notable changes to this project are documented in this file using Conventional Commits and Semantic Versioning.

## [Unreleased] - 0.0.0

### Summary
Active development on `experimental_docking` branch with focus on docking mechanisms, navigation, and task execution.

### Commits by Category

| Commit | Type | Date | Message |
|--------|------|------|---------|
| f651fa4 | feat | 2026-04-16 | new |
| 0a7acf8 | fix | 2026-04-16 | fix |
| 9919f1f | fix | 2026-04-16 | fixed docking approach |
| b850864 | fix | 2026-04-16 | docking fiy |
| 58cb9b0 | feat | 2026-04-16 | changed odom drive |
| f76ae85 | fix | 2026-04-16 | no nav fix 2 |
| 81c865c | fix | 2026-04-16 | no nav fix |
| 624c8cb | feat | 2026-04-16 | try new code |
| b2def3d | feat | 2026-04-16 | changed aruco marker size and balls fired for task b |
| 1fbc56d | fix | 2026-04-16 | speed stall issue |
| b29aadb | feat | 2026-04-16 | normal line in place alignment |
| b3f830d | refactor | 2026-04-16 | changed to head straight towards normal |
| d49afcc | docs | 2026-04-16 | added more logs for debugging |
| e93c3c0 | feat | 2026-04-16 | direct approach docking |
| c16b107 | fix | 2026-04-15 | docking patch |
| f45dbfd | feat | 2026-04-15 | docking 2.0 - experimental |
| eda16e8 | feat | 2026-04-15 | added new bearing angle calculator & new docking code |
| 114b056 | feat | 2026-04-15 | a very experimental code |
| 62bc568 | fix | 2026-04-15 | maybe fix |
| 52c4b59 | feat | 2026-04-15 | no nav |
| 511b5ab | docs | 2026-04-15 | updated launch files |
| ff65539 | init | 2026-04-15 | initial commit |
| 150626e | feat | 2026-04-15 | experimental docking logic #2 |
| be0ae1e | refactor | 2026-04-15 | task b simplified |
| 4ebd047 | feat | 2026-04-15 | experimental docking logic |
| 48f4336 | docs | 2026-04-15 | updated launch file |
| affbf64 | fix | 2026-04-15 | ctrl c interrupt |
| df900b7 | docs | 2026-04-15 | all in one launch file |
| 6aebfe2 | docs | 2026-04-15 | added new launch files |
| aca06b8 | fix | 2026-04-15 | integration + task b docking fix |

### Recent Changes (Branch: experimental_docking)

#### Features & Enhancements
- Docking mechanism improvements (bearing angle calculator, direct approach, 2.0 experimental)
- ArUco marker size adjustments and ball firing for Task B
- Navigation without map (`no_nav` implementation)
- Odometry drive changes
- Line-based and normal-directed alignment algorithms

#### Fixes & Bug Resolution
- Docking approach corrections
- Speed stall issue resolution
- Navigation integration fixes
- Servo interrupt handling
- Marker offset and alignment accuracy improvements

#### Documentation & Configuration
- Updated launch files (laptop, RPi, all-in-one configurations)
- Enhanced debugging logs
- Refactored heading and alignment logic

### Entry Points
The project defines the following executable nodes:
- `camera_node` - ArUco detection via camera
- `pnp_node` - PnP estimation node
- `docking_node` - Main docking controller
- `task_a_node` - Task A execution
- `task_b_node` - Task B execution  
- `mission_manager` - Main mission orchestrator
- `docking_no_nav` - Docking without navigation stack
- `servo_node` - Servo control interface
