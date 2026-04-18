
Note that although this branch is named experimental_docking, it is the main branch for our detection and shooting code



# Changelog

All notable changes to this project are documented in this file using Conventional Commits and Semantic Versioning.

### Summary
Active development on `experimental_docking` branch with focus on docking mechanisms, navigation, and task execution. Total of 61 commits from 2 contributors (yammmyu, Mayuresh2706).

### All Commits (experimental_docking branch)

| Commit | Type | Version Impact | Date | Author | Message |
|--------|------|-----------------|------|--------|---------|
| 10d6007 | init | 0.1.0 | 2026-04-08 | Mayuresh2706 | Initial commit |
| efe5475 | feat | 0.2.0 | 2026-04-09 | yammmyu | Updated Task_A_Main with firing code |
| a75dbb1 | feat | 0.3.0 | 2026-04-09 | yammmyu | Add MG996R class for servo control |
| 5813f14 | feat | 0.4.0 | 2026-04-09 | yammmyu | Add servo control class for MG996R motor |
| b27219f | refactor | 0.4.0 | 2026-04-09 | Mayuresh2706 | Remove obsolete source files and resolve merge artifacts |
| a6a1cf0 | feat | 0.5.0 | 2026-04-09 | Mayuresh2706 | Docking no task A |
| 0b576b8 | feat | 0.6.0 | 2026-04-11 | yammmyu | working ball launching code |
| 8e5a9a0 | feat | 0.7.0 | 2026-04-11 | Mayuresh2706 | shooting_code |
| 3049377 | test | 0.7.0 | 2026-04-11 | yammmyu | added test.py for testing firing |
| a1c9ee7 | refactor | 0.7.0 | 2026-04-11 | Mayuresh2706 | Removed Duplicates |
| 3254bac | merge | 0.7.0 | 2026-04-11 | Mayuresh2706 | Merge branch 'master' of github.com:Mayuresh2706/detection_final |
| 736383c | refactor | 0.7.0 | 2026-04-11 | Mayuresh2706 | removed duplicates |
| 95824fb | feat | 0.8.0 | 2026-04-12 | yammmyu | claude's magic |
| 2c013ee | feat | 0.9.0 | 2026-04-12 | yammmyu | claude's magic adding no_nav |
| c892792 | fix | 0.9.1 | 2026-04-12 | yammmyu | package import fixed |
| 4e2561f | feat | 0.10.0 | 2026-04-12 | yammmyu | added loging for distance it is away from marker |
| 3b3d99f | refactor | 0.10.0 | 2026-04-12 | yammmyu | removed camera offset |
| 60b9e13 | fix | 0.10.1 | 2026-04-12 | yammmyu | small terminal log issue fixed |
| 4020596 | feat | 0.11.0 | 2026-04-12 | yammmyu | new two step alignment |
| 86b9d6c | fix | 0.11.1 | 2026-04-12 | Mayuresh2706 | Not working yet |
| 2b39b9d | feat | 0.12.0 | 2026-04-12 | Mayuresh2706 | added camera calbration file |
| 7ec36f2 | feat | 0.13.0 | 2026-04-12 | yammmyu | updated detection code with new calibration |
| 5d69cc6 | feat | 0.14.0 | 2026-04-13 | Mayuresh2706 | param changes |
| 8129ba6 | feat | 0.15.0 | 2026-04-13 | Mayuresh2706 | calibration and marker size changes |
| d3f2adb | refactor | 0.15.0 | 2026-04-13 | Mayuresh2706 | Alignment changes |
| 379486e | feat | 0.16.0 | 2026-04-13 | Mayuresh2706 | updated docking |
| 0c22647 | feat | 0.17.0 | 2026-04-13 | yammmyu | new head |
| 3b1ae65 | fix | 0.17.1 | 2026-04-13 | yammmyu | fixed only rotating but not moving |
| 089f11e | fix | 0.17.2 | 2026-01-16 | Mayuresh2706 | servo spin direction |
| 99ae62a | refactor | 0.17.2 | 2026-04-13 | yammmyu | changed servo.fire call method |
| 4103b8b | fix | 0.17.3 | 2026-04-13 | yammmyu | servo fixed? |
| c66e90d | docs | 0.17.3 | 2026-01-16 | Mayuresh2706 | working launch |
| d962d0d | feat | 0.18.0 | 2026-04-14 | Mayuresh2706 | Final docking - not tested |
| 2424b02 | fix | 0.18.1 | 2026-04-14 | Mayuresh2706 | docking not working |
| 150626e | feat | 0.19.0 | 2026-04-15 | yammmyu | experimental docking logic #2 |
| be0ae1e | refactor | 0.19.0 | 2026-04-15 | yammmyu | task b simplified |
| 4ebd047 | feat | 0.20.0 | 2026-04-15 | yammmyu | experimental docking logic |
| 48f4336 | docs | 0.20.0 | 2026-04-15 | yammmyu | updated launch file |
| affbf64 | fix | 0.20.1 | 2026-04-15 | yammmyu | ctrl c interrupt |
| df900b7 | docs | 0.20.1 | 2026-04-15 | yammmyu | all in one launch file |
| 6aebfe2 | docs | 0.20.1 | 2026-04-15 | yammmyu | added new launch files |
| aca06b8 | fix | 0.20.2 | 2026-04-15 | yammmyu | integration + task b docking fix |
| ff65539 | init | 1.0.0 | 2026-04-15 | yammmyu | initial commit |
| f45dbfd | feat | 2.0.0 | 2026-04-15 | yammmyu | docking 2.0 - experimental |
| eda16e8 | feat | 2.1.0 | 2026-04-15 | yammmyu | added new bearing angle calculator & new docking code |
| 114b056 | feat | 2.2.0 | 2026-04-15 | yammmyu | a very experimental code |
| 62bc568 | fix | 2.2.1 | 2026-04-15 | yammmyu | maybe fix |
| 52c4b59 | feat | 2.3.0 | 2026-04-15 | yammmyu | no nav |
| 511b5ab | docs | 2.3.0 | 2026-04-15 | yammmyu | updated launch files |
| c16b107 | fix | 2.3.1 | 2026-04-15 | yammmyu | docking patch |
| f651fa4 | feat | 2.4.0 | 2026-04-16 | yammmyu | new |
| 0a7acf8 | fix | 2.4.1 | 2026-04-16 | yammmyu | fix |
| 9919f1f | fix | 2.4.2 | 2026-04-16 | yammmyu | fixed docking approach |
| b850864 | fix | 2.4.3 | 2026-04-16 | yammmyu | docking fix |
| 58cb9b0 | feat | 2.5.0 | 2026-04-16 | yammmyu | changed odom drive |
| f76ae85 | fix | 2.5.1 | 2026-04-16 | yammmyu | no nav fix 2 |
| 81c865c | fix | 2.5.2 | 2026-04-16 | yammmyu | no nav fix |
| 624c8cb | feat | 2.6.0 | 2026-04-16 | yammmyu | try new code |
| b2def3d | feat | 2.7.0 | 2026-04-16 | yammmyu | changed aruco marker size and balls fired for task b |
| 1fbc56d | fix | 2.7.1 | 2026-04-16 | yammmyu | speed stall issue |
| b29aadb | feat | 2.8.0 | 2026-04-16 | yammmyu | normal line in place alignment |
| b3f830d | refactor | 2.8.0 | 2026-04-16 | yammmyu | changed to head straight towards normal |
| d49afcc | docs | 2.8.0 | 2026-04-16 | yammmyu | added more logs for debugging |
| e93c3c0 | feat | 2.9.0 | 2026-04-16 | yammmyu | direct approach docking |
### Recent Changes (Branch: experimental_docking)

#### Features & Enhancements
- Docking mechanism improvements (bearing angle calculator, direct approach, 2.0 experimental)
- ArUco marker size adjustments and ball firing for Task B
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
