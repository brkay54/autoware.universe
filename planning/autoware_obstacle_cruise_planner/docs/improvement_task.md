Improve the obstacle_cruise_planner to make it more reactive for the predicted objects

## Description

In the current implementation, obstacle_cruise_planner is limited to react to the predicted objects such cases:

- If the predicted object is cutting in to the ego's trajectory, planner responds late because the predicted object is
  filtered. (because its lateral distance is higher that some threshold)