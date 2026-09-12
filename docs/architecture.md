# System Architecture

This document gives a simple overview of how the main parts of the warehouse robot work together.

The project is divided into five ROS 2 packages, with the safety controller responsible for making the final motion decision.

## 1. Safety Architecture

The `safety_controller` is the only node that publishes the final `/cmd_vel` command.

Every control cycle, it checks the safety conditions in this order:

```text
E-stop
   ↓
LiDAR health
   ↓
Odometry health
   ↓
Battery communication
   ↓
Critical battery
   ↓
Obstacle distance
   ↓
Speed limiting
   ↓
Normal movement
```

If a critical condition is detected, the controller stops the robot instead of sending a normal movement command.

```mermaid
flowchart TD
    A["Safety Controller"]

    A --> P1{"E-stop active?"}
    P1 -- Yes --> Z1["Stop — E_STOPPED"]
    P1 -- No --> P2{"LiDAR fresh?"}

    P2 -- No --> Z2["Stop — LIDAR_STALE"]
    P2 -- Yes --> P3{"Odometry fresh?"}

    P3 -- No --> Z3["Stop — ODOM_STALE"]
    P3 -- Yes --> P4{"Battery communication fresh?"}

    P4 -- No --> Z4["Stop — BATTERY_STALE"]
    P4 -- Yes --> P5{"Battery critical?"}

    P5 -- Yes --> Z5["Stop — BATTERY_CRITICAL_STOP"]
    P5 -- No --> P6{"Obstacle too close?"}

    P6 -- Stop --> Z6["Stop / turn — OBSTACLE_STOP"]
    P6 -- Slow --> Z7["Reduce speed — SLOW"]
    P6 -- Clear --> Z8["Normal / low-battery speed"]
```

## 2. ROS 2 Topic Flow

The main sensor and control flow is:

```mermaid
flowchart LR
    L["LiDAR"] --> FI["Sensor Fault Injector"]
    I["IMU"] --> FI
    O["Odometry"] --> FI

    FI --> SC["Safety Controller"]
    FI --> LM["Localization Monitor"]
    FI --> SH["Sensor Heartbeat Monitor"]

    B["Battery Manager"] --> SC
    B --> HA["Health Aggregator"]

    E["E-stop Manager"] --> SC
    E --> HA

    SC --> R["Robot"]
    SC --> HA
    LM --> HA
    SH --> HA

    HA --> D["Dashboard Bridge"]
    D --> UI["Web Dashboard"]
```

The fault injector sits between the simulated sensors and the monitoring system so that sensor failures can be tested safely.

## 3. E-Stop

The E-stop uses a simple state machine:

```mermaid
stateDiagram-v2
    [*] --> CLEAR
    CLEAR --> ACTIVE: activate
    ACTIVE --> ACTIVE: activate
    ACTIVE --> RESET_REQUESTED: explicit reset
    RESET_REQUESTED --> ACTIVE: E-stop triggered again
    RESET_REQUESTED --> CLEAR: grace period completed
```

An active E-stop cannot be cleared directly. A reset request and recovery period are required before the robot can move again.

## 4. Robot Health

The health aggregator combines information from the safety and monitoring nodes.

The main health levels are:

```text
HEALTHY
DEGRADED
WARNING
CRITICAL
E_STOP
```

The general priority is:

```text
E_STOP
   ↓
CRITICAL
   ↓
WARNING
   ↓
DEGRADED
   ↓
HEALTHY
```

This allows the dashboard to show the overall condition of the robot instead of displaying each sensor independently.

## 5. Dashboard Architecture

The dashboard is separate from the safety system.

```mermaid
flowchart LR
    HA["Health Aggregator"] --> DB["Dashboard Bridge"]
    SC["Safety Controller"] --> DB

    DB --> API["FastAPI / WebSocket"]
    API --> UI["Web Dashboard"]

    UI --> API
    API --> E["E-stop Service"]
    API --> F["Fault Injection Service"]
    API --> B["Battery Override"]
```

The dashboard does **not** control `/cmd_vel`.

If the dashboard or `dashboard_bridge` stops, the safety controller and other ROS 2 nodes continue to operate normally.

## 6. Multi-Robot Support

The current project is designed for a single robot and uses global topic names such as:

```text
/scan
/odom
/cmd_vel
```

Multi-robot support is planned for future work.

The next step would be to run each robot inside its own ROS 2 namespace, for example:

```text
/robot1/scan
/robot1/odom
/robot1/cmd_vel
```

The dashboard would also need to support multiple robots.

## 7. Package Structure

The project uses five ROS 2 packages:

```text
warehouse_robot_msgs
        ↓
warehouse_robot_control
        ↓
warehouse_robot_bringup

warehouse_robot_description
        ↓
warehouse_robot_bringup

warehouse_robot_dashboard
        ↓
warehouse_robot_bringup
```

Keeping the project in five packages keeps the structure simple while separating messages, control, simulation, dashboard, and launch functionality.
