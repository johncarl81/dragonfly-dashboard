package edu.unm.dragonfly.mission.step;

/**
 * @author John Ericksen
 */
public enum MissionStepType {

    START(0, "Start"),
    TAKEOFF(1, "Takeoff"),
    SLEEP(2, "Sleep"),
    LAND(3, "Land"),
    GOTO_WAYPOINT(4, "Goto Waypoint"),
    SEMAPHORE(5, "Semaphore"),
    RTL(6, "RTL"),
    DDSA(7, "DDSA"),
    LAWNMOWER(8, "Lawnmower"),
    NAVIGATION(9, "Navigation"),
    FLOCK(10, "Flock"),
    GRADIENT(11, "Gradient");

    private final int mission_type;
    private final String name;

    MissionStepType(int mission_type, String name) {
        this.mission_type = mission_type;
        this.name = name;
    }

    public int getMission_type() {
        return mission_type;
    }

    public String getName() {
        return name;
    }

    public String toString() {
        return name;
    }
}
