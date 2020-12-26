package edu.unm.dragonfly.mission.step;

/**
 * @author John Ericksen
 */
public enum MissionStepType {

    START("Start"),
    TAKEOFF("Takeoff"),
    SLEEP("Sleep"),
    LAND("Land"),
    GOTO_WAYPOINT("Goto Waypoint"),
    SEMAPHORE("Semaphore"),
    RTL("RTL"),
    DDSA("DDSA"),
    LAWNMOWER("Lawnmower");

    private final String name;

    MissionStepType(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public String toString() {
        return name;
    }
}
