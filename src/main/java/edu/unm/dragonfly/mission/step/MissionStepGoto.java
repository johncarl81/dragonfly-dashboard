package edu.unm.dragonfly.mission.step;

/**
 * @author John Ericksen
 */
public class MissionStepGoto implements MissionStep {

    private final String drone;
    private final String waypoint;

    public MissionStepGoto(String drone, String waypoint) {
        this.drone = drone;
        this.waypoint = waypoint;
    }

    @Override
    public String toString() {
        return "Goto{" +
                "drone='" + drone + '\'' +
                ", waypoint='" + waypoint + '\'' +
                '}';
    }
}
