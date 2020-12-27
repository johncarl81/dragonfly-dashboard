package edu.unm.dragonfly.mission.step;

/**
 * @author John Ericksen
 */
public class MissionStepGoto implements MissionStep {

    private String drone;
    private String waypoint;

    public MissionStepGoto() {
        // Empty Bean Constructor
    }

    public MissionStepGoto(String drone, String waypoint) {
        this.drone = drone;
        this.waypoint = waypoint;
    }

    public String getDrone() {
        return drone;
    }

    public String getWaypoint() {
        return waypoint;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepGoto that = (MissionStepGoto) o;

        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        return waypoint != null ? waypoint.equals(that.waypoint) : that.waypoint == null;
    }

    @Override
    public int hashCode() {
        int result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypoint != null ? waypoint.hashCode() : 0);
        return result;
    }

    @Override
    public String toString() {
        return "Goto{" +
                "drone='" + drone + '\'' +
                ", waypoint='" + waypoint + '\'' +
                '}';
    }
}
