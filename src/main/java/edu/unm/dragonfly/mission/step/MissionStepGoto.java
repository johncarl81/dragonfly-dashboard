package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Fixture;
import edu.unm.dragonfly.WaypointFixture;

/**
 * @author John Ericksen
 */
public class MissionStepGoto implements MissionStep {

    private final String drone;
    private final String waypoint;

    @JsonCreator
    public MissionStepGoto(@JsonProperty("drone") String drone, @JsonProperty("waypoint") String waypoint) {
        this.drone = drone;
        this.waypoint = waypoint;
    }

    @Override
    public boolean references(Fixture fixture) {
        return fixture instanceof WaypointFixture && fixture.getName().equals(waypoint);
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode gotoWaypoint = mapper.createObjectNode();

        gotoWaypoint.put("msg_type", MissionStepType.GOTO_WAYPOINT.getMission_type());
        ObjectNode data = gotoWaypoint.putObject("goto_step");

        data.put("waypoint", waypoint);

        return gotoWaypoint;
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
