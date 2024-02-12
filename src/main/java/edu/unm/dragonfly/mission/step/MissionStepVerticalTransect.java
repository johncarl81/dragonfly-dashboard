package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.*;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.*;
import edu.unm.dragonfly.*;

import java.util.*;

/**
 * @author John Ericksen
 */
public class MissionStepVerticalTransect implements MissionStep {

    private final String drone;
    private final String waypoint;
    private final double minimumAltitude;
    private final double maximumAltitude;

    @JsonCreator
    public MissionStepVerticalTransect(@JsonProperty("drone") String drone,
                           @JsonProperty("waypoint") String waypoint,
                           @JsonProperty("minimum_altitude") double minimumAltitude,
                           @JsonProperty("maximum_altitude") double maximumAltitude) {
        this.drone = drone;
        this.waypoint = waypoint;
        this.minimumAltitude = minimumAltitude;
        this.maximumAltitude = maximumAltitude;
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

        gotoWaypoint.put("msg_type", MissionStepType.VERTICAL_TRANSECT.getMission_type());
        ObjectNode data = gotoWaypoint.putObject("vertical_transect_step");

        data.put("waypoint", waypoint);
        data.put("minimum_altitude", minimumAltitude);
        data.put("maximum_altitude", maximumAltitude);

        return gotoWaypoint;
    }

    public String getDrone() {
        return drone;
    }

    public String getWaypoint() {
        return waypoint;
    }

    public double getMinimumAltitude() {
        return minimumAltitude;
    }

    public double getMaximumAltitude() {
        return maximumAltitude;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepVerticalTransect that = (MissionStepVerticalTransect) o;

        if (Double.compare(minimumAltitude, that.minimumAltitude) != 0) return false;
        if (Double.compare(maximumAltitude, that.maximumAltitude) != 0) return false;
        if (!Objects.equals(drone, that.drone)) return false;
        return Objects.equals(waypoint, that.waypoint);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypoint != null ? waypoint.hashCode() : 0);
        temp = Double.doubleToLongBits(minimumAltitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(maximumAltitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "VerticalTransect{" +
                "drone='" + drone + '\'' +
                ", waypoint='" + waypoint + '\'' +
                ", minimumAltitude=" + minimumAltitude +
                ", maximumAltitude=" + maximumAltitude +
                '}';
    }
}
