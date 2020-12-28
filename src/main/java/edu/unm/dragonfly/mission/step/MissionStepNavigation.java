package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.mission.Waypoint;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepNavigation implements MissionStep {

    private final String drone;
    private final List<Waypoint> waypoints;
    private final double waitTime;
    private final double distanceThreshold;

    @JsonCreator
    public MissionStepNavigation(@JsonProperty("drone") String drone,
                                 @JsonProperty("waypoints") List<Waypoint> waypoints,
                                 @JsonProperty("waitTime") double waitTime,
                                 @JsonProperty("distanceThreshold") double distanceThreshold) {
        this.drone = drone;
        this.waypoints = waypoints;
        this.waitTime = waitTime;
        this.distanceThreshold = distanceThreshold;
    }

    public String getDrone() {
        return drone;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public double getDistanceThreshold() {
        return distanceThreshold;
    }

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper) {
        ObjectNode lawnmower = mapper.createObjectNode();

        lawnmower.put("msg_type", MissionStepType.NAVIGATION.getMission_type());
        ObjectNode data = lawnmower.putObject("navigation");

        ArrayNode waypointsArray = data.putArray("waypoints");

        for(Waypoint wp : waypoints) {
            ObjectNode jsonNode = waypointsArray.addObject();
            jsonNode.put("longitude", wp.getLongitude());
            jsonNode.put("latitude", wp.getLatitude());
            jsonNode.put("relativeAltitude", wp.getAltitude());
        }

        data.put("waitTime", waitTime);
        data.put("distanceThreshold", distanceThreshold);

        return lawnmower;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepNavigation that = (MissionStepNavigation) o;

        if (Double.compare(that.waitTime, waitTime) != 0) return false;
        if (Double.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        return waypoints != null ? waypoints.equals(that.waypoints) : that.waypoints == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypoints != null ? waypoints.hashCode() : 0);
        temp = Double.doubleToLongBits(waitTime);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(distanceThreshold);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "MissionStepNavigation{" +
                "drone='" + drone + '\'' +
                ", waypoints=" + waypoints +
                ", waitTime=" + waitTime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
