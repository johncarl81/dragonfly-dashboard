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
public class MissionStepCurtain implements MissionStep {

    private final String drone;
    private final String waypointStart;
    private final String waypointEnd;
    private final float altitude;
    private final int stacks;
    private final float distanceThreshold;

    @JsonCreator
    public MissionStepCurtain(@JsonProperty("drone") String drone,
                              @JsonProperty("waypointStart") String waypointStart,
                              @JsonProperty("waypointEnd") String waypointEnd,
                              @JsonProperty("altitude") float altitude,
                              @JsonProperty("stacks") int stacks,
                              @JsonProperty("distanceThreshold") float distanceThreshold) {
        this.drone = drone;
        this.waypointStart = waypointStart;
        this.waypointEnd = waypointEnd;
        this.altitude = altitude;
        this.stacks = stacks;
        this.distanceThreshold = distanceThreshold;
    }

    public String getDrone() {
        return drone;
    }

    public String getWaypointStart() {
        return waypointStart;
    }

    public String getWaypointEnd() {
        return waypointEnd;
    }

    public float getDistanceThreshold() {
        return distanceThreshold;
    }

    public float getAltitude() {
        return altitude;
    }

    public int getStacks() {
        return stacks;
    }

    @Override
    public boolean references(Fixture fixture) {
        return fixture instanceof WaypointFixture && (
                fixture.getName().equals(waypointStart) ||
                        fixture.getName().equals(waypointEnd)) ;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode curtain = mapper.createObjectNode();

        curtain.put("msg_type", MissionStepType.CURTAIN.getMission_type());
        ObjectNode data = curtain.putObject("curtain");

        data.put("start_waypoint", waypointStart);
        data.put("end_waypoint", waypointEnd);
        data.put("stacks", stacks);
        data.put("altitude", altitude);
        data.put("distanceThreshold", distanceThreshold);

        return curtain;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepCurtain that = (MissionStepCurtain) o;

        if (Float.compare(that.altitude, altitude) != 0) return false;
        if (stacks != that.stacks) return false;
        if (Float.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        if (waypointStart != null ? !waypointStart.equals(that.waypointStart) : that.waypointStart != null)
            return false;
        return waypointEnd != null ? waypointEnd.equals(that.waypointEnd) : that.waypointEnd == null;
    }

    @Override
    public int hashCode() {
        int result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypointStart != null ? waypointStart.hashCode() : 0);
        result = 31 * result + (waypointEnd != null ? waypointEnd.hashCode() : 0);
        result = 31 * result + (altitude != +0.0f ? Float.floatToIntBits(altitude) : 0);
        result = 31 * result + stacks;
        result = 31 * result + (distanceThreshold != +0.0f ? Float.floatToIntBits(distanceThreshold) : 0);
        return result;
    }

    @Override
    public String toString() {
        return "Curtain{" +
                "drone='" + drone + '\'' +
                ", waypointStart='" + waypointStart + '\'' +
                ", waypointEnd='" + waypointEnd + '\'' +
                ", altitude=" + altitude +
                ", stacks=" + stacks +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
