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
    private final double altitude;
    private final int stacks;
    private final double distanceThreshold;
    private final double stackHeight;
    private final boolean co2Limit;
    private final double co2Threshold;
    private final double co2LimitMargin;

    @JsonCreator
    public MissionStepCurtain(@JsonProperty("drone") String drone,
                              @JsonProperty("waypoint_start") String waypointStart,
                              @JsonProperty("waypoint_end") String waypointEnd,
                              @JsonProperty("altitude") double altitude,
                              @JsonProperty("stacks") int stacks,
                              @JsonProperty("distance_threshold") double distanceThreshold,
                              @JsonProperty("stack_height") double stackHeight,
                              @JsonProperty("co2_limit") boolean co2Limit,
                              @JsonProperty("co2_threshold")  double co2Threshold,
                              @JsonProperty("co2_limit_margin")  double co2LimitMargin) {
        this.drone = drone;
        this.waypointStart = waypointStart;
        this.waypointEnd = waypointEnd;
        this.altitude = altitude;
        this.stacks = stacks;
        this.distanceThreshold = distanceThreshold;
        this.stackHeight = stackHeight;
        this.co2Limit = co2Limit;
        this.co2Threshold = co2Threshold;
        this.co2LimitMargin = co2LimitMargin;
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

    public double getDistanceThreshold() {
        return distanceThreshold;
    }

    public double getAltitude() {
        return altitude;
    }

    public int getStacks() {
        return stacks;
    }

    public double getStackHeight() {
        return stackHeight;
    }

    public boolean isCo2Limit() {
        return co2Limit;
    }

    public double getCo2Threshold() {
        return co2Threshold;
    }

    public double getCo2LimitMargin() {
        return co2LimitMargin;
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
        ObjectNode data = curtain.putObject("curtain_step");

        data.put("start_waypoint", waypointStart);
        data.put("end_waypoint", waypointEnd);
        data.put("stacks", stacks);
        data.put("altitude", altitude);
        data.put("distance_threshold", distanceThreshold);
        data.put("stack_height", stackHeight);
        data.put("co2_limit", co2Limit);
        data.put("co2_threshold", co2Threshold);
        data.put("co2_limit_margin", co2LimitMargin);

        return curtain;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepCurtain that = (MissionStepCurtain) o;

        if (Double.compare(that.altitude, altitude) != 0) return false;
        if (stacks != that.stacks) return false;
        if (Double.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (Double.compare(that.stackHeight, stackHeight) != 0) return false;
        if (co2Limit != that.co2Limit) return false;
        if (Double.compare(that.co2Threshold, co2Threshold) != 0) return false;
        if (Double.compare(that.co2LimitMargin, co2LimitMargin) != 0) return false;
        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        if (waypointStart != null ? !waypointStart.equals(that.waypointStart) : that.waypointStart != null)
            return false;
        return waypointEnd != null ? waypointEnd.equals(that.waypointEnd) : that.waypointEnd == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (waypointStart != null ? waypointStart.hashCode() : 0);
        result = 31 * result + (waypointEnd != null ? waypointEnd.hashCode() : 0);
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + stacks;
        temp = Double.doubleToLongBits(distanceThreshold);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(stackHeight);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (co2Limit ? 1 : 0);
        temp = Double.doubleToLongBits(co2Threshold);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(co2LimitMargin);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
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
                ", stackHeight=" + stackHeight +
                ", distanceThreshold=" + distanceThreshold +
                ", co2Limit=" + co2Limit +
                ", co2Threshold=" + co2Threshold +
                ", co2LimitMargin=" + co2LimitMargin +
                '}';
    }
}
