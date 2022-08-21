package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.BoundaryFixture;
import edu.unm.dragonfly.Fixture;

/**
 * @author John Ericksen
 */
public class MissionStepLawnmower implements MissionStep {

    private final String boundaryName;
    private final String drone;
    private final double stepLength;
    private final boolean walkBoundary;
    private final int walk;
    private final int stacks;
    private final double altitude;
    private final double waitTime;
    private final double distanceThreshold;
    private final boolean co2Limit;
    private final double co2Threshold;
    private final double co2LimitMargin;

    @JsonCreator
    public MissionStepLawnmower(@JsonProperty("boundaryName") String boundaryName,
                                @JsonProperty("drone") String drone,
                                @JsonProperty("step_length") double stepLength,
                                @JsonProperty("altitude") double altitude,
                                @JsonProperty("stacks") int stacks,
                                @JsonProperty("walk_boundary") boolean walkBoundary,
                                @JsonProperty("walk") int walk,
                                @JsonProperty("wait_time") double waitTime,
                                @JsonProperty("distance_threshold") double distanceThreshold,
                                @JsonProperty("co2_limit") boolean co2Limit,
                                @JsonProperty("co2_threshold")  double co2Threshold,
                                @JsonProperty("co2_limit_margin")  double co2LimitMargin) {
        this.boundaryName = boundaryName;
        this.drone = drone;
        this.stepLength = stepLength;
        this.walkBoundary = walkBoundary;
        this.walk = walk;
        this.stacks = stacks;
        this.altitude = altitude;
        this.waitTime = waitTime;
        this.distanceThreshold = distanceThreshold;
        this.co2Limit = co2Limit;
        this.co2Threshold = co2Threshold;
        this.co2LimitMargin = co2LimitMargin;
    }


    public String getBoundaryName() {
        return boundaryName;
    }
    public String getDrone() {
        return drone;
    }

    public double getStepLength() {
        return stepLength;
    }

    public boolean isWalkBoundary() {
        return walkBoundary;
    }

    public int getWalk() {
        return walk;
    }

    public int getStacks() {
        return stacks;
    }

    public double getAltitude() {
        return altitude;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public double getDistanceThreshold() {
        return distanceThreshold;
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
        return fixture instanceof BoundaryFixture && fixture.getName().equals(boundaryName);
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode lawnmower = mapper.createObjectNode();

        lawnmower.put("msg_type", MissionStepType.LAWNMOWER.getMission_type());
        ObjectNode data = lawnmower.putObject("lawnmower_step");

        data.put("boundary", boundaryName);
        data.put("step_length", stepLength);
        data.put("walk_boundary", walkBoundary);
        data.put("walk", walk);
        data.put("stacks", stacks);
        data.put("altitude", altitude);
        data.put("wait_time", waitTime);
        data.put("distance_threshold", distanceThreshold);
        data.put("co2_limit", co2Limit);
        data.put("co2_threshold", co2Threshold);
        data.put("co2_limit_margin", co2LimitMargin);

        return lawnmower;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepLawnmower that = (MissionStepLawnmower) o;

        if (Double.compare(that.stepLength, stepLength) != 0) return false;
        if (walkBoundary != that.walkBoundary) return false;
        if (walk != that.walk) return false;
        if (stacks != that.stacks) return false;
        if (Double.compare(that.altitude, altitude) != 0) return false;
        if (Double.compare(that.waitTime, waitTime) != 0) return false;
        if (Double.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (co2Limit != that.co2Limit) return false;
        if (Double.compare(that.co2Threshold, co2Threshold) != 0) return false;
        if (Double.compare(that.co2LimitMargin, co2LimitMargin) != 0) return false;
        if (boundaryName != null ? !boundaryName.equals(that.boundaryName) : that.boundaryName != null) return false;
        return drone != null ? drone.equals(that.drone) : that.drone == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = boundaryName != null ? boundaryName.hashCode() : 0;
        result = 31 * result + (drone != null ? drone.hashCode() : 0);
        temp = Double.doubleToLongBits(stepLength);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (walkBoundary ? 1 : 0);
        result = 31 * result + walk;
        result = 31 * result + stacks;
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(waitTime);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(distanceThreshold);
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
        return "MissionStepLawnmower{" +
                "boundaryName='" + boundaryName + '\'' +
                ", drone='" + drone + '\'' +
                ", stepLength=" + stepLength +
                ", walkBoundary=" + walkBoundary +
                ", walk=" + walk +
                ", stacks=" + stacks +
                ", altitude=" + altitude +
                ", waitTime=" + waitTime +
                ", distanceThreshold=" + distanceThreshold +
                ", co2Limit=" + co2Limit +
                ", co2Threshold=" + co2Threshold +
                ", co2LimitMargin=" + co2LimitMargin +
                '}';
    }
}
