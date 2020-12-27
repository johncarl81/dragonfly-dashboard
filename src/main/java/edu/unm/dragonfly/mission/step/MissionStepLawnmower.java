package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * @author John Ericksen
 */
public class MissionStepLawnmower implements MissionStep {

    private final String drone;
    private final double stepLength;
    private final boolean walkBoundary;
    private final int walk;
    private final int stacks;
    private final double altitude;
    private final double waitTime;
    private final double distanceThreshold;

    @JsonCreator
    public MissionStepLawnmower(@JsonProperty("drone") String drone,
                                @JsonProperty("stepLength") double stepLength,
                                @JsonProperty("altitude") double altitude,
                                @JsonProperty("stacks") int stacks,
                                @JsonProperty("walkBoundary") boolean walkBoundary,
                                @JsonProperty("walk") int walk,
                                @JsonProperty("waitTime") double waitTime,
                                @JsonProperty("distanceThreshold") double distanceThreshold) {
        this.drone = drone;
        this.stepLength = stepLength;
        this.walkBoundary = walkBoundary;
        this.walk = walk;
        this.stacks = stacks;
        this.altitude = altitude;
        this.waitTime = waitTime;
        this.distanceThreshold = distanceThreshold;
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
        return drone != null ? drone.equals(that.drone) : that.drone == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
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
        return result;
    }

    @Override
    public String toString() {
        return "Lawnmower{" +
                "drone='" + drone + '\'' +
                ", steplength=" + stepLength +
                ", walkBoundary=" + walkBoundary +
                ", walk=" + walk +
                ", stacks=" + stacks +
                ", altitude=" + altitude +
                ", waittime=" + waitTime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
