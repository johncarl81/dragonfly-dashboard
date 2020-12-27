package edu.unm.dragonfly.mission.step;

import edu.unm.dragonfly.Walk;

/**
 * @author John Ericksen
 */
public class MissionStepDDSA implements MissionStep{

    private String drone;
    private float radius;
    private float stepLength;
    private float altitude;
    private int loops;
    private int stacks;
    private Walk walk;
    private float waittime;
    private float distanceThreshold;

    public MissionStepDDSA() {
        // Empty Bean Constructor
    }

    public MissionStepDDSA(String drone, float radius, float stepLength, float altitude, int loops, int stacks, Walk walk, float waittime, float distanceThreshold) {
        this.drone = drone;
        this.radius = radius;
        this.stepLength = stepLength;
        this.altitude = altitude;
        this.loops = loops;
        this.stacks = stacks;
        this.walk = walk;
        this.waittime = waittime;
        this.distanceThreshold = distanceThreshold;
    }

    public String getDrone() {
        return drone;
    }

    public float getRadius() {
        return radius;
    }

    public float getStepLength() {
        return stepLength;
    }

    public float getAltitude() {
        return altitude;
    }

    public int getLoops() {
        return loops;
    }

    public int getStacks() {
        return stacks;
    }

    public Walk getWalk() {
        return walk;
    }

    public float getWaittime() {
        return waittime;
    }

    public float getDistanceThreshold() {
        return distanceThreshold;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepDDSA that = (MissionStepDDSA) o;

        if (Float.compare(that.radius, radius) != 0) return false;
        if (Float.compare(that.stepLength, stepLength) != 0) return false;
        if (Float.compare(that.altitude, altitude) != 0) return false;
        if (loops != that.loops) return false;
        if (stacks != that.stacks) return false;
        if (Float.compare(that.waittime, waittime) != 0) return false;
        if (Float.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        return walk == that.walk;
    }

    @Override
    public int hashCode() {
        int result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (radius != +0.0f ? Float.floatToIntBits(radius) : 0);
        result = 31 * result + (stepLength != +0.0f ? Float.floatToIntBits(stepLength) : 0);
        result = 31 * result + (altitude != +0.0f ? Float.floatToIntBits(altitude) : 0);
        result = 31 * result + loops;
        result = 31 * result + stacks;
        result = 31 * result + (walk != null ? walk.hashCode() : 0);
        result = 31 * result + (waittime != +0.0f ? Float.floatToIntBits(waittime) : 0);
        result = 31 * result + (distanceThreshold != +0.0f ? Float.floatToIntBits(distanceThreshold) : 0);
        return result;
    }

    @Override
    public String toString() {
        return "DDSA{" +
                "drone='" + drone + '\'' +
                ", radius=" + radius +
                ", stepLength=" + stepLength +
                ", altitude=" + altitude +
                ", loops=" + loops +
                ", stacks=" + stacks +
                ", walk=" + walk +
                ", waittime=" + waittime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
