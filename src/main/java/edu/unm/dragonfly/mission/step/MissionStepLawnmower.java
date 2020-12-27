package edu.unm.dragonfly.mission.step;

import edu.unm.dragonfly.mission.step.MissionStep;

/**
 * @author John Ericksen
 */
public class MissionStepLawnmower implements MissionStep {

    private String drone;
    private double steplength;
    private boolean walkBoundary;
    private int walk;
    private int stacks;
    private double altitude;
    private double waittime;
    private double distanceThreshold;

    public MissionStepLawnmower() {
        // Empty bean constructor
    }

    public MissionStepLawnmower(String drone, double steplength, double altitude, int stacks, boolean walkBoundary, int walk, double waittime, double distanceThreshold) {
        this.drone = drone;
        this.steplength = steplength;
        this.walkBoundary = walkBoundary;
        this.walk = walk;
        this.stacks = stacks;
        this.altitude = altitude;
        this.waittime = waittime;
        this.distanceThreshold = distanceThreshold;
    }

    public String getDrone() {
        return drone;
    }

    public double getSteplength() {
        return steplength;
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

    public double getWaittime() {
        return waittime;
    }

    public double getDistanceThreshold() {
        return distanceThreshold;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepLawnmower that = (MissionStepLawnmower) o;

        if (Double.compare(that.steplength, steplength) != 0) return false;
        if (walkBoundary != that.walkBoundary) return false;
        if (walk != that.walk) return false;
        if (stacks != that.stacks) return false;
        if (Double.compare(that.altitude, altitude) != 0) return false;
        if (Double.compare(that.waittime, waittime) != 0) return false;
        if (Double.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        return drone != null ? drone.equals(that.drone) : that.drone == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        temp = Double.doubleToLongBits(steplength);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (walkBoundary ? 1 : 0);
        result = 31 * result + walk;
        result = 31 * result + stacks;
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(waittime);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(distanceThreshold);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Lawnmower{" +
                "drone='" + drone + '\'' +
                ", steplength=" + steplength +
                ", walkBoundary=" + walkBoundary +
                ", walk=" + walk +
                ", stacks=" + stacks +
                ", altitude=" + altitude +
                ", waittime=" + waittime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
