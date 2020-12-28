package edu.unm.dragonfly.msgs;

/**
 * @author John Ericksen
 */
public class DDSAWaypointsRequest {
    public double radius;
    public double stepLength;
    public int walk;
    public int stacks;
    public int loops;
    public double altitude;
    public double waitTime;

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double getStepLength() {
        return stepLength;
    }

    public void setStepLength(double stepLength) {
        this.stepLength = stepLength;
    }

    public int getWalk() {
        return walk;
    }

    public void setWalk(int walk) {
        this.walk = walk;
    }

    public int getStacks() {
        return stacks;
    }

    public void setStacks(int stacks) {
        this.stacks = stacks;
    }

    public int getLoops() {
        return loops;
    }

    public void setLoops(int loops) {
        this.loops = loops;
    }

    public double getAltitude() {
        return altitude;
    }

    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public void setWaitTime(double waitTime) {
        this.waitTime = waitTime;
    }
}