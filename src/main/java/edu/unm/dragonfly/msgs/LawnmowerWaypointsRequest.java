package edu.unm.dragonfly.msgs;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LawnmowerWaypointsRequest {
    public List<LatLon> boundary;
    public double stepLength;
    public boolean walkBoundary;
    public int walk;
    public int stacks;
    public double altitude;
    public double waitTime;

    public void setBoundary(List<LatLon> boundary) {
        this.boundary = boundary;
    }

    public void setStepLength(double stepLength) {
        this.stepLength = stepLength;
    }

    public void setWalkBoundary(boolean walkBoundary) {
        this.walkBoundary = walkBoundary;
    }

    public void setWalk(int walk) {
        this.walk = walk;
    }

    public void setStacks(int stacks) {
        this.stacks = stacks;
    }

    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }

    public void setWaitTime(double waitTime) {
        this.waitTime = waitTime;
    }
}
