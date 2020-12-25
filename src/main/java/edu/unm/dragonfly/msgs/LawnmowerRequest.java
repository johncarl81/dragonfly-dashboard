package edu.unm.dragonfly.msgs;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LawnmowerRequest {
    public List<LatLon> boundary;
    public double steplength;
    public boolean walkBoundary;
    public int walk;
    public int stacks;
    public double altitude;
    public double waittime;
    public double distanceThreshold;

    public void setBoundary(List<LatLon> boundary) {
        this.boundary = boundary;
    }

    public void setSteplength(double steplength) {
        this.steplength = steplength;
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

    public void setWaittime(double waittime) {
        this.waittime = waittime;
    }

    public void setDistanceThreshold(double distanceThreshold) {
        this.distanceThreshold = distanceThreshold;
    }
}
