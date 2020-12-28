package edu.unm.dragonfly.msgs;

import java.util.List;

/**
 * @author John Ericksen
 */
public class NavigationRequest {
    public List<LatLon> waypoints;
    public double waitTime;
    public double distanceThreshold;

    public List<LatLon> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(List<LatLon> waypoints) {
        this.waypoints = waypoints;
    }

    public double getWaitTime() {
        return waitTime;
    }

    public void setWaitTime(double waitTime) {
        this.waitTime = waitTime;
    }

    public double getDistanceThreshold() {
        return distanceThreshold;
    }

    public void setDistanceThreshold(double distanceThreshold) {
        this.distanceThreshold = distanceThreshold;
    }
}