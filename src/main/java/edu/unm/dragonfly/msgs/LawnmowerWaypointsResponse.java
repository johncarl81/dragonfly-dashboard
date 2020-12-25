package edu.unm.dragonfly.msgs;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LawnmowerWaypointsResponse {
    public List<LatLon> waypoints;

    public List<LatLon> getWaypoints() {
        return waypoints;
    }
}
