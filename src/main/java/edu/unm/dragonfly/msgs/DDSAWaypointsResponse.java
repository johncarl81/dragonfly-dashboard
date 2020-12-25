package edu.unm.dragonfly.msgs;

import java.util.List;

/**
 * @author John Ericksen
 */
public class DDSAWaypointsResponse {
    public List<LatLon> waypoints;

    public List<LatLon> getWaypoints() {
        return waypoints;
    }
}
