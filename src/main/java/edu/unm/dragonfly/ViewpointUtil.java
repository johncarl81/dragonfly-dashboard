package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Envelope;
import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.geometry.SpatialReferences;
import com.esri.arcgisruntime.mapping.Viewpoint;
import edu.unm.dragonfly.mission.Waypoint;

import java.util.List;

/**
 * @author John Ericksen
 */
public class ViewpointUtil {

    public static Viewpoint create(Waypoint waypoint) {
        return create(waypoint.getLatitude(), waypoint.getLongitude());
    }

    public static Viewpoint create(double latitude, double longitude) {
        return new Viewpoint(new Point(longitude, latitude, SpatialReferences.getWgs84()), 1000);
    }

    public static Viewpoint create(Drone.LatLonRelativeAltitude position) {
        return create(position.getLatitude(), position.getLongitude());
    }

    public static Viewpoint create(List<Waypoint> boundary) {
        double maxlat = Double.NEGATIVE_INFINITY;
        double minlat = Double.POSITIVE_INFINITY;
        double maxlon = Double.NEGATIVE_INFINITY;
        double minlon = Double.POSITIVE_INFINITY;


        for (Waypoint wapoint : boundary) {
            maxlon = Math.max(maxlon, wapoint.getLongitude());
            minlon = Math.min(minlon, wapoint.getLongitude());
            maxlat = Math.max(maxlat, wapoint.getLatitude());
            minlat = Math.min(minlat, wapoint.getLatitude());
        }

        return new Viewpoint(new Envelope(maxlon, maxlat, minlon, minlat, SpatialReferences.getWgs84()));
    }
}
