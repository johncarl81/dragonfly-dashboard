package edu.unm.dragonfly.mission;

import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.geometry.SpatialReferences;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.unm.dragonfly.msgs.LatLon;

import java.beans.Transient;

/**
 * @author John Ericksen
 */
public class Waypoint {

    private final double longitude;
    private final double latitude;
    private final double altitude;

    @JsonCreator
    public Waypoint(@JsonProperty("longitude") double longitude, @JsonProperty("latitude") double latitude, @JsonProperty("altitude") double altitude) {
        this.longitude = longitude;
        this.latitude = latitude;
        this.altitude = altitude;
    }

    public static Waypoint from(Point point) {
        return new Waypoint(point.getX(), point.getY(), point.getZ());
    }

    public double getLongitude() {
        return longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getAltitude() {
        return altitude;
    }

    @Transient
    public Point toPoint() {
        return new Point(longitude, latitude, altitude, SpatialReferences.getWgs84());
    }

    @Transient
    public LatLon toLatLon() {
        return LatLon.builder().latitude(latitude).longitude(longitude).relativeAltitude(altitude).build();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Waypoint waypoint = (Waypoint) o;

        if (Double.compare(waypoint.longitude, longitude) != 0) return false;
        if (Double.compare(waypoint.latitude, latitude) != 0) return false;
        return Double.compare(waypoint.altitude, altitude) == 0;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(longitude);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
