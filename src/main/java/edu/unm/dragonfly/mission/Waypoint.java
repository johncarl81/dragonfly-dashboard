package edu.unm.dragonfly.mission;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

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

    public double getLongitude() {
        return longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getAltitude() {
        return altitude;
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
