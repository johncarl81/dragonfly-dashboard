package edu.unm.dragonfly.msgs;

/**
 * @author John Ericksen
 */
public class LatLon {
    public double latitude;
    public double longitude;
    public double relativeAltitude;

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public double getRelativeAltitude() {
        return relativeAltitude;
    }

    public void setRelativeAltitude(double relativeAltitude) {
        this.relativeAltitude = relativeAltitude;
    }
}
