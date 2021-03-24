package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.mission.Waypoint;

import java.beans.Transient;

public class NavigateWaypoint {

    private final Waypoint point;
    private final float distanceThreshold;

    @JsonCreator
    public NavigateWaypoint(@JsonProperty("waypoint") Waypoint point, @JsonProperty("distanceThreshold") float distanceThreshold) {
        this.point = point;
        this.distanceThreshold = distanceThreshold;
    }

    public Waypoint getWaypoint() {
        return point;
    }

    @Transient
    public Point toPoint() {
        return point.toPoint();
    }

    public float getDistanceThreshold() {
        return distanceThreshold;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        NavigateWaypoint that = (NavigateWaypoint) o;

        if (Float.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        return point != null ? point.equals(that.point) : that.point == null;
    }

    @Override
    public int hashCode() {
        int result = point != null ? point.hashCode() : 0;
        result = 31 * result + (distanceThreshold != +0.0f ? Float.floatToIntBits(distanceThreshold) : 0);
        return result;
    }

    public ObjectNode toROSJson(ObjectMapper mapper) {
        final ObjectNode objectNode = mapper.createObjectNode();

        objectNode.put("distanceThreshold", distanceThreshold);
        objectNode.put("longitude", point.getLongitude());
        objectNode.put("latitude", point.getLatitude());
        objectNode.put("relativeAltitude", point.getAltitude());

        return objectNode;
    }
}
