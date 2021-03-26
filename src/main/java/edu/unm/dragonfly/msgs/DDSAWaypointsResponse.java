package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

import java.util.List;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class DDSAWaypointsResponse {
    @JsonProperty
    public abstract List<LatLon> waypoints();

    @JsonCreator
    public static DDSAWaypointsResponse create(@JsonProperty("waypoints") List<LatLon> waypoints) {
        return new AutoValue_DDSAWaypointsResponse(waypoints);
    }
}
