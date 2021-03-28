package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

import java.util.List;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Boundary {
    @JsonProperty
    public abstract String name();
    @JsonProperty
    public abstract List<LatLon> points();

    @JsonCreator
    public static Boundary create(@JsonProperty("name") String name, @JsonProperty("points") List<LatLon> points) {
        return new AutoValue_Boundary(name, points);
    }
}
