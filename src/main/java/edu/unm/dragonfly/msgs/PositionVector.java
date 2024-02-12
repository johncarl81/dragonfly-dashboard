package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class PositionVector {

    @JsonProperty
    public abstract int movement();
    @JsonProperty
    public abstract LatLon position();
    @JsonProperty
    public abstract double x();
    @JsonProperty
    public abstract double y();
    @JsonProperty
    public abstract double distance();
    @JsonProperty
    public abstract double radius();
    @JsonProperty
    public abstract LatLon center();
    @JsonProperty
    public abstract double a();
    @JsonProperty
    public abstract int p();

    @JsonCreator
    public static PositionVector create(@JsonProperty("movement") int movement,
                                        @JsonProperty("position") LatLon position,
                                         @JsonProperty("x") double x,
    @JsonProperty("y") double y,
    @JsonProperty("distance") double distance,
    @JsonProperty("radius") double radius,
    @JsonProperty("center") LatLon center,
    @JsonProperty("a") double a,
    @JsonProperty("p") int p) {
        return new AutoValue_PositionVector(movement, position, x, y, distance, radius, center, a, p);
    }
}
