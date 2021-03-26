package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Point {
    @JsonProperty
    public abstract double x();
    @JsonProperty
    public abstract double y();
    @JsonProperty
    public abstract double z();

    @JsonCreator
    public static Point create(@JsonProperty("x") double x, @JsonProperty("y") double y, @JsonProperty("z") double z) {
        return new AutoValue_Point(x, y, z);
    }
}
