package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Quaternion {
    @JsonProperty
    public abstract double x();
    @JsonProperty
    public abstract double y();
    @JsonProperty
    public abstract double z();
    @JsonProperty
    public abstract double w();

    @JsonCreator
    public static Quaternion create(@JsonProperty("x") double x, @JsonProperty("y") double y, @JsonProperty("z") double z, @JsonProperty("w") double w) {
        return new AutoValue_Quaternion(x, y, z, w);
    }
}
