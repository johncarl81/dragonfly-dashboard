package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Pose {
    @JsonProperty
    public abstract Point position();
    @JsonProperty
    public abstract Quaternion orientation();

    @JsonCreator
    public static Pose create(@JsonProperty("position") Point position, @JsonProperty("orientation") Quaternion orientation) {
        return new AutoValue_Pose(position, orientation);
    }
}
