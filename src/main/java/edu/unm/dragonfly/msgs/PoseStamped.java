package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;
import ros.msgs.std_msgs.Header;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class PoseStamped {
    @JsonProperty
    public abstract Header header();
    @JsonProperty
    public abstract Pose pose();

    @JsonCreator
    public static PoseStamped create(@JsonProperty("header") Header header,@JsonProperty("pose") Pose pose) {
        return new AutoValue_PoseStamped(header, pose);
    }
}
