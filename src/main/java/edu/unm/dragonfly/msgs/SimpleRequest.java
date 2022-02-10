package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;
import ros.msgs.std_msgs.Time;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class SimpleRequest {
    @JsonProperty("command_time")
    public abstract Time commandTime();

    @JsonCreator
    public static SimpleRequest create(@JsonProperty("command_time") Time startTime) {
        return new AutoValue_SimpleRequest(startTime);
    }

    public static SimpleRequest now() {
        return create(Time.now());
    }
}
