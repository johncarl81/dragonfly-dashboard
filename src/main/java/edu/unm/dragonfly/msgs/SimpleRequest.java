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

    @JsonCreator
    public static SimpleRequest create() {
        return new AutoValue_SimpleRequest();
    }

    public static SimpleRequest now() {
        return create();
    }
}
