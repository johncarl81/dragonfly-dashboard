package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class PumpRequest {

    @JsonProperty
    abstract int pump_num();

    @JsonCreator
    public static PumpRequest create(@JsonProperty("pump_num") int pump_num) {
        return new AutoValue_PumpRequest(pump_num);
    }
}
