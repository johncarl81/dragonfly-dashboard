package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class PumpResponse {

    @JsonProperty
    abstract boolean done();

    @JsonCreator
    public static PumpResponse create(@JsonProperty("done") boolean done) {
        return new AutoValue_PumpResponse(done);
    }
}

