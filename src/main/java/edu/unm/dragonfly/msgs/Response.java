package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Response {
    @JsonProperty
    public abstract int success();
    @JsonProperty
    public abstract String message();

    @JsonCreator
    public static Response create(@JsonProperty("success") int success, @JsonProperty("message") String message) {
        return new AutoValue_Response(success, message);
    }
}
