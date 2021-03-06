package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class NavSatStatus {
    @JsonProperty
    public abstract int status();
    @JsonProperty
    public abstract int service();

    @JsonCreator
    public static NavSatStatus create(@JsonProperty("status") int status, @JsonProperty("service") int service) {
        return new AutoValue_NavSatStatus(status, service);
    }
}
