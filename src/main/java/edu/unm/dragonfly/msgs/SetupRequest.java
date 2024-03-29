package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class SetupRequest {
    @JsonProperty("rtl_altitude")
    public abstract int rtlAltitude();
    @JsonProperty("max_altitude")
    public abstract int maxAltitude();
    @JsonProperty("rtl_boundary")
    public abstract Boundary rtlBoundary();

    @JsonCreator
    public static SetupRequest create(@JsonProperty("rtl_altitude") int rtlAltitude,
                                      @JsonProperty("max_altitude") int maxAltitude,
                                      @JsonProperty("rtl_boundary") Boundary rtlBoundary) {
        return new AutoValue_SetupRequest(rtlAltitude, maxAltitude, rtlBoundary);
    }
}