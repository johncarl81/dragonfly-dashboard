package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.*;
import com.google.auto.value.*;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class Plume {
    @JsonProperty
    public abstract LatLon source();
    @JsonProperty("wind_direction")
    public abstract double windDirection();
    @JsonProperty
    public abstract double q();
    @JsonProperty
    public abstract double k();
    @JsonProperty
    public abstract double u();

    @JsonCreator
    public static Plume create(@JsonProperty("source") LatLon source,
                                            @JsonProperty("wind_direction") double windDirection,
                                            @JsonProperty("q") double q,
                                            @JsonProperty("k") double k,
                                            @JsonProperty("u") double u) {
        return new AutoValue_Plume(source, windDirection, q, k, u);
    }
}
