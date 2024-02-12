package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.*;
import com.google.auto.value.*;

import java.util.*;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class SetupPlumesRequest {
    @JsonProperty("plumes")
    public abstract List<Plume> plumes();

    @JsonCreator
    public static SetupPlumesRequest create(@JsonProperty("plumes") List<Plume> plumes) {
        return new AutoValue_SetupPlumesRequest(plumes);
    }
}