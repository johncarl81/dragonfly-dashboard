package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonPOJOBuilder;
import com.google.auto.value.AutoValue;
import ros.msgs.std_msgs.Time;

import java.util.List;

/**
 * @author John Ericksen
 */
@AutoValue
@JsonDeserialize(builder = AutoValue_NavigationRequest.Builder.class)
public abstract class NavigationRequest {
    @JsonProperty("command_time")
    public abstract Time commandTime();
    @JsonProperty
    public abstract List<LatLon> waypoints();
    @JsonProperty
    public abstract double waitTime();
    @JsonProperty
    public abstract double distanceThreshold();

    public static Builder builder() {
        return new AutoValue_NavigationRequest.Builder();
    }

    @AutoValue.Builder
    @JsonPOJOBuilder(withPrefix = "")
    public abstract static class Builder {
        @JsonProperty("command_time")
        public abstract Builder commandTime(Time time);
        public abstract Builder waypoints(List<LatLon> waypoints);
        public abstract Builder waitTime(double waitTime);
        public abstract Builder distanceThreshold(double distanceThreshold);
        public abstract NavigationRequest build();
    }
}