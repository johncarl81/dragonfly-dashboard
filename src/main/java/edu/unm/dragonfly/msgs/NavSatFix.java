package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.google.auto.value.AutoValue;
import ros.msgs.std_msgs.Header;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class NavSatFix {
    @JsonProperty
    public abstract Header header();
    @JsonProperty
    public abstract NavSatStatus status();
    @JsonProperty
    public abstract double latitude();
    @JsonProperty
    public abstract double longitude();
    @JsonProperty
    public abstract double altitude();
    @SuppressWarnings("mutable")
    @JsonProperty
    public abstract double[] position_covariance();
    @JsonProperty
    public abstract int position_covariance_type();

    @JsonCreator
    public static NavSatFix create(@JsonProperty("header") Header header,
                                   @JsonProperty("status") NavSatStatus status,
                                   @JsonProperty("latitude") double latitude,
                                   @JsonProperty("longitude") double longitude,
                                   @JsonProperty("altitude") double altitude,
                                   @JsonProperty("position_covariance") double[] position_covariance,
                                   @JsonProperty("position_covariance_type") int position_covariance_type) {
        return new AutoValue_NavSatFix(header, status, latitude, longitude, altitude, position_covariance, position_covariance_type);
    }
}
