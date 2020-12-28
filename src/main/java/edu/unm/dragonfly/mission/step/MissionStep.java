package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import static com.fasterxml.jackson.annotation.JsonTypeInfo.As.PROPERTY;

/**
 * @author John Ericksen
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = PROPERTY)
@JsonSubTypes({
        @JsonSubTypes.Type(value = MissionStart.class, name = "Start"),
        @JsonSubTypes.Type(value = MissionStepTakeoff.class, name = "Takeoff"),
        @JsonSubTypes.Type(value = MissionStepSleep.class, name = "Sleep"),
        @JsonSubTypes.Type(value = MissionStepLand.class, name = "Land"),
        @JsonSubTypes.Type(value = MissionStepGoto.class, name = "Waypoint"),
        @JsonSubTypes.Type(value = MissionStepSemaphore.class, name = "Semaphore"),
        @JsonSubTypes.Type(value = MissionStepRTL.class, name = "RTL"),
        @JsonSubTypes.Type(value = MissionStepDDSA.class, name = "DDSA"),
        @JsonSubTypes.Type(value = MissionStepLawnmower.class, name = "Lawnmower"),
        @JsonSubTypes.Type(value = MissionStepNavigation.class, name = "Navigation")
})
public interface MissionStep {
    boolean appliesTo(String name);

    ObjectNode toROSJson(ObjectMapper mapper);
}
