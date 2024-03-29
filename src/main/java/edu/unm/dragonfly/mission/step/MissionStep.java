package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.*;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.*;
import edu.unm.dragonfly.*;

import static com.fasterxml.jackson.annotation.JsonTypeInfo.As.*;

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
        @JsonSubTypes.Type(value = MissionStepNavigation.class, name = "Navigation"),
        @JsonSubTypes.Type(value = MissionStepFlock.class, name = "Flock"),
        @JsonSubTypes.Type(value = MissionStepGradient.class, name = "Gradient"),
        @JsonSubTypes.Type(value = MissionStepCurtain.class, name = "Curtain"),
        @JsonSubTypes.Type(value = MissionStepPump.class, name = "Pump"),
        @JsonSubTypes.Type(value = MissionStepCalibration.class, name = "Calibration"),
        @JsonSubTypes.Type(value = MissionStepSketch.class, name = "Sketch"),
        @JsonSubTypes.Type(value = MissionStepVerticalTransect.class, name = "VerticalTransect"),
        @JsonSubTypes.Type(value = MissionStepFlockStop.class, name = "FlockStop"),
})
public interface MissionStep {
    boolean appliesTo(String name);

    ObjectNode toROSJson(ObjectMapper mapper, String droneName);

    boolean references(Fixture fixture);
}
