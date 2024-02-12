package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.*;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.databind.node.*;
import edu.unm.dragonfly.*;

import java.util.*;

/**
 * @author John Ericksen
 */
public class MissionStepFlockStop implements MissionStep {

    private final String drone;

    @JsonCreator
    public MissionStepFlockStop(@JsonProperty("drone") String drone) {
        this.drone = drone;
    }

    public String getDrone() {
        return drone;
    }

    @Override
    public boolean references(Fixture fixture) {
        return false;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode semaphore = mapper.createObjectNode();
        semaphore.put("msg_type", MissionStepType.FLOCK_STOP.getMission_type());
        return semaphore;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepFlockStop that = (MissionStepFlockStop) o;

        return Objects.equals(drone, that.drone);
    }

    @Override
    public int hashCode() {
        return drone != null ? drone.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "FlockStop{" + drone + '}';
    }
}
