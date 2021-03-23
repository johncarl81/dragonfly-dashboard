package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepLand implements MissionStep {

    private final List<String> drones;

    @JsonCreator
    public MissionStepLand(@JsonProperty("drones") List<String> drones) {
        this.drones = drones;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drones.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode land = mapper.createObjectNode();

        land.put("msg_type", MissionStepType.LAND.getMission_type());

        return land;
    }

    public List<String> getDrones() {
        return drones;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepLand that = (MissionStepLand) o;

        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        return drones != null ? drones.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "Land{" +
                "drones=" + drones +
                '}';
    }
}
