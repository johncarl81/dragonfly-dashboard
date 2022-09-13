package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Fixture;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepCalibration implements MissionStep {

    private final String target;
    private final List<String> drones;

    @JsonCreator
    public MissionStepCalibration(@JsonProperty("target") String target, @JsonProperty("drones") List<String> drones) {
        this.target = target;
        this.drones = drones;
    }

    public String getTarget() {
        return target;
    }

    public List<String> getDrones() {
        return drones;
    }

    @Override
    public boolean references(Fixture fixture) {
        return false;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.target.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode semaphore = mapper.createObjectNode();

        semaphore.put("msg_type", MissionStepType.CALIBRATION.getMission_type());
        ObjectNode data = semaphore.putObject("calibrate_step");

        ArrayNode dronesArray = data.putArray("drones");
        drones.forEach(dronesArray::add);

        return semaphore;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepCalibration that = (MissionStepCalibration) o;

        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        return drones != null ? drones.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "Calibrate{" +
                "target='" + target + '\'' +
                ", drones=" + drones +
                '}';
    }
}
