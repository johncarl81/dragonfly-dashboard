package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.List;
import java.util.UUID;

/**
 * @author John Ericksen
 */
public class MissionStepSemaphore implements MissionStep {

    private final List<String> drones;
    private final String id;

    @JsonCreator
    public MissionStepSemaphore(@JsonProperty("drones") List<String> drones, @JsonProperty("id") String id) {
        this.drones = drones;
        this.id = id;
    }

    public MissionStepSemaphore(List<String> drones) {
        this(drones, UUID.randomUUID().toString());
    }

    public List<String> getDrones() {
        return drones;
    }

    public String getId() {
        return id;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drones.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper) {
        ObjectNode semaphore = mapper.createObjectNode();

        semaphore.put("msg_type", MissionStepType.SEMAPHORE.getMission_type());
        ObjectNode data = semaphore.putObject("semaphore");

        ArrayNode dronesArray = data.putArray("drones");
        drones.forEach(dronesArray::add);
        data.put("id", id);

        return semaphore;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepSemaphore that = (MissionStepSemaphore) o;

        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        return drones != null ? drones.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "Semaphore{" + drones + '}';
    }
}
