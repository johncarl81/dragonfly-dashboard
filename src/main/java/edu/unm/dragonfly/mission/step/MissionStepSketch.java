package edu.unm.dragonfly.mission.step;


import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Fixture;

/**
 * @author John Ericksen
 */
public class MissionStepSketch implements MissionStep {

    private final String drone;
    private final String partner;
    private final double offset;
    private final double threshold;
    private final boolean leader;

    @JsonCreator
    public MissionStepSketch(@JsonProperty("drone") String drone,
                             @JsonProperty("partner") String partner,
                             @JsonProperty("offset") double offset,
                             @JsonProperty("threshold") double threshold,
                             @JsonProperty("leader") boolean leader) {
        this.drone = drone;
        this.partner = partner;
        this.offset = offset;
        this.threshold = threshold;
        this.leader = leader;
    }


    public String getDrone() {
        return drone;
    }

    public String getPartner() {
        return partner;
    }

    public double getOffset() {
        return offset;
    }

    public boolean isLeader() {
        return leader;
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
        ObjectNode flock = mapper.createObjectNode();

        flock.put("msg_type", MissionStepType.SKETCH.getMission_type());
        ObjectNode data = flock.putObject("sketch_step");

        data.put("offset", offset);
        data.put("threshold", threshold);
        data.put("partner", partner);
        data.put("leader", leader);

        return flock;
    }



    @Override
    public String toString() {
        return "Sketch{" +
                "drone='" + drone + '\'' +
                ", partner='" + partner + '\'' +
                ", offset=" + offset +
                ", threshold=" + threshold +
                ", leader=" + leader +
                '}';
    }
}
