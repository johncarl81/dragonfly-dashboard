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

    private final String leader;
    private final String partner;
    private final double offset;
    private final double threshold;

    @JsonCreator
    public MissionStepSketch(@JsonProperty("leader") String leader,
                             @JsonProperty("partner") String partner,
                             @JsonProperty("offset") double offset,
                             @JsonProperty("threshold") double threshold) {
        this.leader = leader;
        this.partner = partner;
        this.offset = offset;
        this.threshold = threshold;
    }

    @Override
    public boolean references(Fixture fixture) {
        return false;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.leader.equals(name) || this.partner.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode flock = mapper.createObjectNode();

        flock.put("msg_type", MissionStepType.SKETCH.getMission_type());
        ObjectNode data = flock.putObject("sketch_step");

        data.put("offset", offset);
        data.put("threshold", threshold);
        boolean leaderDrone = leader.equals(droneName);
        data.put("leader", leaderDrone);
        if (leaderDrone) {
            data.put("partner", partner);
        } else {
            data.put("partner", leader);
        }

        return flock;
    }



    @Override
    public String toString() {
        return "Sketch{" +
                "leader='" + leader + '\'' +
                ", partner='" + partner + '\'' +
                ", offset=" + offset +
                ", threshold=" + threshold +
                '}';
    }
}
