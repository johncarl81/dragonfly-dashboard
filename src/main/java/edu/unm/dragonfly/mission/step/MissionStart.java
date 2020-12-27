package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * @author John Ericksen
 */
public class MissionStart implements MissionStep {

    @Override
    public boolean appliesTo(String name) {
        return true;
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper) {
        ObjectNode missionStart = mapper.createObjectNode();

        missionStart.put("msg_type", MissionStepType.START.getMission_type());

        return missionStart;
    }

    @Override
    public String toString() {
        return "Start{}";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        return o != null && getClass() == o.getClass();
    }
}
