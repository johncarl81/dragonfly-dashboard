package edu.unm.dragonfly.mission;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.unm.dragonfly.Walk;
import edu.unm.dragonfly.mission.step.MissionStart;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepDDSA;
import edu.unm.dragonfly.mission.step.MissionStepGoto;
import edu.unm.dragonfly.mission.step.MissionStepLand;
import edu.unm.dragonfly.mission.step.MissionStepLawnmower;
import edu.unm.dragonfly.mission.step.MissionStepRTL;
import edu.unm.dragonfly.mission.step.MissionStepSemaphore;
import edu.unm.dragonfly.mission.step.MissionStepSleep;
import edu.unm.dragonfly.mission.step.MissionStepTakeoff;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.*;

/**
 * @author John Ericksen
 */
public class MissionDataHolderTest {

    @Test
    public void testMissionStepSerialization() throws IOException {
        ObjectMapper mapper = new ObjectMapper();

        List<MissionStep> steps = new ArrayList<>();

        steps.add(new MissionStart());
        steps.add(new MissionStepTakeoff(Arrays.asList("one", "two"), 3.1415));
        steps.add(new MissionStepSleep(Arrays.asList("one"), 42));
        steps.add(new MissionStepLand(Arrays.asList("two")));
        steps.add(new MissionStepGoto("three", "waypoint 1"));
        steps.add(new MissionStepRTL(Arrays.asList("one")));
        steps.add(new MissionStepSemaphore(Arrays.asList("one")));
        steps.add(new MissionStepDDSA("four", 1, 2, 3, 4, 5, Walk.RANGE, 10, 20));
        steps.add(new MissionStepLawnmower("four", 5, 4, 3, true, 1, 5, 3));

        MissionDataHolder holder = new MissionDataHolder(steps);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, holder);

        MissionDataHolder result = mapper.readValue(outputStream.toByteArray(), MissionDataHolder.class);

        assertEquals(steps, result.getSteps());
    }
}
