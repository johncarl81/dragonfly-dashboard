package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.*;
import javafx.collections.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;

import java.util.*;

/**
 * @author John Ericksen
 */
public class FlockStopCreator implements MissionStepCreator {

    private final ComboBox<String> leaderSelection;

    public FlockStopCreator(List<String> drones) {
        this.leaderSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {
        GridUtil.builder(grid).increment()
                .add("Leader:", leaderSelection);
    }

    @Override
    public MissionStep build() {
        return new MissionStepFlockStop(leaderSelection.getSelectionModel().getSelectedItem());
    }
}
