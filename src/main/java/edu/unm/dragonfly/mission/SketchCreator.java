package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepSketch;
import javafx.collections.FXCollections;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class SketchCreator implements MissionStepCreator {

    private final ComboBox<String> leaderSelection;
    private final ComboBox<String> partnerSelection;
    private TextField offset;
    private TextField threshold;

    public SketchCreator(List<String> drones) {
        this.leaderSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.partnerSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {

        offset = new TextField();
        threshold = new TextField();

        offset.setText("10");
        threshold.setText("425");

        GridUtil.builder(grid).increment()
                .add("Leader:", leaderSelection)
                .add("Partner:", partnerSelection)
                .add("ε offset:", offset)
                .add("Threshold:", threshold);
    }

    @Override
    public MissionStep build() {
        return new MissionStepSketch(leaderSelection.getSelectionModel().getSelectedItem(),
                partnerSelection.getSelectionModel().getSelectedItem(),
                Double.parseDouble(offset.getText()),
                Double.parseDouble(threshold.getText()));
    }
}