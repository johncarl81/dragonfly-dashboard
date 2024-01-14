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

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> partnerSelection;
    private TextField offset;
    private TextField threshold;
    private CheckBox leader;

    public SketchCreator(List<String> drones) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.partnerSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {

        leader = new CheckBox();
        offset = new TextField();
        threshold = new TextField();

        offset.setText("10");
        threshold.setText("425");
        leader.setSelected(false);

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Partner:", partnerSelection)
                .add("ε offset:", offset)
                .add("Threshold:", threshold)
                .add("Leader:", leader);
    }

    @Override
    public MissionStep build() {
        return new MissionStepSketch(droneSelection.getSelectionModel().getSelectedItem(),
                partnerSelection.getSelectionModel().getSelectedItem(),
                Double.parseDouble(offset.getText()),
                Double.parseDouble(threshold.getText()),
                leader.isSelected());
    }
}