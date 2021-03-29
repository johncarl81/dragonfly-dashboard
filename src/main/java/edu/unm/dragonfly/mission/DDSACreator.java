package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.Walk;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepDDSA;
import javafx.collections.FXCollections;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class DDSACreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;
    private final ComboBox<String> waypointSelection;
    private TextField radiusField;
    private TextField stepLengthField;
    private TextField loopsField;
    private TextField stacksField;
    private TextField altitudeField;
    private ComboBox<Walk> walkComboBox;
    private TextField waitTimeField;
    private TextField distanceThreshold;
    private CheckBox uniqueAltitudes;

    public DDSACreator(List<String> drones, List<String> waypoints) {
        this.droneSelection = new SelectableDronesCreator(drones);
        this.waypointSelection = new ComboBox<>(FXCollections.observableList(waypoints));
    }

    @Override
    public void create(GridPane grid) {

        radiusField = new TextField();
        stepLengthField = new TextField();
        loopsField = new TextField();
        stacksField = new TextField();
        altitudeField = new TextField();
        walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        waitTimeField = new TextField();
        distanceThreshold = new TextField();
        uniqueAltitudes = new CheckBox();

        // Set Defaults
        radiusField.setText("1");
        stepLengthField.setText("1");
        loopsField.setText("5");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkComboBox.getSelectionModel().select(Walk.RANGE);
        waitTimeField.setText("0");
        distanceThreshold.setText("1");
        uniqueAltitudes.setSelected(true);

        GridUtil.builder(grid).increment()
                .add(droneSelection)
                .add("Waypoint:", waypointSelection)
                .add("Radius:", radiusField)
                .add("Step Length:", stepLengthField)
                .add("Altitude:", altitudeField)
                .add("Loops:", loopsField)
                .add("Stacks:", stacksField)
                .add("Walk:", walkComboBox)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold)
                .add("Unique Altitudes:", uniqueAltitudes);

    }

    @Override
    public MissionStep build() {
        return new MissionStepDDSA(droneSelection.getSelectedDrones(),
                waypointSelection.getSelectionModel().getSelectedItem(),
                Float.parseFloat(radiusField.getText()),
                Float.parseFloat(stepLengthField.getText()),
                Float.parseFloat(altitudeField.getText()),
                Integer.parseInt(loopsField.getText()),
                Integer.parseInt(stacksField.getText()),
                walkComboBox.getSelectionModel().getSelectedItem(),
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()),
                uniqueAltitudes.isSelected());
    }
}
