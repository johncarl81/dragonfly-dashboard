package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.LawnmowerDialogFactory;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepLawnmower;
import javafx.collections.FXCollections;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LawnmowerCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> boundarySelection;
    private TextField stepLengthField;
    private TextField stacksField;
    private TextField altitudeField;
    private CheckBox walkBoundaryField;
    private ComboBox<LawnmowerDialogFactory.Walk> walkComboBox;
    private TextField waitTimeField;
    private TextField distanceThreshold;

    public LawnmowerCreator(List<String> drones, List<String> boundaries) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.boundarySelection = new ComboBox<>(FXCollections.observableList(boundaries));
    }

    @Override
    public void create(GridPane grid) {

        stepLengthField = new TextField();
        stacksField = new TextField();
        altitudeField = new TextField();
        walkBoundaryField = new CheckBox();
        walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(LawnmowerDialogFactory.Walk.values());
        waitTimeField = new TextField();
        distanceThreshold = new TextField();

        // Set Defaults
        stepLengthField.setText("1");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkBoundaryField.setSelected(true);
        walkComboBox.getSelectionModel().select(LawnmowerDialogFactory.Walk.RANGE);
        waitTimeField.setText("0");
        distanceThreshold.setText("1");

        GridUtil.builder(grid).increment().increment()
                .add("Drone:", droneSelection)
                .add("Boundary:", boundarySelection)
                .add("Step Length:", stepLengthField)
                .add("Altitude:", altitudeField)
                .add("Stacks:", stacksField)
                .add("Walk Boundary:", walkBoundaryField)
                .add("Walk:", walkComboBox)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold);
    }

    @Override
    public MissionStep build() {
        return new MissionStepLawnmower(
                boundarySelection.getSelectionModel().getSelectedItem(),
                droneSelection.getSelectionModel().getSelectedItem(),
                Float.parseFloat(stepLengthField.getText()),
                Float.parseFloat(altitudeField.getText()),
                Integer.parseInt(stacksField.getText()),
                walkBoundaryField.isSelected(),
                walkComboBox.getSelectionModel().getSelectedItem().id,
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }
}
