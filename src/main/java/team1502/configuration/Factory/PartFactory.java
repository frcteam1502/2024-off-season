package team1502.configuration.Factory;

import java.util.HashMap;
import java.util.function.Function;

import team1502.configuration.Builders.Builder;
import team1502.configuration.Builders.IBuild;
import team1502.configuration.Builders.Motor;
import team1502.configuration.Builders.PartBuilder;
import team1502.configuration.Builders.RobotBuilder;
import team1502.configuration.Builders.SwerveDrive;
import team1502.configuration.Builders.SwerveModule;
import team1502.configuration.Builders.Controllers.GyroSensor;
import team1502.configuration.Builders.Controllers.MotorController;
import team1502.configuration.CAN.Manufacturer;

public class PartFactory {
    private HashMap<String, PartBuilder> _partMap = new HashMap<>(); 
    private HashMap<String, Builder> _builderMap = new HashMap<>(); 
    
    private IBuild _build;
    private RobotBuilder _robotBuilder;
    public void setBuilder(RobotBuilder rb) {
        _robotBuilder=rb;
    }
    
    public PartFactory Part(String name, Function<Builder, Builder> fn) {
        _builderMap.put(name,  new Builder(name, fn));
        return this;
    }

    public void useBuilder(String partName, Builder builder) {
        var template = _builderMap.get(partName);
        if (template != null) {
            template.createBuilder(builder);
        }
    }
    
    public Builder getBuilder(String name) {
        var template = _builderMap.get(name);
        return template.createBuilder();
    }

    public PartFactory Motor(Function<Motor, Builder> fn) {
        return Motor("Motor", fn);
    }
    public PartFactory Motor(String name, Function<Motor, Builder> fn) {
        _builderMap.put(name,  new Motor(name, fn));
        return this;
    }
    
    // MOTOR CONTROLLER
    public PartFactory MotorController(Function<MotorController, Builder> fn) {
        return MotorController("MotorController", fn);
    }
    public PartFactory MotorController(String name, Manufacturer manufacturer, Function<MotorController, Builder> fn) {
        _builderMap.put(name,  new MotorController(name, manufacturer, fn));
        return this;
    }
    public PartFactory MotorController(String name, Function<MotorController, Builder> fn) {
        _builderMap.put(name,  new MotorController(name, fn));
        return this;
    }

    // GYRO SENSOR
    public PartFactory GyroSensor(String name, Manufacturer manufacturer, Function<GyroSensor, Builder> fn) {
        _builderMap.put(name,  new GyroSensor(name, manufacturer, fn));
        return this;
    }


    public PartFactory SwerveModule(Function<SwerveModule, Builder> fn) {
        _builderMap.put("SwerveModule",  new SwerveModule(fn));
        return this;
    }
    public PartFactory SwerveDrive(Function<SwerveDrive, Builder> fn) {
        _builderMap.put("SwerveDrive",  new SwerveDrive(fn));
        return this;
    }

    /*
     * 
    public PartFactory Part(String name, Function<Part, Part> fn) {
        _partMap.put(name,  new PartBuilder<Part>(name, nm -> new Part(name, _robotBuilder), fn));

        return this;
    }

    public PartFactory Gyro(String name, Manufacturer manufacturer, Function<GyroSensor, Part> fn) {
        _partMap.put(name, new PartBuilder<GyroSensor>(name, nm -> new GyroSensor(name, manufacturer), fn));
        return this;
    }

    public PartFactory MotorController(String name, Manufacturer manufacturer, Function<MotorController, Part> fn) {
        _partMap.put(name, new PartBuilder<MotorController>(name, nm -> new MotorController(name, manufacturer), fn));
        return this;
    }

    public PartFactory MotorController(String name, Function<Part, Part> fn) {
        // get parent name for full path
        return Part(name, fn);
    }
     */
}
