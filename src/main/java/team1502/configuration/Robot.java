package team1502.configuration;

import java.util.function.Function;

import team1502.configuration.Builders.RobotBuilder;
import team1502.configuration.CAN.CanMap;
import team1502.configuration.Factory.PartFactory;

public class Robot {

    public String name;
    private PartFactory _partFactory = new PartFactory();
    private RobotBuilder _robotBuilder;
    private Evaluator _evaluator;
    
    public static Robot Create(String name, Function<Robot, Robot> fn) {
        var robot = new Robot();
        robot.name = name;
        return fn.apply(robot);

    }

    static Robot Create(Function<Robot, Robot> fn) {
        var robot = new Robot();
        return fn.apply(robot);

    }

    private RobotBuilder getBuilder() {
        if (_robotBuilder == null) {
            _robotBuilder = RobotBuilder.Create(_partFactory);
        }
        return _robotBuilder;
    }
    
    private Evaluator getEvaluator() {
        if (_evaluator == null) {
            _evaluator = new Evaluator(getBuilder());
        }
        return _evaluator;
    }

    public CanMap getCanMap() {return _robotBuilder.getCanMap();}

    public Robot Parts(Function<PartFactory, PartFactory> fn) {
        fn.apply(_partFactory);
        return this;
    }
    
    public Robot Build(Function<RobotBuilder, RobotBuilder> fn) {
        fn.apply(getBuilder());
        return this;
    }

    public Evaluator Values() {
        return getEvaluator();
    }

    public Robot Values(Function<Evaluator, Evaluator> fn) {
        fn.apply(getEvaluator());
        return this;
    }

    public Object getValue(String valueName) {
        return Values().getValue(valueName);
    }

    public Object getValue(String valueName, String partName) {
        return Values().getValue(valueName, partName);
    }

}
