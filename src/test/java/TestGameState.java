import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.GameState;

public class TestGameState {
    @Test
    public void Test1() {
        assertEquals(true, GameState.isFirst());
        GameState.robotPeriodic();
        assertEquals(false, GameState.isFirst());
    }
}
