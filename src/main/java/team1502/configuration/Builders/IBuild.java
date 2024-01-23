package team1502.configuration.Builders;

import java.util.function.Function;

import team1502.configuration.Parts.Part;

public interface IBuild {
    //void getBuilder(String name);
    //Part createPart(String newName, String partName);
    //void install(Builder builder);

    Builder createBuilder(String partName, Function<? extends Builder, Builder> fn);
    Builder modifyBuilder(String partName, Function<? extends Builder, Builder> fn);
}
