package com.team3478.frc2025.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

// Clase para hacer comandos que se inicializen en el runtime y esperar a que terminen.
// Util para casos cuando queremos hacer un calculo antes de generar el comando.

public class LazyCommandWrapper extends Command {

    private final Supplier<Command> commandSupplier;
    private Command currentCommand = null;;

    public LazyCommandWrapper(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public void initialize() {
        // Get the command in runtime
        currentCommand = commandSupplier.get();
        if (currentCommand != null) {
            currentCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (currentCommand != null) {
            currentCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (currentCommand == null) {
            return true;
        }
        return currentCommand.isFinished();
    }
}
