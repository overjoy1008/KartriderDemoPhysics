// List every instruction that embeds one of the requested scalar offsets.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;
import ghidra.program.model.scalar.Scalar;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.HashSet;
import java.util.Set;

public class OffsetUsers extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) {
            throw new IllegalArgumentException(
                "Usage: OffsetUsers.java <output-file> <hex-offset> [hex-offset ...]");
        }
        Set<Long> wanted = new HashSet<>();
        for (int i = 1; i < args.length; i++) {
            wanted.add(Long.parseLong(args[i].replaceFirst("^(0x|0X)", ""), 16));
        }

        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            FunctionIterator functions = currentProgram.getFunctionManager().getFunctions(true);
            while (functions.hasNext()) {
                Function function = functions.next();
                boolean wroteHeader = false;
                InstructionIterator instructions = currentProgram.getListing()
                    .getInstructions(function.getBody(), true);
                while (instructions.hasNext()) {
                    Instruction instruction = instructions.next();
                    boolean matches = false;
                    for (int operand = 0; operand < instruction.getNumOperands(); operand++) {
                        for (Object object : instruction.getOpObjects(operand)) {
                            if (object instanceof Scalar &&
                                wanted.contains(((Scalar) object).getUnsignedValue())) {
                                matches = true;
                            }
                        }
                    }
                    if (matches) {
                        if (!wroteHeader) {
                            writer.write(function.getEntryPoint() + " " + function.getName(true) + "\n");
                            wroteHeader = true;
                        }
                        writer.write("  " + instruction.getAddress() + " " + instruction + "\n");
                    }
                }
            }
        }
        println("Wrote offset-user report: " + output.getAbsolutePath());
    }
}
