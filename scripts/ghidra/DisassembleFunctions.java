// Dump instructions for selected functions.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class DisassembleFunctions extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) throw new IllegalArgumentException(
            "Usage: DisassembleFunctions.java <output-file> <address> [address ...]");
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            for (int i = 1; i < args.length; i++) {
                Address requested = toAddr(args[i]);
                Function function = currentProgram.getFunctionManager().getFunctionContaining(requested);
                if (function == null) { writer.write("NO FUNCTION " + requested + "\n"); continue; }
                writer.write("FUNCTION " + function.getEntryPoint() + " " + function.getName(true) + "\n");
                InstructionIterator instructions = currentProgram.getListing()
                    .getInstructions(function.getBody(), true);
                while (instructions.hasNext()) {
                    Instruction instruction = instructions.next();
                    writer.write("  " + instruction.getAddress() + "  " + instruction + "\n");
                }
                writer.write("\n");
            }
        }
    }
}
