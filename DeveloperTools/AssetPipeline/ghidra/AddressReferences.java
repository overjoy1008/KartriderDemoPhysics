// List references to selected addresses and their containing functions.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class AddressReferences extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) throw new IllegalArgumentException(
            "Usage: AddressReferences.java <output-file> <address> [address ...]");
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            for (int i = 1; i < args.length; i++) {
                Address target = toAddr(args[i]);
                writer.write("TARGET " + target + "\n");
                ReferenceIterator refs = currentProgram.getReferenceManager().getReferencesTo(target);
                while (refs.hasNext()) {
                    Reference ref = refs.next();
                    Function function = currentProgram.getFunctionManager()
                        .getFunctionContaining(ref.getFromAddress());
                    writer.write("  " + ref.getFromAddress() + " " + ref.getReferenceType() +
                        " function=" + (function == null ? "none" :
                        function.getEntryPoint() + " " + function.getName(true)) + "\n");
                }
                writer.write("\n");
            }
        }
    }
}
