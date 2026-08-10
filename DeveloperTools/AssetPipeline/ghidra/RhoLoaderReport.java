// Reports references and nearby decompilation for KartRider's RHO/1S loader strings.
// Run with analyzeHeadless as a post-script against an already analyzed program.

import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.data.StringDataInstance;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.DataIterator;
import ghidra.program.model.symbol.Reference;

import java.util.LinkedHashSet;
import java.util.Set;

public class RhoLoaderReport extends GhidraScript {
    private boolean interesting(String s) {
        if (s == null) return false;
        String lower = s.toLowerCase();
        return lower.contains(".rho") || lower.contains("track.1s") ||
               lower.contains("rh layer spec") || lower.contains("veblush");
    }

    @Override
    protected void run() throws Exception {
        Set<Function> functions = new LinkedHashSet<>();
        println("RHO/1S string references for " + currentProgram.getName());

        DataIterator allData = currentProgram.getListing().getDefinedData(true);
        while (allData.hasNext()) {
            Data data = allData.next();
            if (data.hasStringValue()) {
                String value = StringDataInstance.getStringDataInstance(data).getStringValue();
                if (interesting(value)) {
                    println("\nSTRING " + data.getAddress() + " = " + value);
                    for (Reference ref : getReferencesTo(data.getAddress())) {
                        println("  REF " + ref.getFromAddress() + " (" + ref.getReferenceType() + ")");
                        Function f = getFunctionContaining(ref.getFromAddress());
                        if (f != null) functions.add(f);
                    }
                }
            }
        }

        DecompInterface decompiler = new DecompInterface();
        decompiler.openProgram(currentProgram);
        for (Function f : functions) {
            println("\n========== " + f.getName() + " @ " + f.getEntryPoint() + " ==========");
            DecompileResults results = decompiler.decompileFunction(f, 60, monitor);
            if (results.decompileCompleted()) {
                println(results.getDecompiledFunction().getC());
            } else {
                println("DECOMPILE FAILED: " + results.getErrorMessage());
            }
        }
        decompiler.dispose();
    }
}
