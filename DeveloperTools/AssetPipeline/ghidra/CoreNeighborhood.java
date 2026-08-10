// Report callers/callees of core physics roots and users of runtime state offsets.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;
import ghidra.program.model.scalar.Scalar;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class CoreNeighborhood extends GhidraScript {
    private static final long[] STATE_OFFSETS = {
        0x0e5, 0x0e6, 0x124, 0x158, 0x1a0, 0x1e4, 0x1e8, 0x1ec,
        0x200, 0x20c, 0x2ac, 0x2b0, 0x2b4, 0x2b8, 0x2bc,
        0x2c0, 0x2c4, 0x2c8, 0x2c9, 0x2ca, 0x2cc, 0x2d0,
        0x2d4, 0x2d8, 0x2dc, 0x2e0, 0x2e4, 0x2e8, 0x2ec, 0x2f0, 0x2f4
    };

    private static class StateHit {
        Function function;
        Map<Long, Set<Address>> offsets = new LinkedHashMap<>();
        StateHit(Function function) { this.function = function; }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) {
            throw new IllegalArgumentException(
                "Usage: CoreNeighborhood.java <output-file> <root-address> [root-address ...]");
        }

        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        List<StateHit> stateHits = collectStateHits();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("Core physics call graph and runtime-state users\n\n");
            for (int i = 1; i < args.length; i++) {
                Function root = currentProgram.getFunctionManager().getFunctionAt(toAddr(args[i]));
                if (root == null) {
                    writer.write("ROOT " + args[i] + " not found\n\n");
                    continue;
                }
                writer.write("ROOT " + root.getEntryPoint() + " " + root.getName(true) + "\n");
                writer.write("  CALLERS\n");
                for (Function caller : callersOf(root)) {
                    writer.write("    " + caller.getEntryPoint() + " " + caller.getName(true) + "\n");
                }
                writer.write("  CALLEES\n");
                for (Function callee : calleesOf(root)) {
                    writer.write("    " + callee.getEntryPoint() + " " + callee.getName(true) + "\n");
                }
                writer.write("\n");
            }

            writer.write("[Functions using runtime state offsets]\n");
            for (StateHit hit : stateHits) {
                writer.write(String.format("%s %s distinct=%d%n",
                    hit.function.getEntryPoint(), hit.function.getName(true), hit.offsets.size()));
                for (Map.Entry<Long, Set<Address>> entry : hit.offsets.entrySet()) {
                    writer.write(String.format("  +0x%03x %s%n", entry.getKey(), entry.getValue()));
                }
            }
        }
        println("Wrote core-neighborhood report: " + output.getAbsolutePath());
    }

    private List<Function> callersOf(Function target) {
        Set<Function> result = new LinkedHashSet<>();
        ReferenceIterator refs = currentProgram.getReferenceManager()
            .getReferencesTo(target.getEntryPoint());
        while (refs.hasNext()) {
            Reference ref = refs.next();
            Function caller = currentProgram.getFunctionManager()
                .getFunctionContaining(ref.getFromAddress());
            if (caller != null && !caller.isExternal()) {
                result.add(caller);
            }
        }
        List<Function> sorted = new ArrayList<>(result);
        sorted.sort(Comparator.comparing(Function::getEntryPoint));
        return sorted;
    }

    private List<Function> calleesOf(Function source) {
        Set<Function> result = new LinkedHashSet<>();
        InstructionIterator instructions = currentProgram.getListing()
            .getInstructions(source.getBody(), true);
        while (instructions.hasNext()) {
            Instruction instruction = instructions.next();
            for (Reference ref : instruction.getReferencesFrom()) {
                if (!ref.getReferenceType().isCall()) {
                    continue;
                }
                Function callee = currentProgram.getFunctionManager()
                    .getFunctionAt(ref.getToAddress());
                if (callee != null) {
                    result.add(callee);
                }
            }
        }
        List<Function> sorted = new ArrayList<>(result);
        sorted.sort(Comparator.comparing(Function::getEntryPoint));
        return sorted;
    }

    private List<StateHit> collectStateHits() {
        Set<Long> wanted = new LinkedHashSet<>();
        for (long offset : STATE_OFFSETS) {
            wanted.add(offset);
        }

        List<StateHit> hits = new ArrayList<>();
        FunctionIterator functions = currentProgram.getFunctionManager().getFunctions(true);
        while (functions.hasNext()) {
            Function function = functions.next();
            if (function.isExternal()) {
                continue;
            }
            StateHit hit = new StateHit(function);
            InstructionIterator instructions = currentProgram.getListing()
                .getInstructions(function.getBody(), true);
            while (instructions.hasNext()) {
                Instruction instruction = instructions.next();
                for (int operand = 0; operand < instruction.getNumOperands(); operand++) {
                    for (Object object : instruction.getOpObjects(operand)) {
                        if (object instanceof Scalar) {
                            long value = ((Scalar) object).getUnsignedValue();
                            if (wanted.contains(value)) {
                                hit.offsets.computeIfAbsent(value, ignored -> new LinkedHashSet<>())
                                    .add(instruction.getAddress());
                            }
                        }
                    }
                }
            }
            if (hit.offsets.size() >= 3) {
                hits.add(hit);
            }
        }
        hits.sort(Comparator
            .comparingInt((StateHit hit) -> hit.offsets.size()).reversed()
            .thenComparing(hit -> hit.function.getEntryPoint()));
        return hits;
    }
}
