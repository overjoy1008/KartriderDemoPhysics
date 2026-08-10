// Ghidra headless script: locate likely kart/vehicle/physics code.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import ghidra.program.model.listing.Program;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;
import ghidra.program.model.symbol.SymbolTable;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

public class PhysicsTriage extends GhidraScript {
    private static final String[] DOMAIN_TERMS = {
        "kart", "rider", "vehicle", "drive", "drift", "steer", "wheel",
        "accel", "brake", "boost", "collision", "ground", "gravity",
        "velocity", "speed", "physics", "torque", "suspension", "tire"
    };

    private static final Set<String> MATH_IMPORTS = new HashSet<>();
    static {
        Collections.addAll(MATH_IMPORTS,
            "_cisqrt", "_cisin", "_cicos", "_citan", "_ciatan", "_ciacos",
            "_cipow", "_ciexp", "_cilog", "floor", "sqrt", "sin", "cos",
            "tan", "atan", "acos", "pow", "exp", "log");
    }

    private static class Candidate {
        Function function;
        int score;
        Set<String> reasons = new HashSet<>();

        Candidate(Function function) {
            this.function = function;
        }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException("Usage: PhysicsTriage.java <output-file>");
        }

        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        Map<Address, Candidate> candidates = new HashMap<>();
        SymbolTable symbols = currentProgram.getSymbolTable();
        List<String> interestingSymbols = new ArrayList<>();
        List<String> vftables = new ArrayList<>();

        FunctionIterator functions = currentProgram.getFunctionManager().getFunctions(true);
        int functionCount = 0;
        while (functions.hasNext()) {
            Function function = functions.next();
            functionCount++;
            String lowered = function.getName(true).toLowerCase(Locale.ROOT);
            for (String term : DOMAIN_TERMS) {
                if (lowered.contains(term)) {
                    addCandidate(candidates, function, 8, "function-name:" + term);
                }
            }
        }

        SymbolIterator iterator = symbols.getAllSymbols(true);
        while (iterator.hasNext()) {
            Symbol symbol = iterator.next();
            String fullName = symbol.getName(true);
            String lowered = fullName.toLowerCase(Locale.ROOT);

            boolean domainMatch = false;
            for (String term : DOMAIN_TERMS) {
                if (lowered.contains(term)) {
                    domainMatch = true;
                    interestingSymbols.add(symbol.getAddress() + " " + fullName);
                    scoreReferences(candidates, symbol, 6, "symbol:" + term);
                    break;
                }
            }

            if (lowered.contains("vftable") || lowered.contains("vtable")) {
                vftables.add(symbol.getAddress() + " " + fullName);
                if (domainMatch) {
                    scoreReferences(candidates, symbol, 10, "domain-vftable");
                }
            }

            String leaf = symbol.getName().toLowerCase(Locale.ROOT);
            if (MATH_IMPORTS.contains(leaf)) {
                scoreReferences(candidates, symbol, 3, "math-call:" + leaf);
            }
        }

        List<Candidate> ranked = new ArrayList<>(candidates.values());
        ranked.sort(Comparator
            .comparingInt((Candidate c) -> c.score).reversed()
            .thenComparing(c -> c.function.getEntryPoint()));
        Collections.sort(interestingSymbols);
        Collections.sort(vftables);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("KartRider Demo physics triage\n");
            writer.write("Program: " + currentProgram.getName() + "\n");
            writer.write("Language: " + currentProgram.getLanguageID() + "\n");
            writer.write("Compiler: " + currentProgram.getCompilerSpec().getCompilerSpecID() + "\n");
            writer.write("Functions: " + functionCount + "\n\n");

            writer.write("[Memory blocks]\n");
            for (MemoryBlock block : currentProgram.getMemory().getBlocks()) {
                writer.write(String.format("%s %s-%s size=0x%x r=%s w=%s x=%s%n",
                    block.getName(), block.getStart(), block.getEnd(), block.getSize(),
                    block.isRead(), block.isWrite(), block.isExecute()));
            }

            writer.write("\n[Ranked candidate functions]\n");
            int limit = Math.min(250, ranked.size());
            for (int i = 0; i < limit; i++) {
                Candidate candidate = ranked.get(i);
                List<String> reasons = new ArrayList<>(candidate.reasons);
                Collections.sort(reasons);
                writer.write(String.format("%3d score=%2d %s %s reasons=%s%n",
                    i + 1, candidate.score, candidate.function.getEntryPoint(),
                    candidate.function.getName(true), String.join(",", reasons)));
            }

            writer.write("\n[Domain symbols]\n");
            for (String line : interestingSymbols) {
                writer.write(line + "\n");
            }

            writer.write("\n[Vftables]\n");
            for (String line : vftables) {
                writer.write(line + "\n");
            }
        }

        println("Wrote physics triage report: " + output.getAbsolutePath());
    }

    private void scoreReferences(Map<Address, Candidate> candidates, Symbol symbol,
            int points, String reason) {
        ReferenceIterator refs = currentProgram.getReferenceManager()
            .getReferencesTo(symbol.getAddress());
        while (refs.hasNext()) {
            Reference reference = refs.next();
            Function caller = currentProgram.getFunctionManager()
                .getFunctionContaining(reference.getFromAddress());
            if (caller != null && !caller.isExternal()) {
                addCandidate(candidates, caller, points, reason);
            }
        }
    }

    private void addCandidate(Map<Address, Candidate> candidates, Function function,
            int points, String reason) {
        Candidate candidate = candidates.computeIfAbsent(
            function.getEntryPoint(), ignored -> new Candidate(function));
        candidate.score += points;
        candidate.reasons.add(reason);
    }
}
