// Dump program symbols whose name matches a regex, with their addresses.
//
// The demo's engine registers its classes through a hand-rolled RTTI table
// rather than the compiler's, so the class names survive as data symbols even
// though the binary carries only one real ".?AV" descriptor. Searching the
// symbol table is therefore the fastest way from a class name to its code.
//
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.regex.Pattern;

public class SymbolSearch extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) {
            throw new IllegalArgumentException(
                "Usage: SymbolSearch.java <output-file> <name-regex> [name-regex ...]");
        }

        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        List<Pattern> patterns = new ArrayList<>();
        for (int i = 1; i < args.length; i++) {
            patterns.add(Pattern.compile(args[i], Pattern.CASE_INSENSITIVE));
        }

        List<String> lines = new ArrayList<>();
        SymbolIterator symbols = currentProgram.getSymbolTable().getAllSymbols(true);
        while (symbols.hasNext()) {
            Symbol symbol = symbols.next();
            String name = symbol.getName(true);
            boolean matched = false;
            for (Pattern pattern : patterns) {
                if (pattern.matcher(name).find()) {
                    matched = true;
                    break;
                }
            }
            if (!matched) continue;
            Function function = currentProgram.getFunctionManager()
                .getFunctionContaining(symbol.getAddress());
            lines.add(symbol.getAddress() + "  " + symbol.getSymbolType() + "  " + name +
                (function == null ? "" : "   [in " + function.getEntryPoint() + " " +
                    function.getName(true) + "]"));
        }
        Collections.sort(lines);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            for (String line : lines) {
                writer.write(line);
                writer.write("\n");
            }
        }
        println("Wrote " + lines.size() + " symbols: " + output.getAbsolutePath());
    }
}
