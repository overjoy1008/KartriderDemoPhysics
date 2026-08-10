// Report original executable strings/symbols and their referencing functions for
// race-progress features: checkpoints, lap counting, reverse-direction detection
// and respawn/restart. Read-only reporting; nothing is renamed or patched.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.DataIterator;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Locale;
import java.util.TreeSet;

public class RaceProgressSymbolReport extends GhidraScript {
    private static final String[] TERMS = {
        // checkpoint / progress
        "checkpoint", "check_point", "checkpt", "cp_", "waypoint", "way_point",
        "lap", "progress", "section", "split", "node",
        // reverse direction
        "reverse", "reversed", "backward", "wrongway", "wrong_way", "wrong way",
        "opposite", "retrograde", "counter",
        // respawn / restart
        "respawn", "re_spawn", "spawn", "revive", "restart", "reset",
        "rescue", "recover", "comeback", "return",
        // race state
        "rank", "finish", "goal", "start_line", "startline", "startpos",
        "gridslot", "grid_slot", "track", "course", "path", "route"
    };

    private boolean matches(String text) {
        String lower = text.toLowerCase(Locale.ROOT);
        for (String term : TERMS) {
            if (lower.contains(term)) return true;
        }
        return false;
    }

    private void writeReferences(BufferedWriter writer, Address address) throws Exception {
        ReferenceIterator refs = currentProgram.getReferenceManager()
            .getReferencesTo(address);
        TreeSet<String> seen = new TreeSet<>();
        int count = 0;
        while (refs.hasNext() && count < 48) {
            Reference ref = refs.next();
            Function from = currentProgram.getFunctionManager()
                .getFunctionContaining(ref.getFromAddress());
            seen.add("    ref " + ref.getFromAddress() +
                (from != null ? " in " + from.getEntryPoint() + " " + from.getName(true)
                              : " (no function)"));
            count++;
        }
        for (String line : seen) {
            writer.write(line + "\n");
        }
        if (seen.isEmpty()) {
            writer.write("    (no references)\n");
        }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException(
                "Usage: RaceProgressSymbolReport.java <output-file>");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("KartRider.exe race-progress symbol and string report\n");
            writer.write("Checkpoint / lap / reverse-direction / respawn search.\n\n");

            writer.write("=== DEFINED STRINGS ===\n");
            int stringHits = 0;
            DataIterator data = currentProgram.getListing().getDefinedData(true);
            while (data.hasNext() && !monitor.isCancelled()) {
                Data item = data.next();
                Object value = item.getValue();
                if (!(value instanceof String)) continue;
                String text = (String) value;
                if (text.length() < 3 || !matches(text)) continue;
                writer.write("STRING " + item.getAddress() + " " +
                    item.getDataType().getName() + " \"" + text + "\"\n");
                writeReferences(writer, item.getAddress());
                stringHits++;
            }
            writer.write("string matches: " + stringHits + "\n\n");

            writer.write("=== SYMBOLS ===\n");
            int symbolHits = 0;
            SymbolIterator symbols = currentProgram.getSymbolTable().getAllSymbols(true);
            while (symbols.hasNext() && !monitor.isCancelled()) {
                Symbol symbol = symbols.next();
                String name = symbol.getName(true);
                if (name.startsWith("FUN_") || name.startsWith("DAT_") ||
                    name.startsWith("LAB_") || name.startsWith("SUB_") ||
                    name.startsWith("s_") || name.startsWith("u_")) {
                    continue;
                }
                if (!matches(name)) continue;
                writer.write("SYMBOL " + symbol.getAddress() + " " + name +
                    " type=" + symbol.getSymbolType() + "\n");
                writeReferences(writer, symbol.getAddress());
                symbolHits++;
            }
            writer.write("symbol matches: " + symbolHits + "\n");
        }

        println("Wrote race-progress report");
    }
}
