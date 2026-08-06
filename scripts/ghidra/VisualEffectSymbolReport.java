// Report original executable symbols/strings and references related to camera and speed effects.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Locale;

public class VisualEffectSymbolReport extends GhidraScript {
    private static final String[] TERMS = {
        "blur", "distort", "motion", "speed", "fov", "fieldofview",
        "field_of_view", "zoom", "lens", "shake", "bloom", "trail",
        "streak", "boost", "booster", "nitro", "postprocess",
        "post_process", "render_target", "rendertarget", "cameraman",
        "camera", "kart"
    };

    private boolean matches(String text) {
        String lower = text.toLowerCase(Locale.ROOT);
        for (String term : TERMS) {
            if (lower.contains(term)) return true;
        }
        return false;
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException("Usage: VisualEffectSymbolReport.java <output-file>");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("KartRider.exe camera/speed visual-effect symbol report\n\n");
            SymbolIterator symbols = currentProgram.getSymbolTable().getAllSymbols(true);
            while (symbols.hasNext() && !monitor.isCancelled()) {
                Symbol symbol = symbols.next();
                String name = symbol.getName(true);
                if (!matches(name)) continue;

                writer.write("SYMBOL " + symbol.getAddress() + " " + name +
                    " type=" + symbol.getSymbolType() + "\n");
                ReferenceIterator refs = currentProgram.getReferenceManager()
                    .getReferencesTo(symbol.getAddress());
                int count = 0;
                while (refs.hasNext() && count < 64) {
                    Reference ref = refs.next();
                    Address from = ref.getFromAddress();
                    Function function = currentProgram.getFunctionManager()
                        .getFunctionContaining(from);
                    writer.write("  ref " + from + " " + ref.getReferenceType() +
                        " function=" + (function == null ? "none" :
                        function.getEntryPoint() + " " + function.getName(true)) + "\n");
                    count++;
                }
                writer.write("\n");
            }
        }
        println("Wrote visual-effect symbol report: " + output.getAbsolutePath());
    }
}
