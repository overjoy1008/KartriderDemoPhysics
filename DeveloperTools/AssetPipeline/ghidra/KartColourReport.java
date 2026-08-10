// Find the demo's kart colour candidates.
//
// docs/KART_MODEL_CATALOG.md established that every kart skin is achromatic
// apart from a pure-blue (0, 0, 255) patch of about 900 texels in the same
// place on all 26. That patch is a marker the runtime recolours, so the colours
// it can take are a table in the executable rather than anything in the asset.
//
// This dumps every candidate table: runs of 32-bit values that look like packed
// colours (0x00RRGGBB / 0xAARRGGBB) and runs of floats in 0..1 that come in
// threes or fours, together with the functions that reference them.
//
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

public class KartColourReport extends GhidraScript {

    private static boolean looksLikeColour(int value) {
        // A packed colour with a zero or 0xff alpha byte and at least one
        // channel that is not 0 or 0xff, which rules out the flag words and
        // the -1 sentinels that otherwise dominate the data section.
        int alpha = (value >>> 24) & 0xff;
        if (alpha != 0x00 && alpha != 0xff) return false;
        int r = (value >>> 16) & 0xff;
        int g = (value >>> 8) & 0xff;
        int b = value & 0xff;
        if (r == 0 && g == 0 && b == 0) return false;
        if (r == 0xff && g == 0xff && b == 0xff) return false;
        int distinct = 0;
        if (r != 0 && r != 0xff) distinct++;
        if (g != 0 && g != 0xff) distinct++;
        if (b != 0 && b != 0xff) distinct++;
        return distinct >= 1;
    }

    private static boolean unitFloat(float value) {
        return value >= 0.0f && value <= 1.0f;
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 1) {
            throw new IllegalArgumentException("Usage: KartColourReport.java <output-file>");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        Memory memory = currentProgram.getMemory();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("# Kart colour candidates in " + currentProgram.getName() + "\n\n");

            for (MemoryBlock block : memory.getBlocks()) {
                if (!block.isInitialized() || block.isExecute()) continue;
                writer.write("## block " + block.getName() + " "
                        + block.getStart() + ".." + block.getEnd() + "\n\n");

                // Packed 32-bit colour runs.
                List<Integer> run = new ArrayList<>();
                Address runStart = null;
                for (Address a = block.getStart();
                     a.compareTo(block.getEnd().subtract(3)) <= 0;
                     a = a.add(4)) {
                    int value;
                    try {
                        value = memory.getInt(a);
                    } catch (Exception e) {
                        run.clear();
                        runStart = null;
                        continue;
                    }
                    if (looksLikeColour(value)) {
                        if (run.isEmpty()) runStart = a;
                        run.add(value);
                    } else {
                        emitColourRun(writer, runStart, run);
                        run.clear();
                        runStart = null;
                    }
                }
                emitColourRun(writer, runStart, run);

                // Runs of unit floats, which is how a colour table would look if
                // it were stored the way the renderer wants it.
                List<Float> floats = new ArrayList<>();
                Address floatStart = null;
                for (Address a = block.getStart();
                     a.compareTo(block.getEnd().subtract(3)) <= 0;
                     a = a.add(4)) {
                    float value;
                    try {
                        value = Float.intBitsToFloat(memory.getInt(a));
                    } catch (Exception e) {
                        emitFloatRun(writer, floatStart, floats);
                        floats.clear();
                        floatStart = null;
                        continue;
                    }
                    if (unitFloat(value) && value != 0.0f) {
                        if (floats.isEmpty()) floatStart = a;
                        floats.add(value);
                    } else if (value == 0.0f && !floats.isEmpty()) {
                        floats.add(value);
                    } else {
                        emitFloatRun(writer, floatStart, floats);
                        floats.clear();
                        floatStart = null;
                    }
                }
                emitFloatRun(writer, floatStart, floats);
            }
        }
        println("Wrote " + output.getAbsolutePath());
    }

    private void emitColourRun(BufferedWriter writer, Address start, List<Integer> run)
            throws Exception {
        if (start == null || run.size() < 4) return;
        writer.write("colours at " + start + " (" + run.size() + ")\n");
        for (int i = 0; i < run.size(); i++) {
            int v = run.get(i);
            writer.write(String.format("    [%2d] 0x%08x  rgb(%3d,%3d,%3d)%n",
                    i, v, (v >>> 16) & 0xff, (v >>> 8) & 0xff, v & 0xff));
        }
        writeReferences(writer, start);
        writer.write("\n");
    }

    private void emitFloatRun(BufferedWriter writer, Address start, List<Float> run)
            throws Exception {
        if (start == null || run.size() < 9 || run.size() > 256) return;
        writer.write("unit floats at " + start + " (" + run.size() + ")\n   ");
        for (int i = 0; i < run.size(); i++) {
            writer.write(String.format(" %.3f", run.get(i)));
            if ((i + 1) % 12 == 0) writer.write("\n   ");
        }
        writer.write("\n");
        writeReferences(writer, start);
        writer.write("\n");
    }

    private void writeReferences(BufferedWriter writer, Address start) throws Exception {
        ReferenceIterator references =
                currentProgram.getReferenceManager().getReferencesTo(start);
        int count = 0;
        while (references.hasNext() && count < 12) {
            Reference reference = references.next();
            Function function = getFunctionContaining(reference.getFromAddress());
            writer.write("    <- " + reference.getFromAddress()
                    + (function != null ? " in " + function.getName() : "") + "\n");
            count++;
        }
    }
}
