// Report camera data constants and every executable reference to 90 degrees/pi/2.
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

public class CameraConstantReport extends GhidraScript {
    private byte[] littleEndian(float value) {
        int bits = Float.floatToRawIntBits(value);
        return new byte[] {
            (byte)(bits & 0xff),
            (byte)((bits >>> 8) & 0xff),
            (byte)((bits >>> 16) & 0xff),
            (byte)((bits >>> 24) & 0xff),
        };
    }

    private void reportFloatOccurrences(BufferedWriter writer, float wanted)
            throws Exception {
        Memory memory = currentProgram.getMemory();
        byte[] pattern = littleEndian(wanted);
        writer.write("FLOAT " + wanted + " bits=0x" +
            Integer.toHexString(Float.floatToRawIntBits(wanted)) + "\n");
        for (MemoryBlock block : memory.getBlocks()) {
            if (!block.isInitialized()) {
                continue;
            }
            Address cursor = block.getStart();
            while (cursor != null && cursor.compareTo(block.getEnd()) <= 0) {
                Address found = memory.findBytes(
                    cursor, block.getEnd(), pattern, null, true, monitor);
                if (found == null) {
                    break;
                }
                writer.write("  at " + found + " block=" + block.getName() + "\n");
                ReferenceIterator refs = currentProgram.getReferenceManager().getReferencesTo(found);
                while (refs.hasNext()) {
                    Reference ref = refs.next();
                    Function function = currentProgram.getFunctionManager()
                        .getFunctionContaining(ref.getFromAddress());
                    writer.write("    ref " + ref.getFromAddress() + " " +
                        (function == null ? "<no-function>" :
                         function.getEntryPoint() + " " + function.getName(true)) + "\n");
                }
                cursor = found.add(1);
            }
        }
        writer.write("\n");
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException("Usage: CameraConstantReport.java <output-file>");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("Chase camera constants\n\n");
            String[] addresses = {
                "00572678", "0057267c", "00572680", "00572684", "00572688",
                "00571490", "00571808", "005712c0", "005722c0"
            };
            Memory memory = currentProgram.getMemory();
            for (String text : addresses) {
                Address address = toAddr(text);
                int bits = memory.getInt(address);
                writer.write(address + " bits=0x" + Integer.toHexString(bits) +
                    " float=" + Float.intBitsToFloat(bits) + "\n");
            }
            writer.write("\n");
            reportFloatOccurrences(writer, 90.0f);
            reportFloatOccurrences(writer, (float)(Math.PI / 2.0));
            reportFloatOccurrences(writer, -((float)(Math.PI / 2.0)));
        }
        println("Wrote camera constant report: " + output.getAbsolutePath());
    }
}
