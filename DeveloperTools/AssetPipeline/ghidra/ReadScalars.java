// Read auditable 32-bit scalar values at selected addresses.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class ReadScalars extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) {
            throw new IllegalArgumentException(
                "Usage: ReadScalars.java <output-file> <address> [address ...]");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            for (int i = 1; i < args.length; i++) {
                Address address = toAddr(args[i]);
                int bits = currentProgram.getMemory().getInt(address);
                writer.write(String.format("%s  hex=%08x  uint=%d  int=%d  float=%.9g%n",
                    address, bits, Integer.toUnsignedLong(bits), bits, Float.intBitsToFloat(bits)));
            }
        }
    }
}
