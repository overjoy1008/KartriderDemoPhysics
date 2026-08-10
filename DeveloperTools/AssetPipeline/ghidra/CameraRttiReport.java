// Recover MSVC x86 camera-class vftables and their function entries.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.Reference;
import ghidra.program.model.symbol.ReferenceIterator;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class CameraRttiReport extends GhidraScript {
    private Address pointerAt(Address address) throws Exception {
        long value = Integer.toUnsignedLong(currentProgram.getMemory().getInt(address));
        return currentProgram.getAddressFactory().getDefaultAddressSpace().getAddress(value);
    }

    private boolean isExecutable(Address address) {
        MemoryBlock block = currentProgram.getMemory().getBlock(address);
        return block != null && block.isExecute();
    }

    private List<Address> rawPointerLocations(Address target) throws Exception {
        List<Address> result = new ArrayList<>();
        long value = target.getOffset();
        byte[] pattern = new byte[] {
            (byte)(value & 0xff),
            (byte)((value >>> 8) & 0xff),
            (byte)((value >>> 16) & 0xff),
            (byte)((value >>> 24) & 0xff),
        };
        Memory memory = currentProgram.getMemory();
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
                result.add(found);
                cursor = found.add(1);
            }
        }
        return result;
    }

    private void writeContainingFunction(BufferedWriter writer, Address address)
            throws Exception {
        Function function = currentProgram.getFunctionManager().getFunctionContaining(address);
        if (function == null) {
            writer.write("  containing-function: none\n");
        } else {
            writer.write("  containing-function: " + function.getEntryPoint() +
                " " + function.getName(true) + "\n");
        }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException("Usage: CameraRttiReport.java <output-file>");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("KartRider.exe camera RTTI and vftable report\n\n");
            SymbolIterator symbols = currentProgram.getSymbolTable().getAllSymbols(true);
            while (symbols.hasNext() && !monitor.isCancelled()) {
                Symbol symbol = symbols.next();
                String name = symbol.getName(true);
                String lower = name.toLowerCase(Locale.ROOT);
                if (!lower.contains("camera")) {
                    continue;
                }
                writer.write("SYMBOL " + symbol.getAddress() + " " + name +
                    " type=" + symbol.getSymbolType() + "\n");
                ReferenceIterator refs = currentProgram.getReferenceManager()
                    .getReferencesTo(symbol.getAddress());
                int referenceCount = 0;
                while (refs.hasNext() && referenceCount < 32) {
                    Reference ref = refs.next();
                    writer.write("  ref-from " + ref.getFromAddress() +
                        " type=" + ref.getReferenceType() + "\n");
                    writeContainingFunction(writer, ref.getFromAddress());
                    referenceCount++;
                }

                if (!lower.contains("rtti_type_descriptor") ||
                    !lower.contains("cameraman")) {
                    writer.write("\n");
                    continue;
                }

                writer.write("  RTTI traversal:\n");
                for (Address tdPointer : rawPointerLocations(symbol.getAddress())) {
                    Address completeObjectLocator = tdPointer.subtract(12);
                    writer.write("    type-pointer " + tdPointer +
                        " -> COL " + completeObjectLocator + "\n");
                    for (Address colPointer : rawPointerLocations(completeObjectLocator)) {
                        Address vftable = colPointer.add(4);
                        Address firstEntry;
                        try {
                            firstEntry = pointerAt(vftable);
                        } catch (Exception error) {
                            continue;
                        }
                        if (!isExecutable(firstEntry)) {
                            continue;
                        }
                        writer.write("      COL-pointer " + colPointer +
                            " -> vftable " + vftable + "\n");
                        for (int slot = 0; slot < 32; slot++) {
                            Address entryAddress = vftable.add(slot * 4L);
                            Address target;
                            try {
                                target = pointerAt(entryAddress);
                            } catch (Exception error) {
                                break;
                            }
                            if (!isExecutable(target)) {
                                break;
                            }
                            Function function = currentProgram.getFunctionManager()
                                .getFunctionAt(target);
                            writer.write("        [" + slot + "] " + entryAddress +
                                " -> " + target + " " +
                                (function == null ? "<no-function>" : function.getName(true)) +
                                "\n");
                        }
                    }
                }
                writer.write("\n");
            }
        }
        println("Wrote camera RTTI report: " + output.getAbsolutePath());
    }
}
