// Traverse MSVC x86 RTTI to vftables for matching type-descriptor names.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class RttiVtableReport extends GhidraScript {
    private Address pointerAt(Address address) throws Exception {
        return toAddr(Integer.toUnsignedLong(currentProgram.getMemory().getInt(address)));
    }

    private boolean executable(Address address) {
        MemoryBlock block = currentProgram.getMemory().getBlock(address);
        return block != null && block.isExecute();
    }

    private List<Address> pointersTo(Address target) throws Exception {
        List<Address> result = new ArrayList<>();
        long value = target.getOffset();
        byte[] pattern = {
            (byte)value, (byte)(value >>> 8), (byte)(value >>> 16), (byte)(value >>> 24)
        };
        Memory memory = currentProgram.getMemory();
        for (MemoryBlock block : memory.getBlocks()) {
            if (!block.isInitialized()) continue;
            Address cursor = block.getStart();
            while (cursor != null && cursor.compareTo(block.getEnd()) <= 0) {
                Address found = memory.findBytes(cursor, block.getEnd(), pattern, null, true, monitor);
                if (found == null) break;
                result.add(found);
                cursor = found.add(1);
            }
        }
        return result;
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length < 2) {
            throw new IllegalArgumentException("Usage: RttiVtableReport.java <output-file> <term> [term ...]");
        }
        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            SymbolIterator symbols = currentProgram.getSymbolTable().getAllSymbols(true);
            while (symbols.hasNext() && !monitor.isCancelled()) {
                Symbol symbol = symbols.next();
                String lower = symbol.getName(true).toLowerCase(Locale.ROOT);
                if (!lower.contains("rtti_type_descriptor")) continue;
                boolean match = false;
                for (int i = 1; i < args.length; i++) {
                    if (lower.contains(args[i].toLowerCase(Locale.ROOT))) { match = true; break; }
                }
                if (!match) continue;

                writer.write("TYPE " + symbol.getAddress() + " " + symbol.getName(true) + "\n");
                for (Address tdPointer : pointersTo(symbol.getAddress())) {
                    Address col = tdPointer.subtract(12);
                    for (Address colPointer : pointersTo(col)) {
                        Address vftable = colPointer.add(4);
                        Address first;
                        try { first = pointerAt(vftable); } catch (Exception error) { continue; }
                        if (!executable(first)) continue;
                        writer.write("  COL " + col + " pointer " + colPointer + " vftable " + vftable + "\n");
                        for (int slot = 0; slot < 96; slot++) {
                            Address cell = vftable.add(slot * 4L);
                            Address target;
                            try { target = pointerAt(cell); } catch (Exception error) { break; }
                            if (!executable(target)) break;
                            Function function = currentProgram.getFunctionManager().getFunctionAt(target);
                            writer.write("    [" + slot + "] " + cell + " -> " + target + " " +
                                (function == null ? "<no-function>" : function.getName(true)) + "\n");
                        }
                    }
                }
                writer.write("\n");
            }
        }
        println("Wrote RTTI/vftable report: " + output.getAbsolutePath());
    }
}
