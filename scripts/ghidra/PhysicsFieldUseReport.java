// Find functions that use field offsets established by the dynamics config loader.
// @category KartRiderDemoPhysics

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;
import ghidra.program.model.scalar.Scalar;

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

public class PhysicsFieldUseReport extends GhidraScript {
    private static final Map<Long, String> FIELDS = new LinkedHashMap<>();
    static {
        FIELDS.put(0x128L, "mass");
        FIELDS.put(0x15cL, "air_friction");
        FIELDS.put(0x160L, "drag_factor");
        FIELDS.put(0x164L, "forward_accel_force");
        FIELDS.put(0x168L, "backward_accel_force");
        FIELDS.put(0x16cL, "grip_brake_force");
        FIELDS.put(0x170L, "slip_brake_force");
        FIELDS.put(0x174L, "max_steer_angle");
        FIELDS.put(0x178L, "steer_constraint");
        FIELDS.put(0x17cL, "front_grip_factor");
        FIELDS.put(0x180L, "rear_grip_factor");
        FIELDS.put(0x184L, "drift_trigger_factor");
        FIELDS.put(0x188L, "drift_trigger_time");
        FIELDS.put(0x18cL, "drift_slip_factor");
        FIELDS.put(0x190L, "drift_escape_force");
        FIELDS.put(0x194L, "corner_draw_factor");
        FIELDS.put(0x198L, "drift_lean_factor");
        FIELDS.put(0x19cL, "steer_lean_factor");
        FIELDS.put(0x218L, "weight_force");
        FIELDS.put(0x220L, "derived_weight_force");
    }

    private static class Hit {
        Function function;
        Map<Long, Set<Address>> uses = new LinkedHashMap<>();

        Hit(Function function) {
            this.function = function;
        }

        int distinctFields() {
            return uses.size();
        }

        int totalUses() {
            int total = 0;
            for (Set<Address> addresses : uses.values()) {
                total += addresses.size();
            }
            return total;
        }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        if (args.length != 1) {
            throw new IllegalArgumentException("Usage: PhysicsFieldUseReport.java <output-file>");
        }

        List<Hit> hits = new ArrayList<>();
        FunctionIterator functions = currentProgram.getFunctionManager().getFunctions(true);
        while (functions.hasNext()) {
            Function function = functions.next();
            if (function.isExternal()) {
                continue;
            }
            Hit hit = new Hit(function);
            InstructionIterator instructions = currentProgram.getListing()
                .getInstructions(function.getBody(), true);
            while (instructions.hasNext()) {
                Instruction instruction = instructions.next();
                for (int operand = 0; operand < instruction.getNumOperands(); operand++) {
                    for (Object object : instruction.getOpObjects(operand)) {
                        if (!(object instanceof Scalar)) {
                            continue;
                        }
                        long value = ((Scalar) object).getUnsignedValue();
                        if (FIELDS.containsKey(value)) {
                            hit.uses.computeIfAbsent(value, ignored -> new LinkedHashSet<>())
                                .add(instruction.getAddress());
                        }
                    }
                }
            }
            if (!hit.uses.isEmpty()) {
                hits.add(hit);
            }
        }

        hits.sort(Comparator
            .comparingInt(Hit::distinctFields).reversed()
            .thenComparing(Comparator.comparingInt(Hit::totalUses).reversed())
            .thenComparing(hit -> hit.function.getEntryPoint()));

        File output = new File(args[0]);
        File parent = output.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IllegalStateException("Cannot create output directory: " + parent);
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(output))) {
            writer.write("Functions using confirmed GoKart dynamics field offsets\n");
            writer.write("Caution: scalar matching can include unrelated constants; rank by distinct fields.\n\n");
            for (Hit hit : hits) {
                writer.write(String.format("%s %s distinct=%d uses=%d%n",
                    hit.function.getEntryPoint(), hit.function.getName(true),
                    hit.distinctFields(), hit.totalUses()));
                for (Map.Entry<Long, Set<Address>> entry : hit.uses.entrySet()) {
                    List<Address> addresses = new ArrayList<>(entry.getValue());
                    Collections.sort(addresses);
                    writer.write(String.format("  +0x%03x %-24s %s%n", entry.getKey(),
                        FIELDS.get(entry.getKey()), addresses));
                }
            }
        }

        println("Wrote physics field-use report: " + output.getAbsolutePath());
    }
}
