package frc.robot.commands.auto.config;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.lang.reflect.Constructor;
import java.lang.reflect.Parameter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class AutoCommandRegistry {

    public static class SubsystemRegistry {
        private final Map<Class<?>, Subsystem> subsystems = new HashMap<>();

        public <T extends Subsystem> void register(Class<T> type, T subsystem) {
            subsystems.put(type, subsystem);
        }

        @SuppressWarnings("unchecked")
        public <T extends Subsystem> T get(Class<T> type) {
            return (T) subsystems.get(type);
        }
    }

    public record ParamInfo(
        String name,
        Class<?> javaType,
        String unit,
        String description,
        double min,
        double max,
        double defaultValue,
        boolean required,
        String[] options
    ) {}

    public record CommandRegistration(
        String typeName,
        String description,
        String category,
        Class<?> commandClass,
        Constructor<?> constructor,
        List<ParamInfo> params
    ) {}

    private final Map<String, CommandRegistration> registrations = new LinkedHashMap<>();
    private static final Gson gson = new Gson();

    public void register(Class<?> commandClass) {
        AutoConfigurable annotation = commandClass.getAnnotation(AutoConfigurable.class);
        if (annotation == null) {
            throw new IllegalArgumentException(
                commandClass.getName() + " is not annotated with @AutoConfigurable");
        }

        Constructor<?> selectedConstructor = null;
        for (Constructor<?> c : commandClass.getConstructors()) {
            if (c.getParameterCount() > 0) {
                selectedConstructor = c;
                break;
            }
        }
        if (selectedConstructor == null) {
            // Use default constructor
            try {
                selectedConstructor = commandClass.getConstructors()[0];
            } catch (Exception e) {
                throw new IllegalArgumentException(
                    "No accessible constructor found for " + commandClass.getName());
            }
        }

        List<ParamInfo> params = new ArrayList<>();
        for (Parameter param : selectedConstructor.getParameters()) {
            ConfigParam cp = param.getAnnotation(ConfigParam.class);
            if (cp != null) {
                params.add(new ParamInfo(
                    cp.value(),
                    param.getType(),
                    cp.unit(),
                    cp.description(),
                    cp.min(),
                    cp.max(),
                    cp.defaultValue(),
                    cp.required(),
                    cp.options()
                ));
            }
            // Subsystem-typed params are not added to ParamInfo — they're injected
        }

        registrations.put(annotation.value(), new CommandRegistration(
            annotation.value(),
            annotation.description(),
            annotation.category(),
            commandClass,
            selectedConstructor,
            params
        ));
    }

    public Command createCommand(String type, Map<String, Object> params, SubsystemRegistry subs) {
        CommandRegistration reg = registrations.get(type);
        if (reg == null) {
            System.out.println("AutoCommandRegistry: Unknown command type: " + type);
            return null;
        }

        try {
            Constructor<?> ctor = reg.constructor();
            Parameter[] ctorParams = ctor.getParameters();
            Object[] args = new Object[ctorParams.length];

            for (int i = 0; i < ctorParams.length; i++) {
                Parameter p = ctorParams[i];
                ConfigParam cp = p.getAnnotation(ConfigParam.class);

                if (cp != null) {
                    // Config parameter — pull from map
                    Object raw = params.get(cp.value());
                    args[i] = coerce(raw, p.getType(), cp);
                } else if (Subsystem.class.isAssignableFrom(p.getType())) {
                    // Subsystem parameter — inject by type
                    @SuppressWarnings("unchecked")
                    Class<? extends Subsystem> subType = (Class<? extends Subsystem>) p.getType();
                    args[i] = subs.get(subType);
                    if (args[i] == null) {
                        System.out.println("AutoCommandRegistry: No subsystem registered for "
                            + subType.getSimpleName());
                        return null;
                    }
                } else {
                    System.out.println("AutoCommandRegistry: Cannot resolve parameter "
                        + p.getName() + " of type " + p.getType().getSimpleName()
                        + " for command " + type);
                    return null;
                }
            }

            return (Command) ctor.newInstance(args);
        } catch (Exception e) {
            System.out.println("AutoCommandRegistry: Error creating " + type + ": " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    public boolean hasType(String type) {
        return registrations.containsKey(type);
    }

    public String getMetadataJson() {
        List<Map<String, Object>> metadata = new ArrayList<>();
        for (CommandRegistration reg : registrations.values()) {
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("type", reg.typeName());
            entry.put("description", reg.description());
            entry.put("category", reg.category());

            List<Map<String, Object>> paramList = new ArrayList<>();
            for (ParamInfo pi : reg.params()) {
                Map<String, Object> paramEntry = new LinkedHashMap<>();
                paramEntry.put("name", pi.name());
                paramEntry.put("javaType", pi.javaType().getSimpleName());
                paramEntry.put("unit", pi.unit());
                paramEntry.put("description", pi.description());
                if (pi.min() != Double.NEGATIVE_INFINITY) {
                    paramEntry.put("min", pi.min());
                }
                if (pi.max() != Double.POSITIVE_INFINITY) {
                    paramEntry.put("max", pi.max());
                }
                paramEntry.put("defaultValue", pi.defaultValue());
                paramEntry.put("required", pi.required());
                if (pi.options().length > 0) {
                    paramEntry.put("options", pi.options());
                }
                paramList.add(paramEntry);
            }
            entry.put("params", paramList);
            metadata.add(entry);
        }
        return gson.toJson(metadata);
    }

    private Object coerce(Object raw, Class<?> targetType, ConfigParam cp) {
        // Handle options (string-enum params)
        if (cp.options().length > 0 && targetType == String.class) {
            if (raw == null) return cp.options()[0];
            return String.valueOf(raw);
        }

        if (raw == null) {
            if (targetType == double.class) return cp.defaultValue();
            if (targetType == int.class) return (int) cp.defaultValue();
            if (targetType == boolean.class) return cp.defaultValue() != 0;
            if (targetType == String.class) return "";
            return null;
        }

        // Gson deserializes numbers as Double
        if (raw instanceof Number number) {
            if (targetType == double.class || targetType == Double.class) {
                return number.doubleValue();
            }
            if (targetType == int.class || targetType == Integer.class) {
                return number.intValue();
            }
            if (targetType == boolean.class || targetType == Boolean.class) {
                return number.doubleValue() != 0;
            }
        }

        if (raw instanceof Boolean b) {
            if (targetType == boolean.class || targetType == Boolean.class) {
                return b;
            }
            if (targetType == double.class) return b ? 1.0 : 0.0;
            if (targetType == int.class) return b ? 1 : 0;
        }

        if (raw instanceof String s) {
            if (targetType == String.class) return s;
            if (targetType == double.class || targetType == Double.class) {
                try { return Double.parseDouble(s); } catch (NumberFormatException e) { return cp.defaultValue(); }
            }
            if (targetType == int.class || targetType == Integer.class) {
                try { return Integer.parseInt(s); } catch (NumberFormatException e) { return (int) cp.defaultValue(); }
            }
            if (targetType == boolean.class || targetType == Boolean.class) {
                return "true".equalsIgnoreCase(s);
            }
        }

        return raw;
    }
}
