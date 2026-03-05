package frc.robot.commands.auto.config;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.PARAMETER)
public @interface ConfigParam {
    String value();
    String description() default "";
    String unit() default "";
    double min() default Double.NEGATIVE_INFINITY;
    double max() default Double.POSITIVE_INFINITY;
    double defaultValue() default 0;
    boolean required() default true;
    String[] options() default {};
}
