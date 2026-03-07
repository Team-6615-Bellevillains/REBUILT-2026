package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MeasureInterpolatingTreeMap<K extends Unit, U extends Unit> {

    private final InterpolatingTreeMap<Measure<K>, Measure<U>> innerMap;

    public MeasureInterpolatingTreeMap(){
        innerMap = new InterpolatingTreeMap<>(this::measureInverseInterpolator, this::measureInterpolator);
    }
    
    public double measureInverseInterpolator(Measure<K> startValue, Measure<K> endValue, Measure<K> query){
        return (query.magnitude() - startValue.magnitude())/(endValue.magnitude() - startValue.magnitude());
    }

    public Measure<U> measureInterpolator(Measure<U> startValue, Measure<U> endValue, double t){
        return endValue.minus(startValue).times(t).plus(startValue);
    }

    public void addPoint(Measure<K> key, Measure<U> value){
        innerMap.put(key, value);
    }

    public Measure<U> getValue(Measure<K> key){
        return innerMap.get(key);
    }
}
