package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MeasureInterpolatingTreeMap<KeyUnit extends Unit, ValueUnit extends Unit> {

    private final InterpolatingTreeMap<Measure<KeyUnit>, Measure<ValueUnit>> innerMap;

    public MeasureInterpolatingTreeMap(){
        innerMap = new InterpolatingTreeMap<>(this::measureInverseInterpolator, this::measureInterpolator);
    }
    
    public void addPoint(Measure<KeyUnit> key, Measure<ValueUnit> value){
        innerMap.put(key, value);
    }

    public Measure<ValueUnit> getValue(Measure<KeyUnit> key){
        return innerMap.get(key);
    }
    
    private Measure<ValueUnit> measureInterpolator(Measure<ValueUnit> startValue, Measure<ValueUnit> endValue, double t){
        return endValue.minus(startValue).times(t).plus(startValue);
    }

    private double measureInverseInterpolator(Measure<KeyUnit> startValue, Measure<KeyUnit> endValue, Measure<KeyUnit> query){
        return (query.baseUnitMagnitude() - startValue.baseUnitMagnitude())/(endValue.baseUnitMagnitude() - startValue.baseUnitMagnitude());
    }
}
