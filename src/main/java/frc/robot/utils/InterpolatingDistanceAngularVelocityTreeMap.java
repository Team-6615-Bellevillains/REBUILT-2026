package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

// Unit-safe wrapper around InterpolatingDoubleTreeMap for mapping arbitray distances to arbitrary angular velocities.
//
// Allows entry of data points in any distance and angular velocity unit, 
// and retrieval of interpolated angular velocity values in any angular 
// velocity unit, given a distance in any distance unit.
//
// For example:
//
/* InterpolatingDistanceAngularVelocityTreeMap map = new InterpolatingDistanceAngularVelocityTreeMap();
   
   map.addDatapoint(Inches.of(12), RPM.of(1000));

   // Equivalent to 24 inches, 2000 RPM
   map.addDatapoint(Feet.of(2), Rotations.per(Second).of(2000d / 60d)); 
   
   // Since 0.4572 meters is roughly between 12 inches and 2 feet, this will 
   // return an AngularVelocity value that is roughly between 1000 RPM and 2000 RPM
   AngularVelocity result = map.getInterpolatedValue(Meters.of(0.4572)); 
*/
public class InterpolatingDistanceAngularVelocityTreeMap {
    private final InterpolatingDoubleTreeMap internalTree = new InterpolatingDoubleTreeMap();

    private static final DistanceUnit keyUnit = Meters;
    private static final AngularVelocityUnit valueUnit = RotationsPerSecond;

    public void addDatapoint(Distance key, AngularVelocity value) {
        internalTree.put(key.in(keyUnit), value.in(valueUnit));
    }

    public AngularVelocity getInterpolatedValue(Distance key) {
        return valueUnit.of(internalTree.get(key.in(keyUnit)));
    }   
}

