include <openscad-util/cycloid_profiles.scad>
include <openscad-util/zero_finders.scad>

wheel_thickness = 6;
pinion_thickness = 7;
//wheel_pd = 80;
//wheel_teeth = 96;
//pinion_ratio = 8;

$fs = 0.025;
$fa = 0.05;

module cycloid_wheel_and_pinion(pd, tooth_count, gear_ratio, wheel_thickness, pinion_thickness, prev_pd, prev_gear_ratio, prev_tooth_count) {

  linear_extrude(wheel_thickness, convexity=3)
    epicycloid_wheel_profile(
      pitch_diameter=pd,
      tooth_count=tooth_count, 
      pinion_gear_ratio=gear_ratio
    );
    
  prev_tooth_angle = 360 / prev_tooth_count / 2;
  generating_radius = pd / 2 / prev_gear_ratio / 2;
  obj_func = function(t)
    epicycloid_point(generating_radius, 2 * prev_gear_ratio, t)[1] - 
    sin(prev_tooth_angle / 2) * prev_pd / 2;

  addendum_t = find_zero_bisect(
    f=obj_func,
    [prev_tooth_angle, 30],
    iterations=30
  );

  pinion_dedenum_len = 1.3 * abs(
    epicycloid_point(generating_radius, 2 * prev_gear_ratio, 0)[0] - 
    epicycloid_point(generating_radius, 2 * prev_gear_ratio, addendum_t)[0]
  ); 
  translate([0, 0, wheel_thickness])
    linear_extrude(pinion_thickness, convexity=3)
      pinion_profile(
        pitch_diameter=prev_pd / prev_gear_ratio,
        prev_tooth_count / prev_gear_ratio,
        dedendum_depth=pinion_dedenum_len
      );
}

wheel_pds = [80, 80, 75, 80, 40];
pinion_pds = [0, 10, 10, 10, 8];
ratios = [8, 8, 7.5, 8, 10];
wheel_teeth = [96, 80, 75, 80, 15];
pinion_teeth = [0, 12, 10, 10, 8];
shaft_diameter = 3;
shaft_notch_size = 0.5;

difference() {
  linear_extrude(wheel_thickness, convexity=3)
    epicycloid_wheel_profile(
      pitch_diameter=wheel_pds[0],
      tooth_count=wheel_teeth[0],
      pinion_gear_ratio=ratios[1]
    );
  cylinder(r = 1.02 * shaft_diameter / 2, h=(wheel_thickness + pinion_thickness) * 3, center=true);
  translate([1.02 * shaft_diameter / 2, 0])
    cube([shaft_notch_size * 2, shaft_notch_size, (wheel_thickness + pinion_thickness) * 3], center=true);
}
  
function total_offset(idx) = 
  idx == 0 ? 0 : total_offset(idx - 1) + wheel_pds[idx - 1] / 2 + pinion_pds[idx] / 2;

for (idx = [1:len(wheel_pds) - 2]) {
  x_offset = wheel_pds[idx - 1] / 2 + pinion_pds[idx] / 2;
  wheel_center = [total_offset(idx), 0, -pinion_thickness * idx * 2];
  echo(wheel_center);
  translate(wheel_center)
    difference() {
      cycloid_wheel_and_pinion(
        pd=wheel_pds[idx],
        tooth_count=wheel_teeth[idx], 
        gear_ratio=ratios[idx],
        wheel_thickness=wheel_thickness,
        pinion_thickness=pinion_thickness,
        prev_pd=wheel_pds[idx - 1],
        prev_gear_ratio=ratios[idx - 1],
        prev_tooth_count=wheel_teeth[idx - 1]
    );
    cylinder(r = 1.02 * shaft_diameter / 2, h=(wheel_thickness + pinion_thickness) * 3, center=true);
    translate([1.02 * shaft_diameter / 2, 0])
      cube([shaft_notch_size * 2, shaft_notch_size, (wheel_thickness + pinion_thickness) * 3], center=true);
  }
}
