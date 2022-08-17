include <openscad-util/cycloid_profiles.scad>
include <openscad-util/zero_finders.scad>
include <openscad-util/radial_square_notches.scad>

wheel_thickness = 6;
pinion_thickness = 7;
//wheel_pd = 80;
//wheel_teeth = 96;
//pinion_ratio = 8;

$fs = 0.02;
$fa = 0.02;

module cycloid_wheel_and_pinion(pd, tooth_count, gear_ratio, wheel_thickness, pinion_thickness, prev_pd, prev_gear_ratio, prev_tooth_count, spoke_width=4, spoke_count=8, spoke_bound_fractions=[0.2, 0.8]) {
  spoke_length = spoke_bound_fractions[1] * pd / 2;
  linear_extrude(wheel_thickness, convexity=5) {
    difference() {
      epicycloid_wheel_profile(
        pitch_diameter=pd,
        tooth_count=tooth_count, 
        pinion_gear_ratio=gear_ratio
      );
      difference() {
        circle(r=spoke_bound_fractions[1] * pd / 2);
        circle(r=spoke_bound_fractions[0] * pd / 2);
      }
    }
    for (a = [0:360/spoke_count:360 - 360 / spoke_count]) {
      rotate([0, 0, a])
        translate([spoke_length / 2, 0, 0])
          square([spoke_length, spoke_width], center=true);
    }
  }
    
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

  pinion_dedenum_len = 1.15 * abs(
    epicycloid_point(generating_radius, 2 * prev_gear_ratio, 0)[0] - 
    epicycloid_point(generating_radius, 2 * prev_gear_ratio, addendum_t)[0]
  ); 
  translate([0, 0, wheel_thickness])
    linear_extrude(pinion_thickness, convexity=4)
      pinion_profile(
        pitch_diameter=prev_pd / prev_gear_ratio,
        prev_tooth_count / prev_gear_ratio,
        dedendum_depth=pinion_dedenum_len
      );
}

wheel_pds = [96, 80, 75, 72, 40];
pinion_pds = [0, 12, 10, 10, 7];
ratios = [8, 8, 7.5, 8, 10];
wheel_teeth = [96, 80, 75, 80, 15];
pinion_teeth = [0, 12, 10, 10, 8];
barrel_shaft_diameter = 12;
barrel_shaft_notch_size = 1.5;
shaft_diameter = 4;
shaft_notch_size = 0.75;
barrel_spoke_length = 0.8 * wheel_pds[0] / 2;

difference() {
  linear_extrude(wheel_thickness, convexity=4) {
    difference() {
      epicycloid_wheel_profile(
        pitch_diameter=wheel_pds[0],
        tooth_count=wheel_teeth[0],
        pinion_gear_ratio=ratios[1]
      );
      difference() {
        circle(r=0.8 * wheel_pds[0] / 2);
        circle(r=0.35 * wheel_pds[0] / 2);
      }
    }
    for (a = [0:360/12:360 - 360 / 12]) {
      rotate([0, 0, a])
        translate([barrel_spoke_length / 2, 0, 0])
          square([barrel_spoke_length, 5], center=true);
    }
  }
  cylinder(r = 1.02 * barrel_shaft_diameter / 2, h=(wheel_thickness + pinion_thickness) * 3, center=true);
  radial_square_notches(
    side_length=barrel_shaft_notch_size * 1.02,
    length=wheel_thickness * 3,
    notches=4,
    radius= 1.02 * barrel_shaft_diameter / 2
  );
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
    radial_square_notches(
      side_length=shaft_notch_size * 1.02,
      length=(wheel_thickness + pinion_thickness)* 3,
      notches=1,
      radius= 1.02 * shaft_diameter / 2
    );
  }
}
