include <openscad-util/cycloid_profiles.scad>
include <openscad-util/zero_finders.scad>

wheel_thickness = 4;
pinion_thickness = 6;
//wheel_pd = 80;
//wheel_teeth = 96;
//pinion_ratio = 8;

$fs = 0.1;
$fa = 0.05;

module cycloid_wheel_and_pinion(pd, tooth_count, gear_ratio, wheel_thickness, pinion_thickness, prev_pd, prev_gear_ratio, prev_tooth_count) {

  linear_extrude(wheel_thickness)
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
    linear_extrude(pinion_thickness)
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

linear_extrude(wheel_thickness)
  epicycloid_wheel_profile(
    pitch_diameter=wheel_pds[0],
    tooth_count=wheel_teeth[0],
    pinion_gear_ratio=ratios[1]
  );
  
function total_offset(idx) = 
  idx == 0 ? 0 : total_offset(idx - 1) + wheel_pds[idx - 1] / 2 + pinion_pds[idx] / 2;

for (idx = [1:1:len(wheel_pds) - 1]) {
  x_offset = wheel_pds[idx - 1] / 2 + pinion_pds[idx] / 2;
  translate([total_offset(idx), 0, -wheel_thickness * idx])
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
}
//
//// Barrel
//
//
////difference() {
//cycloid_wheel_and_pinion(
//  pd=75,
//  tooth_count=75, 
//  gear_ratio=7.5,
//  wheel_thickness=5,
//  pinion_thickness=6,
//  prev_pd=80,
//  prev_gear_ratio=8,
//  prev_tooth_count=80
//);
//translate([80 / 2 + 75 / 7.5 / 2, 0, 5 + 1])
//# cycloid_wheel_and_pinion(
//  pd=80,
//  tooth_count=80, 
//  gear_ratio=8,
//  wheel_thickness=5,
//  pinion_thickness=5,
//  prev_pd=75,
//  prev_gear_ratio=7.5,
//  prev_tooth_count=75
//);


//  cylinder(r=6, h=30, center=true);
//}
