
include <bearing-openscad/parts/outer_race.scad>
include <openscad-util/radial_square_notches.scad>
include <openscad-util/rounded_bars.scad>

$fs = 0.1;
$fa = 0.1;

center_leniency = 0.5;
bearing_od = 40;
bearing_notch_depth = 1.5;
base_height = 10;

//gear_centers = [
//  [0, 0],
//  [45 + center_leniency, 0],
//  [90 + center_leniency * 2, 0],
//  [132.5 + center_leniency * 3, 0],
//  [176.5 + center_leniency * 4, 0],
//];

gear_centers = [
  [0, 0],
  [54 + center_leniency, 0],
  [99 + center_leniency * 2, 0],
  [141.5 + center_leniency * 3, 0],
  //[185.5 + center_leniency * 4, 0],
];

gear_heights = [
  0,
  7,
  14,
  21,
  28
];

function next_center_off(gear_centers, idx, default) = 
  idx + 1 < len(gear_centers) ? gear_centers[idx + 1][0] : gear_centers[idx][0] + default;

module bearing_holder(base_height, bearing_od, bearing_notch_depth, notches=16, mount_hole_id=8) {
  base_length = 1.5 * bearing_od + gear_centers[len(gear_centers) - 1][0];
  echo(base_width);
  base_width = 1.5 * bearing_od;
  bearing_effective_od = bearing_od * 1.0075;
  bearing_effective_notch_depth = bearing_notch_depth * 0.96;
  mount_hold_off = bearing_od / 1.8;
  outrigger_len = 15;
  bar_width = 5;
  corner_coords = [
    [base_length / 2, base_width / 2] + [base_length / 2 - bearing_od * 0.75, 0],
    [-base_length / 2, base_width / 2] + [base_length / 2 - bearing_od * 0.75, 0],
    [-base_length / 2, -base_width / 2] + [base_length / 2 - bearing_od * 0.75, 0],
    [base_length / 2, -base_width / 2] + [base_length / 2 - bearing_od * 0.75, 0],
  ];
  outrigger_offset = base_width / 2 + outrigger_len;
  outrigger_bar_points = [
    // lined up with bearing centers
    for (idx = [0:len(gear_centers) - 1]) [[gear_centers[idx][0], base_width / 2], [gear_centers[idx][0], outrigger_offset]],
    for (idx = [0:len(gear_centers) - 1]) [[gear_centers[idx][0], -base_width / 2], [gear_centers[idx][0], -outrigger_offset]],
    // at the ends
    for (idx = [0:len(corner_coords) - 1]) [
      corner_coords[idx] - [sign(corner_coords[idx][0]) * bar_width / 2, 0], 
      corner_coords[idx] - [sign(corner_coords[idx][0]) * bar_width / 2, -sign(corner_coords[idx][1]) * outrigger_len]
    ],
    // along the full length
    [corner_coords[0] + [-bar_width / 2, outrigger_len], corner_coords[1] + [bar_width / 2, outrigger_len]],
    [corner_coords[2] + [bar_width / 2, -outrigger_len], corner_coords[3] + [-bar_width / 2, -outrigger_len]],
  ];
  echo(outrigger_bar_points);
  linear_extrude(base_height) {
    difference() {
      translate([base_length / 2 - bearing_od * 0.75, 0])
        square([base_length, base_width], center=true);
      translate([-bearing_od / 2, mount_hold_off])
        circle(r=mount_hole_id / 2);
      translate([-bearing_od / 2, -mount_hold_off])
        circle(r=mount_hole_id / 2);
      for (idx = [0:len(gear_centers) - 1]) {
        translate(gear_centers[idx])
          circle(r=bearing_effective_od / 2);
        translate([
          (gear_centers[idx][0] + next_center_off(gear_centers, idx, bearing_od)) / 2,
          mount_hold_off
        ])
          circle(r=mount_hole_id / 2);
        translate([
          (gear_centers[idx][0] + next_center_off(gear_centers, idx, bearing_od)) / 2,
          -mount_hold_off
        ])
          circle(r=mount_hole_id / 2);
      }
    }
    for (pt_idx = [0:len(outrigger_bar_points) - 1]) {
      rounded_bar_point2point_profile(
        p1=outrigger_bar_points[pt_idx][0],
        p2=outrigger_bar_points[pt_idx][1],
        width=bar_width,
      );
    }
    difference() {
      for (idx = [0:len(gear_centers) - 1]) {
        translate(gear_centers[idx])
          radial_square_notches_profile(bearing_effective_notch_depth, 16, bearing_effective_od / 2);
      }
    }
//      rounded_bar_point2point_profile(
//        p1=gear_centers[idx] + [0, bearing_od / 1.5],
//        p2=gear_centers[idx] + [0, outrigger_offset],
//        width=bar_width,
//      );
//      rounded_bar_point2point_profile(
//        p1=gear_centers[idx] + [0, -bearing_od / 1.5],
//        p2=gear_centers[idx] + [0, -outrigger_offset],
//        width=bar_width,
//      );
//    }
//    // outer stretches
//    rounded_bar_point2point_profile(
//      p1=gear_centers[0] + [0, outerigger_offset],
//      p2=gear_centers[len(gear_centers) - 1] + [0, outerigger_offset],
//      width=bar_width,
//    );
//    rounded_bar_point2point_profile(
//      p1=gear_centers[0] + [0, -outerigger_offset],
//      p2=gear_centers[len(gear_centers) - 1] + [0, -outerigger_offset],
//      width=bar_width,
//    );
//    // diagonals
//    rounded_bar_point2point_profile(
//      p1=corner_coords[1] + [bar_width/2, outerigger_len -bar_width/2],
//      p2=corner_coords[1] + [bar_width/2, -bar_width/2],
//      width=bar_width,
//    );
//    rounded_bar_point2point_profile(
//      p1=gear_centers[0] + [0, -outerigger_offset],
//      p2=corner_coords[2] + [bar_width/2, bar_width/2],
//      width=bar_width,
//    );
//    rounded_bar_point2point_profile(
//      p1=gear_centers[len(gear_centers) - 1] + [0, outerigger_offset],
//      p2=corner_coords[0] + [-bar_width/2, -bar_width/2],
//      width=bar_width,
//    );
//    rounded_bar_point2point_profile(
//      p1=gear_centers[len(gear_centers) - 1] + [0, -outerigger_offset],
//      p2=corner_coords[3] + [-bar_width/2, bar_width/2],
//      width=bar_width,
//    );
  }
}

bearing_holder(
  base_height=base_height, 
  bearing_od=bearing_od, 
  bearing_notch_depth=bearing_notch_depth,
);
