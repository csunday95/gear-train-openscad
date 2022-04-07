
function rotate_pt(pt, angle) = [
  pt[0] * cos(angle) - pt[1] * sin(angle), 
  pt[0] * sin(angle) + pt[1] * cos(angle)
];

function rotate_pt_about(pt, center, angle) = add_pts(rotate_pt(diff_pts(pt, center), angle), center);

function rot_pt_90(pt) = [
  -pt[1], pt[0]
];

function add_pts(p1, p2) = [
  p1[0] + p2[0], p1[1] + p2[1]
];

function scalar_mult_pt(p1, s) = [
  p1[0] * s, p1[1] * s
];

function scalar_add(p1, s) = [
  p1[0] + s, p1[1] + s
];

function diff_pts(p1, p2) = [
  p1[0] - p2[0],
  p1[1] - p2[1]
];

function normalized(pt) = (
  let (n = norm(pt)) 
    [pt[0] / n, pt[1] / n]
);

function midpoint(p1, p2) = [
  (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
];

function vec_dot(v1, v2) = v1[0] * v2[0] + v1[1] * v2[1];

function vec_angle(dir1, dir2) = acos(vec_dot(dir1, dir2) / (norm(dir1) * norm(dir2)));

module debug_pt(pt, r = 0.25) {
  translate(pt)
    circle(r=r);
}

module circle_3_points(p1, p2, p3) {
  mp12 = midpoint(p1, p2);
  mp23 = midpoint(p2, p3);
  v12 = rot_pt_90(normalized(diff_pts(p2, p1)));
  v23 = rot_pt_90(normalized(diff_pts(p3, p2)));
  dir_ratio_23 = v23[1] / v23[0];
  t_intersect = (dir_ratio_23 * (mp23[0] - mp12[0]) + mp12[1] - mp23[1]) / (v12[0] * v23[1] / v23[0] - v12[1]);
  center = add_pts(mp12, scalar_mult_pt(v12, t_intersect));
  radius = norm(diff_pts(p1, center));
  translate(center)
    circle(r=radius);
}

module debug_dir(dir, width=0.25) {
  echo(dir);
  rot_angle = atan2(dir[1], dir[0]);
  rotate([0, 0, rot_angle])
    polygon([
      [0, -width],
      [0, width],
      [norm(dir), 0]
    ]);
}

module polygon_arc(p1, p2, radius, step=0.1) {
  mp12 = midpoint(p1, p2);
  point_dist = norm(diff_pts(p2, p1));
  orthogonal_dir = rot_pt_90(normalized(diff_pts(p2, p1)));
  // angle of the sector of the circle spanning p1 to p2
  sector_angle = 2 * asin(point_dist / 2 / radius);
  center = add_pts(mp12, scalar_mult_pt(orthogonal_dir, radius * cos(sector_angle / 2)));
  // debug_pt(center);
  p1_rel_dir = diff_pts(p1, center);
  p1_x_angle = atan2(p1_rel_dir[1], p1_rel_dir[0]);
  circle_func = function(r, theta) [r * cos(theta), r * sin(theta)];
  points = [for (a = [0:step:sector_angle]) circle_func(radius, a)];
  points_w_center = concat(points, [[0, 0]]);
  translate(center)
    rotate([0, 0, p1_x_angle])
      polygon(points_w_center);
}

module escape_wheel_profile(wheel_r, tooth_angle, first_tooth_inner_point, outer_circle_point, outer_tooth_tip_pt, dedendum_x_axis_intersection, balance_roller_pitch_r) {
  difference() {
    circle(r=wheel_r);
    for(a = [0:tooth_angle:360 - tooth_angle]) {
      rotate([0, 0, a - tooth_angle / 2]) {
        polygon([
          first_tooth_inner_point,
          [wheel_r, 0],
          outer_circle_point,
          outer_tooth_tip_pt,
          dedendum_x_axis_intersection
        ]);
        difference() {
          polygon_arc(
            outer_tooth_tip_pt,
            dedendum_x_axis_intersection,
            radius = wheel_r,
            step = 0.05
          );
          translate(dedendum_x_axis_intersection)
            mirror([0, 1, 0])
              square([balance_roller_pitch_r / 2, balance_roller_pitch_r / 2]);
        }
      }
    }
  }
}

module impulse_roller_profile(roller_r, crescent_chord_len, escaping_angle) {
  crescent_subtend_angle = asin(crescent_chord_len / 2 / roller_r);
  crescent_end_pt = rotate_pt([roller_r, 0], crescent_subtend_angle);
  // figure out what this is actually supposed to be
  pallet_width = roller_r / 12;
  difference() {
    // roller circle
    circle(r=roller_r);
    // passing crescent
    rotate([0, 0, escaping_angle / 2 - 5])
      polygon_arc(
        crescent_end_pt,
        [roller_r, 0],
        radius=roller_r / 2.5
      );
  }
  // need to figure out this rotation to not have it just arbitrary
  rotate([0, 0, escaping_angle / 2]) {
    translate([roller_r / 2 - pallet_width, -pallet_width])
      square([roller_r / 2, pallet_width]);
    translate([roller_r - pallet_width, 0])
      difference() {
        circle(r=pallet_width);
        translate([-roller_r / 4, 0])
          square([roller_r / 2, pallet_width]);
      }
  }
}

module unlocking_roller_profile(unlocking_r, unlocking_dir, pallet_width=1) {
  circle(r=0.9 * unlocking_r / 2);
  a = atan2(unlocking_dir[1], unlocking_dir[0]);
  echo(a);
  rotate([0, 0, a])
  difference() {
    translate([unlocking_r / 2 - pallet_width, 0])
      circle(r=pallet_width);
    translate([0, -unlocking_r])
      square([unlocking_r, unlocking_r]);
  }
}

// ah, the day I finally use the quadratic formula
function compute_pallet_roller_radius(escaping_angle, center_dist, wheel_radius, impulse_angle=1) = 
  let (theta = escaping_angle / 2 - impulse_angle)
  let (A = cos(theta) ^ 2 + sin(theta) ^ 2, B = 2 * center_dist * cos(theta), C = center_dist^2 - wheel_radius^2)
    abs((-B + sqrt(B ^ 2 - 4 * A * C)) / (2 * A));
    

// Watchmaking, Daniels pg. 233
module chronometer_escapement(pitch_diameter, dedendum_depth, wheel_thickness, unlocking_roller_thickness, tooth_count=15, escaping_angle=36, tooth_tip_subtend=0.5) {
  wheel_r = pitch_diameter / 2;
  escaping_angle_hf = escaping_angle / 2;
  tooth_angle = 360 / tooth_count;
  tooth_angle_hf = tooth_angle / 2;
  center_dist = wheel_r * cos(tooth_angle_hf) + wheel_r * sin(tooth_angle_hf) / tan(escaping_angle_hf);
  pallet_roller_angle = escaping_angle_hf - 1;
  impulse_pallet_pitch_r = wheel_r * sin(tooth_angle_hf) / sin(escaping_angle_hf);
  balance_roller_pitch_r = compute_pallet_roller_radius(escaping_angle, center_dist, wheel_r);
  balance_roller_center = [center_dist, 0];
  intersect_pt = rotate_pt([wheel_r, 0], tooth_angle_hf);
  lock_pt = rotate_pt(intersect_pt, tooth_angle);
  // debug_pt(intersect_pt);
  // debug_pt(lock_pt);
  flexure_pt_dist = 1.25 * pitch_diameter;
  flexure_pt_dir = normalized(diff_pts(lock_pt, balance_roller_center));
  flexure_pt = add_pts(balance_roller_center, scalar_mult_pt(flexure_pt_dir, flexure_pt_dist));
  debug_pt(flexure_pt);
  flexure_orthogonal_dir = rot_pt_90(flexure_pt_dir);
  // debug_dir(flexure_pt_dir);
  // debug_dir(flexure_orthogonal_dir);
  locking_stone_face_dir = rotate_pt(flexure_orthogonal_dir, 5);
  // debug_dir(locking_stone_face_dir);
  tooth_locking_face_dir = rotate_pt(locking_stone_face_dir, 5);
  // debug_dir(tooth_locking_face_dir);
  // angle between tooth locking surface and vector from wheel center to lock point
  locking_tooth_radial_angle = vec_angle(lock_pt, rotate_pt(tooth_locking_face_dir, 180));
  // location of the first tooth tip point
  first_tooth_pt = rotate_pt([wheel_r, 0], -tooth_angle_hf);
  // location of the first tooth's dedendum inset point
  first_tooth_inner_point = add_pts([wheel_r, 0], rotate_pt([-dedendum_depth, 0], locking_tooth_radial_angle));
  // aligned point outside the circle so the sector arc is also removed
  outer_circle_point = [wheel_r, rotate_pt([wheel_r, 0], tooth_angle)[1]];
  // location of the first tooth's inner tip point with some thickness
  outer_tooth_tip_pt = rotate_pt([wheel_r, 0], tooth_angle - tooth_tip_subtend);
  // location where dedendum intersects the x axis, so we have a flash region
  dedendum_x_axis_intersection = [first_tooth_inner_point[0], 0];
  // length of the chord of the impulse crescent
  crescent_chord_len = 0.8 * 2 * wheel_r * sin(tooth_angle);
  roller_start_offset_angle = 5;
  flexure_dir = normalized(diff_pts(flexure_pt, balance_roller_center));
  flexure_dir_angle = atan2(flexure_dir[1], flexure_dir[0]);
  unlocking_drop_tip_pt = rotate_pt([balance_roller_pitch_r / 2, 0], flexure_dir_angle);
  unlocking_tip_pt = rotate_pt(unlocking_drop_tip_pt, -1);
  unlocking_tip_dir = normalized(unlocking_tip_pt);
  rotate([0, 0, 0])
  //mirror([1, 0, 0])
    linear_extrude(wheel_thickness)
      escape_wheel_profile(
        wheel_r, 
        tooth_angle, 
        first_tooth_inner_point, 
        outer_circle_point, 
        outer_tooth_tip_pt, 
        dedendum_x_axis_intersection, 
        impulse_pallet_pitch_r
      );
  translate([center_dist, 0])
  //mirror([0, 1, 0])
  rotate([0, 0, 180])
  linear_extrude(wheel_thickness)
    impulse_roller_profile(
      balance_roller_pitch_r,
      crescent_chord_len,
      escaping_angle
    );
  translate([center_dist, 0, wheel_thickness])
    linear_extrude(unlocking_roller_thickness)
      unlocking_roller_profile(balance_roller_pitch_r, unlocking_tip_dir);
}

module test_chronometer_escapement() {
  $fs = 0.1;
  $fa = 0.1;
  chronometer_escapement(
    pitch_diameter=50,
    dedendum_depth=5,
    wheel_thickness=4,
    unlocking_roller_thickness=4
  );
}

module impulse_wheel_profile(pitch_diameter) {
  
}

test_chronometer_escapement();
