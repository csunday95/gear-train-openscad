
function rotate_pt(pt, angle) = [
  pt[0] * cos(angle) - pt[1] * sin(angle), 
  pt[0] * sin(angle) + pt[1] * cos(angle)
];

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

// Watchmaking, Daniels pg. 233
module escape_wheel_profile(diameter, dedendum_depth, tooth_count=15, escaping_angle=36, tooth_tip_subtend=0.25) {
  wheel_r = diameter / 2;
  escaping_angle_hf = escaping_angle / 2;
  tooth_angle = 360 / tooth_count;
  tooth_angle_hf = tooth_angle / 2;
  center_dist = wheel_r * cos(tooth_angle_hf) + wheel_r * sin(tooth_angle_hf) / tan(escaping_angle_hf);
  balance_roller_pitch_r = wheel_r * sin(tooth_angle_hf) / sin(escaping_angle_hf);
  balance_roller_center = [center_dist, 0];
  // intersect_pt = [wheel_r * cos(tooth_angle_hf), wheel_r * sin(tooth_angle_hf)];
  intersect_pt = rotate_pt([wheel_r, 0], tooth_angle_hf);
  lock_pt = rotate_pt(intersect_pt, tooth_angle);
  // debug_pt(intersect_pt);
  // debug_pt(lock_pt);
  flexure_pt_dist = 1.25 * diameter;
  flexure_pt_dir = normalized(diff_pts(lock_pt, balance_roller_center));
  flexure_pt = add_pts(balance_roller_center, scalar_mult_pt(flexure_pt_dir, flexure_pt_dist));
  // debug_pt(flexure_pt);
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
  difference() {
    circle(r=wheel_r);
    for(a = [0:tooth_angle:360 - tooth_angle]) {
      rotate([0, 0, a - tooth_angle_hf]) {
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
            step = 0.1
          );
          translate(dedendum_x_axis_intersection)
            mirror([0, 1, 0])
              square([balance_roller_pitch_r / 2, balance_roller_pitch_r / 2]);
        }
      }
    }
  }
}

module test_escape_wheel_profile() {
  $fs = 0.1;
  $fa = 0.1;
  tooth_angle = 360 / 15;
  tooth_angle_hf = tooth_angle / 2;
  rotate([1, 0, 0])
    escape_wheel_profile(
      diameter=50,
      dedendum_depth=5
    );
}

module impulse_wheel_profile() {
  
}

test_escape_wheel_profile();
