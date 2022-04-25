
include <openscad-util/rounded_bars.scad>
include <openscad-util/cycloid_profiles.scad>
include <openscad-util/revolve_text.scad>

include <temp-chronometer-escapement-caddy.scad>

function rotate_pt(pt, angle) = [
  pt[0] * cos(angle) - pt[1] * sin(angle), 
  pt[0] * sin(angle) + pt[1] * cos(angle)
];

function rotate_pt_about(pt, center, angle) = rotate_pt(pt - center, angle) + center;

function rot_pt_90(pt) = [
  -pt[1], pt[0]
];

function normalized(pt) = (
  pt / norm(pt)
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
  v12 = rot_pt_90(normalized(p2 - p1));
  v23 = rot_pt_90(normalized(p3 - p2));
  dir_ratio_23 = v23[1] / v23[0];
  t_intersect = (dir_ratio_23 * (mp23[0] - mp12[0]) + mp12[1] - mp23[1]) / (v12[0] * dir_ratio_23 - v12[1]);
  center = mp12 + v12 * t_intersect;
  radius = norm(p1 - center);
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
  point_dist = norm(p2 - p1);
  orthogonal_dir = rot_pt_90(normalized(p2 - p1));
  // angle of the sector of the circle spanning p1 to p2
  sector_angle = 2 * asin(point_dist / 2 / radius);
  center = mp12 + orthogonal_dir * radius * cos(sector_angle / 2);
  // debug_pt(center);
  p1_rel_dir = p1 - center;
  p1_x_angle = atan2(p1_rel_dir[1], p1_rel_dir[0]);
  circle_func = function(r, theta) [r * cos(theta), r * sin(theta)];
  points = [for (a = concat([for (a =[0:step:sector_angle]) a], sector_angle)) circle_func(radius, a)];
  points_w_center = concat(points, [[0, 0]]);
  translate(center)
    rotate([0, 0, p1_x_angle])
      polygon(points_w_center);
}

module escape_wheel_profile(wheel_r, tooth_angle, first_tooth_inner_point, outer_circle_point, 
                            outer_tooth_tip_pt, dedendum_x_axis_intersection, balance_roller_pitch_r) {
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

module escape_wheel(wheel_thickness, wheel_r, tooth_angle, first_tooth_inner_point, outer_circle_point, 
                    outer_tooth_tip_pt, dedendum_x_axis_intersection, impulse_pallet_pitch_r, 
                    train_shaft_diameter, shaft_notch_size) {
  linear_extrude(wheel_thickness, convexity=3)
    difference() {
      escape_wheel_profile(
        wheel_r, 
        tooth_angle, 
        first_tooth_inner_point, 
        outer_circle_point, 
        outer_tooth_tip_pt, 
        dedendum_x_axis_intersection, 
        impulse_pallet_pitch_r
      );
      circle(r=train_shaft_diameter / 2);
      translate([train_shaft_diameter / 2, 0])
        square([shaft_notch_size * 2, shaft_notch_size], center=true);
    }
}

module impulse_roller_profile(roller_r, crescent_chord_len, escaping_angle, shaft_radius, 
                              notch_size, roller_radius_frac=0.95) {
  crescent_subtend_angle = asin(crescent_chord_len / 2 / roller_r);
  crescent_end_pt = rotate_pt([roller_r, 0], crescent_subtend_angle);
  // figure out what this is actually supposed to be
  pallet_width = roller_r / 12;
  difference() {
    // roller circle, only do 90% on the roller to give a little extra laggin tooth clearance
    circle(r=roller_r * roller_radius_frac);
    // passing crescent
    rotate([0, 0, escaping_angle / 2 - 5])
      polygon_arc(
        crescent_end_pt,
        [roller_r, 0],
        radius=roller_r / 2.5
      );
    circle(r=shaft_radius);
    translate([-shaft_radius, 0, 0])
      square([notch_size * 2, notch_size], center=true);
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

module unlocking_roller_profile(unlocking_r, unlocking_dir, shaft_radius, notch_size, pallet_width=1) {
  difference() {
    circle(r=0.95 * unlocking_r / 2);
    circle(r=shaft_radius);
    translate([shaft_radius, 0, 0])
      square([notch_size * 2, notch_size], center=true);
  }
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
// this is basically finding the point where the escaping angle / 2 relative to the
// wheel center lines minus one intersects with the pitch circle of the escaping wheel
function compute_pallet_roller_radius(escaping_angle, center_dist, wheel_radius, impulse_angle=1) = 
  let (theta = escaping_angle / 2 - impulse_angle)
  let (A = cos(theta) ^ 2 + sin(theta) ^ 2, B = 2 * center_dist * cos(theta), C = center_dist^2 - wheel_radius^2)
    abs((-B + sqrt(B ^ 2 - 4 * A * C)) / (2 * A));
    
function lines_intersect(A, Va, B, Vb) =
  let (R = Vb[1] / Vb[0])
    let (t = (R *  (B[0] - A[0]) + A[1] - B[1]) / (R * Va[0] - Va[1]))
      A + t * Va;
      
function point2point_abs_distance(start, end, dist) = start + dist * (end - start) / norm(end - start);

    
module detent_arm(flexure_pt, passing_spring_end_pt, detent_rotation, arm_width, arm_thickness, detent_radius, detent_center, detent_edge, pivot_shaft_radius, arm_spring_free_lenth=1, passing_spring_thickness=0.75) {
  main_arm_length = norm(passing_spring_end_pt - flexure_pt) - arm_spring_free_lenth;
  main_arm_endpoint = point2point_abs_distance(flexure_pt, passing_spring_end_pt, main_arm_length);
  arm_orthogonal_dir = rot_pt_90(normalized(passing_spring_end_pt - flexure_pt));
  arm_backing_target_pt = passing_spring_end_pt + arm_orthogonal_dir * passing_spring_thickness;
  // detent base
  translate(detent_center)
    cylinder(r=detent_radius, h=arm_thickness);
  // detent arm
  difference() {
    union() {
      bar_point2_point(
        flexure_pt, 
        main_arm_endpoint, 
        arm_width, 
        arm_thickness, 
        intrinsic_translation=[-arm_width / 4, 0]
      );
      translate(flexure_pt)
        cylinder(r=pivot_shaft_radius * 2, h=arm_thickness);
    }
    translate(flexure_pt)
      cylinder(r=pivot_shaft_radius, h=arm_thickness * 3, center=true);
    translate([0, 0, -arm_thickness / 3])
      linear_extrude(arm_thickness * 1.5)
        polygon_arc(
          p1=arm_backing_target_pt,
          p2=detent_edge,
          radius=main_arm_length
        );
    translate(concat(midpoint(detent_center, main_arm_endpoint), arm_thickness))
      rotate([0, 90, 50])
        cylinder(r=0.75, h=arm_thickness * 2, center=true);
    translate(concat(flexure_pt, arm_thickness))
      rotate([0, 90, 50])
        cylinder(r=0.75, h=pivot_shaft_radius * 8, center=true);
    
  }
  // detent semicircle
  translate([detent_center[0], detent_center[1], -arm_thickness])
    rotate([0, 0, -detent_rotation])
      linear_extrude(arm_thickness)
        difference() {
          circle(r = detent_radius);
          translate([-detent_radius, 0])
          square([detent_radius * 2, detent_radius * 2]);
        }
  // passing spring
  bar_point2_point(
    detent_edge, 
    passing_spring_end_pt, 
    passing_spring_thickness, 
    arm_thickness,
    [-passing_spring_thickness / 2, 0]
  );
}

// Watchmaking, Daniels pg. 233
module chronometer_escapement_assembled(pitch_diameter, dedendum_depth, wheel_thickness,
      unlocking_roller_thickness, train_shaft_diameter, shaft_notch_size, detent_arm_width, 
      pinion_pitch_dia, pinion_leaf_ct, pinion_dedendum_depth, pinion_thickness,
      tooth_count=15, escaping_angle=36, tooth_tip_subtend=0.5) {
  // pitch radius of escapement wheel
  wheel_r = pitch_diameter / 2;
  // half the escaping angle
  escaping_angle_hf = escaping_angle / 2;
  // angle subtended by each tooth / dedendum
  tooth_angle = 360 / tooth_count;
  // half the tooth angle, the angle subtended by just the active tooth part
  tooth_angle_hf = tooth_angle / 2;
  // the distance between the wheel center and the balance roller center
  center_dist = wheel_r * cos(tooth_angle_hf) + wheel_r * sin(tooth_angle_hf) / tan(escaping_angle_hf);
  // angle the pallet roller makes relative to the line between the two wheel centers
  pallet_roller_angle = escaping_angle_hf - 1;
  // pitch radius of the impulse pallet tip
  impulse_pallet_pitch_r = wheel_r * sin(tooth_angle_hf) / sin(escaping_angle_hf);
  // pitch radius of the impulse roller
  balance_roller_pitch_r = compute_pallet_roller_radius(escaping_angle, center_dist, wheel_r);
  // center of the balance roller, just the center distance along the x axis
  balance_roller_center = [center_dist, 0];
  echo(balance_roller_center);
  // point where the lagging tooth tip will be at the time of lock
  intersect_pt = rotate_pt([wheel_r, 0], tooth_angle_hf);
  // location of the locked tooth tip at time of lock
  lock_pt = rotate_pt(intersect_pt, tooth_angle);
  // flexure point is one and one quarter the pitch diameter distance
  // along the direction of the roller center -> flexure point ray
  flexure_pt_dist = 1.25 * pitch_diameter;
  flexure_pt_dir = normalized(lock_pt - balance_roller_center);
  flexure_pt = balance_roller_center + flexure_pt_dir * flexure_pt_dist;
  echo(flexure_pt);
  // direction relative to the flexure point to which the locking stone must be tangent
  locking_limit_dir = rotate_pt(flexure_pt_dir, -1);
  // orthogonal to roller center -> flexure point direction
  flexure_orthogonal_dir = rot_pt_90(flexure_pt_dir);
  // locking stone face is 5 degrees off of orthogonal to the flexure direction
  locking_stone_face_dir = rotate_pt(flexure_orthogonal_dir, 5);
  // locking stone should be one eighteenth the escaping wheel radius
  locking_stone_radius = wheel_r / 18;
  // the leading edge point of the locking stone; this is computed as the intersection of 
  // the flexure point + t * locking limit direction line and the 
  // lock point + u * locking stone facing direction line
  locking_limit_pt = lines_intersect(flexure_pt, locking_limit_dir, lock_pt, locking_stone_face_dir);
  // center must be locking stone radius distance from the locking stone tangent point in the
  // direction of the stone facing direction
  locking_stone_center = locking_limit_pt - locking_stone_radius * locking_stone_face_dir;
  // the angle the tooth cuts in relative to an escaping wheel radius
  tooth_locking_face_dir = rotate_pt(flexure_orthogonal_dir, 10);
  // the angle the locking stone forms relative to the escaping wheel center <-> roller center line
  locking_stone_radial_angle = -atan2(locking_stone_face_dir[1], locking_stone_face_dir[0]);
  // angle between tooth locking surface and vector from wheel center to lock point
  locking_tooth_radial_angle = vec_angle(lock_pt, rotate_pt(tooth_locking_face_dir, 180));
  // location of the first tooth tip point
  first_tooth_pt = rotate_pt([wheel_r, 0], -tooth_angle_hf);
  // location of the first tooth's dedendum inset point
  first_tooth_inner_point = [wheel_r, 0] + rotate_pt([-dedendum_depth, 0], locking_tooth_radial_angle);
  // aligned point outside the circle so the sector arc is also removed
  outer_circle_point = [wheel_r, rotate_pt([wheel_r, 0], tooth_angle)[1]];
  // location of the first tooth's inner tip point with some thickness
  outer_tooth_tip_pt = rotate_pt([wheel_r, 0], tooth_angle - tooth_tip_subtend);
  // location where dedendum intersects the x axis, so we have a flash region
  dedendum_x_axis_intersection = [first_tooth_inner_point[0], 0];
  // length of the chord of the impulse crescent
  crescent_chord_len = 0.9 * 2 * wheel_r * sin(tooth_angle);
  // should be the same as flexure_pt_dir, but more explicitly the
  // direction of the balance roller center -> flexure point vector
  flexure_dir = normalized(flexure_pt - balance_roller_center);
  // the angle the flexure direction forms relative to the wheel center to center line
  flexure_dir_angle = atan2(flexure_dir[1], flexure_dir[0]);
  // the point on the balance roller edge that intersects the above direction;
  // this is the location of the unlocking pallet tip at the drop point
  unlocking_drop_tip_pt = rotate_pt([balance_roller_pitch_r / 2, 0], flexure_dir_angle);
//  # linear_extrude(20)
//    debug_pt(unlocking_drop_tip_pt + balance_roller_center);
  // 
  unlocking_tip_pt = rotate_pt(unlocking_drop_tip_pt, -1);
  unlocking_tip_dir = normalized(unlocking_tip_pt);
  unlocking_tip_pt_absolute = unlocking_tip_pt + balance_roller_center;
  // rotate unlocking pallete tip about flexure pt by 1 degree,
  // this gives the direction of the spring release point relative to the 
  // roller center 
  spring_release_tip_pt_flexure_relative = rotate_pt_about(unlocking_tip_pt_absolute, flexure_pt, 1);
  // compute the roller center relative direction
  spring_release_tip_dir = normalized(spring_release_tip_pt_flexure_relative - balance_roller_center);
  // compute the point of passing spring release as the pallete distance along the tip point direction
  spring_release_tip_absolute = balance_roller_center + balance_roller_pitch_r / 2 * spring_release_tip_dir;
  // distance between the spring release point and the flexure point
  spring_release_tip_dist = norm(spring_release_tip_absolute - flexure_pt);
  // direction of unlocking tip <-> flexure_pt vector
  unlocking_tip_flexure_pt_dir = normalized(unlocking_tip_pt_absolute - flexure_pt);
  // tip of the unlocking spring at the moment of unlocking resides at the distance between the spring
  // release point and the flexure point along the flexure pt unlocking tip direction
  spring_release_tip_at_unlock = flexure_pt + unlocking_tip_flexure_pt_dir * spring_release_tip_dist;
  text_depth = 0.4;
  // escaping wheel
//  difference() {
//    escape_wheel(
//      wheel_thickness, 
//      wheel_r, 
//      tooth_angle, 
//      first_tooth_inner_point, 
//      outer_circle_point, 
//      outer_tooth_tip_pt, 
//      dedendum_x_axis_intersection, 
//      impulse_pallet_pitch_r,
//      train_shaft_diameter,
//      shaft_notch_size
//    );
//    for (a = [0:360/3:270]) {
//      rotate([0, 0, a - 3])
//      translate([0, wheel_r/2, 0])
//        rotate([90, 0, 0])
//          cylinder(r=0.75, h=wheel_r, center=true);
//    }
//    translate([0, 0, wheel_thickness - text_depth])
//      revolve_text(
//        radius=wheel_r / 2,
//        chars=str("Earnshaw & Arnold - 1783"),
//        font_size=4.5,
//        thickness=text_depth * 2,
//        arc_fraction=0.6
//      );
//  }
  // impulse roller segment
//  translate([center_dist, 0])
//    difference() {
//      rotate([0, 0, 180])
//        linear_extrude(wheel_thickness, convexity=3)
//          impulse_roller_profile(
//            balance_roller_pitch_r,
//            crescent_chord_len,
//            escaping_angle,
//            train_shaft_diameter / 2,
//            shaft_notch_size
//          );
//      for (a = [0:360/3:270]) {
//        rotate([0, 0, a - 3])
//        translate([0, wheel_r/2, 0])
//          rotate([90, 0, 0])
//            cylinder(r=0.75, h=wheel_r, center=true);
//      }
//    }
//  // unlocking roller segment
//  translate([center_dist, 0, wheel_thickness])
//    linear_extrude(unlocking_roller_thickness)
//      unlocking_roller_profile(
//        balance_roller_pitch_r, 
//        unlocking_tip_dir, 
//        train_shaft_diameter / 2,
//        shaft_notch_size
//      );
  // detent arm
//  translate([0, 0, wheel_thickness + 1])
//    detent_arm(
//      flexure_pt=flexure_pt, 
//      passing_spring_end_pt=spring_release_tip_at_unlock,
//      detent_rotation=locking_stone_radial_angle,
//      arm_width=detent_arm_width, 
//      arm_thickness=detent_arm_width, 
//      detent_radius=locking_stone_radius,
//      detent_center=locking_stone_center, 
//      detent_edge=locking_limit_pt, 
//      pivot_shaft_radius=train_shaft_diameter
//    );
  // wheel pinion
// translate([0, 0, wheel_thickness])
//  linear_extrude(pinion_thickness)
//    difference() {
//      pinion_profile(
//        pitch_diameter=pinion_pitch_dia,
//        leaf_count=pinion_leaf_ct,
//        dedendum_depth=pinion_dedendum_depth
//      );
//      circle(r=train_shaft_diameter / 2);
//      translate([train_shaft_diameter / 2, 0, 0])
//        square([shaft_notch_size * 2, shaft_notch_size], center=true);
//    }
}

module test_chronometer_escapement() {
  $fs = 0.02;
  $fa = 0.02;
  chronometer_escapement_assembled(
    pitch_diameter=75,
    dedendum_depth=5,
    wheel_thickness=8,
    unlocking_roller_thickness=8,
    train_shaft_diameter=2.5,
    shaft_notch_size=0.5,
    detent_arm_width=5,
    pinion_pitch_dia=8,
    pinion_leaf_ct=8,
    pinion_dedendum_depth=1.6,
    pinion_thickness=6
  );
}

test_chronometer_escapement();
