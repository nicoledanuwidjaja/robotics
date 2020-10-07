lock throttle to 1.
stage.
wait until stage:ready.
launch().
shutup().

function launch {
  print("Begin launch sequence!").
  // until stable orbit is reached
  until ship:apoapsis > 150000 {
    // calculated least-squares best fit for angle trajectory
    // local shipTrajectory is 1.52685E-2 * alt:radar^2 - 0.00169707 * alt:radar + 91.52510.
    lock shipTrajectory to 90.1367 - 1.03287 * alt:radar^0.409511.
    set targetPitch to shipTrajectory.
    set targetDirection to 90.
    lock steering to heading(targetDirection, targetPitch).

    until ship:apoapsis > 70000 {
      stager().
    }
  }
  print("Launch sequence ended.").
}

// shut down everything
function shutup {
  lock throttle to 0.
  wait 1.
  lock steering to prograde.
  beginManeuver().
}

// auto stages when booster is out of fuel
function stager {
  if not(defined availThrust) {
    global availThrust is ship:availablethrust.
  }

  if ship:availablethrust < (availThrust - 10) {
    print("Current thrust is: " + availThrust).
    print("STAGE!").
    wait until stage:ready.
    stage.
    global availThrust is ship:availablethrust.
  }
}

function beginManeuver {
  print("Begin maneuver node process").

  // calclulate circularization prograde burn around Kerbin
  local progradeBurn is getCircularBurn().
  createManeuverNode(list(time:seconds + eta:apoapsis, 0, 0, progradeBurn)).
  // calculate circularization prograde burn around Mun
  local progradeMunBurn is getCircularMunBurn().
  createManeuverNode(list(time:seconds + eta:apoapsis, 0, 0, progradeMunBurn)).
  
  // TODO: calculate circularization prograde burn around Minimus
}

// calculates the eccentricity from a given maneuver based on burn rates
function getEcc {
  parameter nums.
  print(nums).
  // maneuver node: untilTime (secs), radial (m/s), normal (m/s), prograde (m/s)
  local maneuver is node(nums[0], nums[1], nums[2], nums[3]).
  print("DATA: " + maneuver).
  add maneuver.
  local ecc is maneuver:orbit:eccentricity.
  remove maneuver.
  return ecc.
}

function createManeuverNode {
  print("Creating maneuver node.").
  parameter list.
  local untilTime to list[0].
  local radial to list[1].
  local normal to list[2].
  local prograde to list[3].

  local maneuver is node(untilTime, radial, normal, prograde).

  // avoid targeting Kerbin
  if radial = 0 and normal = 0 and prograde = 0 {
    return.
  }

  print("Setting maneuver node...").
  add maneuver.
  local startTime is getStartTime(maneuver).
  warpto(startTime - 10).
  print("WARPING!").

  wait until time:seconds > startTime - 10.
  lock steering to maneuver:burnvector.

  wait until time:seconds > startTime.
  lock throttle to 1.

  until isManeuverComplete(maneuver) {
    stager().
  }
}

////////// UTILITY FUNCTIONS //////////

// obtains angle in radians for position
function getTrueAnomaly {
  local e to ship:orbit:eccentricity.
  local r to ship:orbit:position:mag.
  local v to arccos((e * r) / (abs(e) * abs(r))).
  print("E: " + e).
  print("R: " + r).
  print("true anomaly " + v).
  wait 1.
  return v.
}

// use hohmann transfer to calculate circularization burn for Kerbin
function getCircularBurn {
  local a2 is 150000 + ship:body:radius.
  local aTrans is ship:orbit:semimajoraxis.
  local kerbin_g is 3.53160E12.

  // rocket's calculated specific mechanical energy
  local sme2 is -(kerbin_g / (2 * aTrans)).
  local sme_trans is -(kerbin_g / (2 * a2)).
  // body radius
  local pos_mag2 is a2.
  local v2 is sqrt(2 * ((kerbin_g / pos_mag2) + sme2)).
  local v_trans is sqrt(2 * ((kerbin_g / pos_mag2) + sme_trans)).
  local dv is abs(v2 - v_trans).
  return dv.
}

// use hohmann transfer to calculate circularization burn for Mun
function getCircularMunBurn {
  local kerbin_g is 3.53160E12.
  local mun_g is 6.5138398E10.

  local a1 is 150000 + ship:body:radius.
  local a2 is a1 + ship:orbit:body:altitude.
  
  // rocket's calculated specific mechanical energy
  local sme_trans is -(mun_g / (2 * a2)).
  local sme1 is -(kerbin_g / (2 * a1)).

  local aTrans is ship:orbit:semimajoraxis.
  local sme2 is -(mun_g / (2 * aTrans)).

  // body radius
  local pos_mag2 is a2.
  local v2 is sqrt(2 * ((kerbin_g / pos_mag2) + sme2)).
  local v_trans is sqrt(2 * ((kerbin_g / pos_mag2) + sme_trans)).
  local v1 is sqrt(2 * ((kerbin_g / pos_mag2) + sme1)).
  print("v trans " + v_trans).
  print("v2 " + v2).
  local dv is v1 + v2.
  print("DV: " + dv).
  return dv.
}

// functions provided by CheersKevin
// function executeManeuver {
//   parameter mList.
//   local maneuver is node(mList[0], mList[1], mList[2], mList[3]).
//   add maneuver.
//   local startTime is getStartTime(maneuver).
//   wait until time:seconds > startTime - 10.
//   lock steering to maneuver:burnvector.
//   wait until time:seconds > startTime.
//   lock throttle to 1.
//   until isManeuverComplete(maneuver) {
//     stager().
//   }
//   lock throttle to 0.
//   unlock steering.
//   remove maneuver.
// }

// calculates the start time for the maneuver node
function getStartTime {
  parameter node.
  local maneuverBurn to 0.
  local dV is node:deltaV:mag.
  local g0 is 9.80665.
  local isp is 0.

  list engines in myEngines.
  for en in myEngines {
    if en:ignition and not en:flameout {
      set isp to isp + (en:isp * (en:availableThrust / ship:availableThrust)).
    }
  }

  // calculate with Tsiolkovsky ideal rocket equation
  local mf is ship:mass / constant():e^(dV / (isp * g0)).
  local fuelFlow is ship:availableThrust / (isp * g0).
  local t is (ship:mass - mf) / fuelFlow.

  set maneuverBurn to t.

  return time:seconds + node:eta - maneuverBurn / 2.
}

function isManeuverComplete {
  parameter mnv.
  if not(defined originalVector) or originalVector = -1 {
    declare global originalVector to mnv:burnvector.
  }
  if vang(originalVector, mnv:burnvector) > 90 {
    declare global originalVector to -1.
    return true.
  }
  return false.
}

// // calculates the start time of the maneuver node
// function getStartTime {
//   parameter maneuver.

//   local dV is maneuver:deltaV:mag.
//   local g0 is 9.80665.
//   local isp is 0.

//   list engines in myEngines.
//   for en in myEngines {
//     if en:ignition and not en:flameout {
//       set isp to isp + (en:isp * (en:availableThrust / ship:availableThrust)).
//     }
//   }

//   local mf is ship:mass / constant():e^(dV / (isp * g0)).
//   local fuelFlow is ship:availableThrust / (isp * g0).
// }


// // checks whether a maneuver is complete
// function isManeuverComplete {
//   parameter maneuver.
//   if not(defined originalVector) or originalVector = -1 {
//     declare global originalVector to maneuver:burnvector.
//   }
//   if vang(originalVector, maneuver:burnvector) > 90 {
//     declare global originalVector to -1.
//     return true.
//   }
//   return false.
// }

// // calculate time of dV for the node function
// function getManeuverBurn {
//   parameter maneuver.
//   local g0 is 9.8.
//   local isp is 0.
//   local dV is maneuver:deltaV:mag.

//   // dV = isp * g0 * ln(m0 / mf)
//   local mf is m0 / e^(dV / (isp * g0)).
//   local fuel is ship:maxthrust / (isp * g0).
//   local t is (ship:mass - mf) / fuel.

//   return t.
// }
