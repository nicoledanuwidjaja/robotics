lock throttle to 1.
sas off.
stage.
wait until stage:ready.
launch().
shutup().

function launch {
  print("Begin launch sequence!").
  // until stable orbit is reached
  until ship:apoapsis > 100000 {
    // calculated least-squares best fit for angle trajectory
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

// initiates maneuvers around various moons
function beginManeuver {
  print("Begin maneuver node process").

  // calclulate circularization prograde burn around Kerbin
  local progradeBurn is getCircularBurn().
  print("Circularize Kerbin").
  createManeuverNode(list(time:seconds + eta:apoapsis, 0, 0, progradeBurn)).
  // calculate circularization prograde burn around Mun
  wait 5.
  local progradeMunBurn is getCircularMunBurn().
  print("Mun Burn: " + progradeMunBurn).
  print("Circularize Mun").
  local munTime is eta_true_anom(0).
  print("Mun Time: " + time:seconds + munTime).
  createManeuverNode(list(time:seconds + munTime, 0, 0, progradeMunBurn)).
  local progradeMinmusBurn is getCircularMinmusBurn().
  print("Circularize Minmus").
  local minmusTime is eta_true_anom(180).
  createManeuverNode(list(time:seconds + minmusTime, 0, 0, progradeMinmusBurn)).
}

// calculates the eccentricity from a given maneuver based on burn rates
function getEcc {
  parameter nums.
  print(nums).
  // maneuver node: untilTime (secs), radial (m/s), normal (m/s), prograde (m/s)
  local maneuver is node(nums[0], nums[1], nums[2], nums[3]).
  add maneuver.
  local ecc is maneuver:orbit:eccentricity.
  remove maneuver.
  return ecc.
}

// creates and calculates node properties
function createManeuverNode {
  print("Creating maneuver node.").
  parameter list.

  local untilTime to list[0].
  local radial to list[1].
  local normal to list[2].
  local prograde to list[3].
  local maneuver is node(untilTime, radial, normal, prograde).

  addManeuver(maneuver).
  local startTime is getStartTime(maneuver).
  wait until time:seconds > startTime - 10.
  lockSteering(maneuver).
  wait until time:seconds > startTime.
  lock throttle to 1.

  until isManeuverComplete(maneuver) {
    stager().
  }

  lock throttle to 0.
  unlock steering.
  removeManeuver(maneuver).
}

// locks steering to aim towards maneuver target
function lockSteering {
  parameter maneuver.
  lock steering to maneuver:burnvector.
}

// removes maneuver node from flight
function removeManeuver {
  parameter maneuver.
  remove maneuver.
}

// adds maneuver node to flight
function addManeuver {
  parameter maneuver.
  add maneuver.
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
  local a2 is 100000 + ship:body:radius.
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
  local a1 is 100000 + kerbin:radius.
  local aTrans is (a1 + mun:body:radius + mun:altitude) / 2.
  
  // rocket's calculated specific mechanical energy
  local smeTrans is -(ship:body:mu / (2 * aTrans)).
  local sme1 is -(ship:body:mu / (2 * a1)).

  // body radius
  local pos_mag1 is a1.
  local v_trans is sqrt(2 * ((kerbin:mu / pos_mag1) + smeTrans)).
  local v1 is sqrt(2 * ((kerbin:mu / pos_mag1) + sme1)).
  local dv is abs(v_trans - v1).
  print("DV: " + dv).
  return dv.
}

// use hohmann transfer to calculate circularization burn for Minmus
function getCircularMinmusBurn {
  local org is 100000 + kerbin:radius.
  local a1 is (org + minmus:body:radius).
  local aTrans is (a1 + minmus:body:radius + minmus:altitude) / 2.

  // body radius
  local pos_mag1 is aTrans.

  // rocket's calculated specific mechanical energy
  local smeTrans is -(ship:body:mu / (2 * aTrans)).
  local sme1 is -(ship:body:mu / (2 * a1)).
  local v_trans is sqrt(2 * ((kerbin:mu / pos_mag1) + smeTrans)).
  local v1 is sqrt(2 * ((kerbin:mu / pos_mag1) + sme1)).
  local dv is abs(v_trans - v1).
  print("DV: " + dv).
  return dv.
}

// FUNCTIONS ATTRIBUTED TO CHEERSKEVIN
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
      // calculate remaining impulse
      set isp to isp + (en:isp * (en:availableThrust / ship:availableThrust)).
    }
  }

  // Tsiolkovsky ideal rocket equation
  local mf is ship:mass / constant():e^(dV / (isp * g0)).
  local fuelFlow is ship:availableThrust / (isp * g0).
  local t is (ship:mass - mf) / fuelFlow.

  set maneuverBurn to t.

  // calculate time of dV for the node function
  return time:seconds + node:eta - maneuverBurn / 2.
}

function isManeuverComplete {
  parameter maneuver.
  
  if not(defined v1) or v1 = -1 {
    declare global v1 to maneuver:burnvector.
  }

  if vang(v1, maneuver:burnvector) > 90 {
    declare global v1 to -1.
    return true.
  }
  return false.
}

// Attributed to nulib-kos (Nolan Bock)
// calculate the estimated time to reach a body's position
function eta_true_anom {
    declare local parameter tgt_lng.
    // convert the positon from reference to deg from PE (which is the true anomaly)
    local ship_ref to mod(obt:lan+obt:argumentofperiapsis+obt:trueanomaly,360).
    print("Ship ref: " + ship_ref).

    local node_true_anom to (mod (720+ tgt_lng - (obt:lan + obt:argumentofperiapsis),360)).

    local node_eta to 0.
    local ecc to OBT:ECCENTRICITY.
    if ecc < 0.001 {
        set node_eta to SHIP:OBT:PERIOD * ((mod(tgt_lng - ship_ref + 360,360))) / 360.

    } else {
        local eccentric_anomaly to  arccos((ecc + cos(node_true_anom)) / (1 + ecc * cos(node_true_anom))).
        local mean_anom to (eccentric_anomaly - ((180 / (constant():pi)) * (ecc * sin(eccentric_anomaly)))).

        // time from periapsis to point
        local time_2_anom to  SHIP:OBT:PERIOD * mean_anom /360.

        local my_time_in_orbit to ((OBT:MEANANOMALYATEPOCH)*OBT:PERIOD /360).
        set node_eta to mod(OBT:PERIOD + time_2_anom - my_time_in_orbit,OBT:PERIOD) .

    }
    print("Node eta: " + node_eta).
    return node_eta.
}