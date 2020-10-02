lock throttle to 1.
stage.
wait until stage:ready.
stage.

print("Height: " + find_height()).

// until stable orbit is reached
until ship:apoapsis > 100000 {
  lock targetPitch to 7.4598E-9 * alt:radar^2 - 0.0011313 * alt:radar + 90.31.
  set targetDirection to 90.
  lock steering to heading(targetDirection, targetPitch).

  until ship:apoapsis > 70000 {
    stager().
  }
}

// shut down everything
lock throttle to 0.
sas on.
wait 1.
set sasmode to "prograde".
// lock steering to prograde.

print("Begin maneuver node process").
// begin maneuver node process with circularization burn
local currEcc is list(time:seconds + 30, 0, 0, 0).
local circEcc is currEcc.
until stepper(currEcc) <= circEcc {
  local circEcc is get_eccentricity(list(time:seconds + 30, 0, 0, 0)).
  set circEcc to stepper(currEcc).
  print("Eccentricity Value: " + circEcc).
}

create_maneuver_node(circEcc).

// stages fuel tanks when empty
function stager {
  if not(defined availThrust) {
    global availThrust is ship:availablethrust.
  }

  if ship:availablethrust < (availThrust - 10) {
    print("Current thrust is: " + availThrust).
    print("STAGE!").
    stage.
    wait until stage:ready.
    stage.
    global availThrust is ship:availablethrust.
  }
}

function find_height {
  list parts in partList.
  set lp to 0.
  set hp to 0.

  for p in partList {
    set cp to facing:vector * p:position.
    if cp < lp
      set lp to cp.
    else if cp > hp
      set hp to cp.
  }

  set height to hp - lp.
  return height.
}

// calculates the eccentricity from a given maneuver based on burn rates
function get_ecc {
  parameter nums.
  local maneuver is node(nums[0], nums[1], nums[2], nums[3]).
  print(maneuver).
  add maneuver.
  local ecc is maneuver:orbit:eccentricity.
  remove maneuver.
  return ecc.
}

function create_maneuver_node {
  parameter curr_time, radial, normal, prograde.
  local maneuver is node(curr_time, radial, normal, prograde).

  if radial = 0 and normal = 0 and prograde = 0 {
    print("Don't target Kerbin").
    return.
  }

  // retrieves orbit's eccentricity
  add maneuver:orbit:eccentricity.
  local startTime is calculateStartTime(maneuver).
  warpto(startTime - 10).
  wait until time:seconds > startTime - 10.
  lock steering to maneuver:burnvector.

  wait until time:seconds > startTime.
  lock throttle to 1.

  until isManeuverComplete(maneuver) {
    stager().
  }
}

// steps through and finds possible values for finding point of interception with Mun
function stepper {
  parameter data.
  local step is 100.
  local max is data.
  local dp is list().
  local total is get_ecc(data).
  local index to 0.

  until index >= data:length {
    local incIndex is data:copy().
    local decIndex is data:copy().
    set incIndex[index] to incIndex[index] + step.
    set decIndex[index] to decIndex[index] - step.
    dp:add(incIndex).
    dp:add(decIndex).
    set index to index + 1.
  }

  for item in dp {
    local sum is get_ecc(dp).
    set total to max(sum, total).

    if sum < total {
      set max to dp.
    }
  }

  return dp.
}

// UTILITY FUNCTIONS

// checks whether a maneuver is complete
function isManeuverComplete {
  parameter maneuver.
  if not(defined originalVector) or originalVector = -1 {
    declare global originalVector to maneuver:burnvector.
  }
  if vang(originalVector, maneuver:burnvector) > 90 {
    declare global originalVector to -1.
    return true.
  }
  return false.
}

// calculate time of dV.
function get_maneuver_burn {
  parameter maneuver.
  local g0 is 9.8.
  local isp is 0.
  local dV is maneuver:deltaV:mag.

  // dV = isp * g0 * ln(m0 / mf)
  local mf is m0 / e^(dV / (isp * g0)).
  local fuel is ship:maxthrust / (isp * g0).
  local t is (ship:mass - mf) / fuel.

  return t.
}


function clamp {
    parameter x0.
    parameter x.
    parameter x1.
    return min(max(x0, x), x1).
}

function land {
  global g is 0.5.
  stage.
  set ship:control:pilotmainthrottle to 0.0.

  wait until alt:radar < 7500.

  set ship:control:pilotmainthrottle to 1.0.
  wait until abs(ship:verticalspeed) < 10.
  coast().
  
  print "Begin landing phase.".

  sas off.
  lock steering to up.

  until alt:radar < 5.0 {
    local zero_th is (g*ship:mass)/ship:maxthrust. 

    local alt_tgt is 1.
    local alt_err is alt:radar - alt_tgt.

    local vel_tgt is clamp(-100, -alt_err, 100).
    local vel_err is ship:verticalspeed - vel_tgt. 
    local vel_fix is -(vel_err / 200).

    lock throttle to clamp(0.0, zero_th + vel_fix, 1.0).
    wait 0.1.
  } 
}
