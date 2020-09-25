set customsteer to heading(90, 90).
lock steering to customsteer.
lock throttle to 1.0.
stage.
stage.

function clamp {
    parameter x0.
    parameter x.
    parameter x1.
    return min(max(x0, x), x1).
}

list engines in engs.
sas off.
set n to node(time:seconds+200, 0, 50, 10).
add n.
set nd to nextnode.
set dv0 to nd:deltav.
set max_acc to ship:maxthrust/ship:mass.
set burn_duration to nd:deltav:mag/max_acc.
set np to nd:deltav.

local isStaged is false.
local isLiquidStaged is false.

until ship:apoapsis > 100000 {
    // print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).
    set customsteer to up + r(0, 0, 180).
    // print round(ship:apoapsis, 0) at (0, 16).
    set targetPitch to max( 5, 90 * (1 - ALT:RADAR / 50000)).     
    lock steering to heading (90, targetPitch).

    for eng in engs {
        if eng:maxthrust = 0 and stage:solidfuel < 1 and isStaged = false {
	    stage.
	    print("HELLO").
	    set isStaged to true.
        }

	if eng:maxthrust = 0 and stage:liquidfuel < 1 and isLiquidStaged = false {
	    stage.
	    set isLiquidStaged to true.
	}
    }

    if ship:altitude > 29000 {
//	lock throttle to clamp(0.0, zero_th + vel_fix, 1.0).
	engs[0]:activate().
    } else if ship:altitude > 10000 {
	set customsteer to 90 * sin(90)/10000 * ship:velocity:surface:mag.
    } else {
	lock steering to heading (90, targetPitch).
	lock throttle to 1.0.
    }

    set max_acc to ship:maxthrust/ship:mass.
    set tset to min(nd:deltav:mag/max_acc, 1).

    if vdot(dv0, nd:deltav) < 0 or ship:altitude > 40000 {
        print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
	set customsteer to heading (90, 180).
	engs[0]:activate().
	break.
    }

    if nd:deltav:mag < 0.1 {
        print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
        wait until vdot(dv0, nd:deltav) < 0.5.
        lock throttle to 0.
        print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
    }

    set ship:control:pilotmainthrottle to 0.

    wait 0.1.
}
