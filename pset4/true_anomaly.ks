// Function for calculating the true anomaly angle of a moving planet in an orbit. This can be used to obtain the position of an orbiting planet at a specific time.
// The true anomaly is the angle of the planet's position relative to the periapsis (from the line of the ascending node). The angle moves fastest at periapsis and slowest at apoapsis. 
// Calculate the mean anomaly by taking the square root of the planet's gravitational parameter by the cube of the semi-major axis.
// https://www.youtube.com/watch?v=cf9Jh44kL20

function true_anomaly {
  local e is ship:orbit:eccentricity.
  local r is ship:orbit:position:mag.
  local v is arccos((e * r) / (abs(e) * abs(r))).
  // value of true anomaly in radians
  return v.
}
