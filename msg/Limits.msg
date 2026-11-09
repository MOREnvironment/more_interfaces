# Operating limits the vehicle enforces. Published latched, and again when they change.
# Frames and units: depth and altitude in m, speeds in m/s, region vertices WGS-84 (degrees).

std_msgs/Header header

# Maximum depth below the surface, m, positive down. NaN: none.
float64 max_depth

# Minimum altitude above the seabed, m. NaN: none.
float64 min_altitude

# Speed limits through the water, m/s. NaN: none.
float64 max_speed
float64 min_speed

# Closed polygon the vehicle must stay inside (the last vertex joins the first; the altitude field is ignored).
# Fewer than three vertices: no region.
geographic_msgs/GeoPoint[] region
