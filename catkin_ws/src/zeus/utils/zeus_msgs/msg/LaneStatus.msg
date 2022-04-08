# Fix-sized array, refering to right lane, centre lane and left lane.
std_msgs/Header header
bool[] lanestatus # True for occupied, False for unoccipied
float64[] x_obstacle # the x - distance to the closest obstacle (from base_link) in each lane
bool end_of_challenge
