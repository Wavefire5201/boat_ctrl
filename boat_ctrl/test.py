from geopy.distance import geodesic
from geopy.point import Point

def find_midpoint(coord1, coord2):
    # Convert coordinates to Points
    point1 = Point(coord1)
    point2 = Point(coord2)

    # Calculate the midpoint
    midpoint = geodesic().midpoint(point1, point2)

    return midpoint

# Example coordinates (Latitude, Longitude)
coord1 = (40.748817, -73.985428)  # Example: New York City
coord2 = (51.5074, 0.1278)        # Example: London

midpoint = find_midpoint(coord1, coord2)

print(f"Midpoint: {midpoint.latitude}, {midpoint.longitude}")
