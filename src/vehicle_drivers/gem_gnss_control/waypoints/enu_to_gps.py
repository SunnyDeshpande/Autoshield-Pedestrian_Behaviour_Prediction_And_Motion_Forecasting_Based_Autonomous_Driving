#!/usr/bin/env python3
import csv
import pymap3d as pm

save_gps_csv = False

origin_lat = 40.0927422
origin_lon = -88.2359639
origin_alt = 0.0

if save_gps_csv:

    track_file = 'track.csv'
    output_file = 'track_gps.csv'

    with open(track_file, newline='') as f:
        reader = csv.reader(f)
        enu_points = [tuple(map(float, row)) for row in reader]

    gps_points = []
    for x, y, yaw in enu_points:
        lat, lon, alt = pm.enu2geodetic(x, y, 0, origin_lat, origin_lon, origin_alt)
        gps_points.append((lat, lon, yaw))

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['lat', 'lon', 'yaw'])
        writer.writerows(gps_points)

    print(f"Saved GPS track to {output_file}")


import pandas as pd
import folium

# (19.61, -0.07), Target: (24.71, 0.07)
current_pos_enu = (19.61, -0.07)
target_pos_enu = (24.71, 0.07)
# (-0.32, 11.87), Target: (2.90, 7.90)
# (0.82, 11.63), Target: (3.18, 7.10),
# (3.49, 10.90), Target: (3.63, 5.83)
# (6.05, 9.45), Target: (4.12, 4.77),
# (7.01, 8.11), Target: (4.89, 3.54),
# (7.38, 5.63), Target: (8.98, 0.86)
# (7.07, 4.01), Target: (10.76, 0.51),
# (6.93, 3.47), Target: (10.96, 0.49),

df = pd.read_csv('track_gps.csv')

m = folium.Map(location=[df['lat'][0], df['lon'][0]], zoom_start=20)

folium.TileLayer(
    tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    attr='Esri',
    name='Esri Satellite',
    overlay=False,
    control=True
).add_to(m)

points = df[['lat', 'lon']].values.tolist()
folium.PolyLine(points, color='blue', weight=3).add_to(m)

if current_pos_enu is not None:
    lat, lon, alt = pm.enu2geodetic(current_pos_enu[0], current_pos_enu[1], 0, origin_lat, origin_lon, origin_alt)
    folium.CircleMarker(location=[lat, lon], radius=6, color='green', fill=True, fill_color='green', fill_opacity=0.8).add_to(m)
    folium.Circle(location=[lat, lon], radius=5, color='green', fill=False).add_to(m)

if target_pos_enu is not None:
    lat, lon, alt = pm.enu2geodetic(target_pos_enu[0], target_pos_enu[1], 0, origin_lat, origin_lon, origin_alt)
    folium.CircleMarker(location=[lat, lon], radius=6, color='red', fill=True, fill_color='red', fill_opacity=0.8).add_to(m)

folium.LayerControl().add_to(m)

m.save('track_map_satellite.html')
print("saved as track_map_satellite.html")