import json
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import numpy as np

import math
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import navpy
import mpu
import time
desired_exploration_time_min=3
speed=10
lat_ref = 34.206616687546656  # Référence latitude en degrés
lon_ref = 108.80471633611563  # Référence longitude en degrés
alt_ref = 0.0  # Référence altitude en mètres
thermal_points_type_1_generated=[] #index: 0
thermal_points_type_2_generated=[] #index: 1
thermal_points_type_3_generated=[] #index: 2
thermal_point_types=[thermal_points_type_1_generated,thermal_points_type_2_generated,thermal_points_type_3_generated]
duration_thermal_type_1_in_min=1
duration_thermal_type_2_in_min=3
duration_thermal_type_3_in_min=5
remaining_time_type_1=[] 
remaining_time_type_2=[] 
remaining_time_type_3=[] 
remaining_time=[remaining_time_type_1,remaining_time_type_2,remaining_time_type_3]
mission_points=[]
thermal_in_circle_type_1=[]
thermal_in_circle_type_2=[]
thermal_in_circle_type_3=[]
thermal_in_circle=[]
grid_size=200
#remaining_time = [0] * 10
async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    termination_task = asyncio.create_task(termination(drone))
    generate_thermals1 = asyncio.create_task(generate_thermals(grid_size,1,0))  
    track_thermals1 = asyncio.create_task(track_thermals(0,duration_thermal_type_1_in_min)) 
    generate_thermals2 = asyncio.create_task(generate_thermals(grid_size,3,1))  
    track_thermals2 = asyncio.create_task(track_thermals(1,duration_thermal_type_2_in_min))  
    generate_thermals3 = asyncio.create_task(generate_thermals(grid_size,5,2)) 
    track_thermals3 = asyncio.create_task(track_thermals(2,duration_thermal_type_3_in_min)) 
    update_points = asyncio.create_task(update_thermal_points())
    mission_tracker = asyncio.create_task(track_mission(drone))
    offboard_switcher = asyncio.create_task(mission_offboard_switcher(drone))
    
    await waypoints(drone)
    print("Arming the drone")
    await drone.action.arm()
    print("starting mission")
    await drone.mission.start_mission() 
    #await termination_task
    await asyncio.gather(termination_task,generate_thermals1,generate_thermals2,generate_thermals3,track_thermals1,track_thermals2,track_thermals3,update_points,offboard_switcher)
def read_plan():
    with open('/home/vudala/ets_research_master/New Folder/mission.plan', 'r') as file:
        data = json.load(file)
        return data  
async def waypoints(drone):
        data=read_plan()
        items = data.get('mission', {}).get('items', [])
        mission_items = []
        
        for item in items:
            if item.get('type') == 'SimpleItem':
                if item.get('command')==22:
                    mission_items.append(MissionItem(
                                item.get('params')[4],      # latitude
                                item.get('params')[5],      # longitude
                                item.get('params')[6],  # altitude
                                10,        # speed (m/s)
                                True,              
                                float('nan'),     
                                float('nan'),     
                                MissionItem.CameraAction.NONE,
                                float('nan'),      
                                float('nan'),     
                                float('nan'),      
                                float('nan'),     
                                float('nan'),     
                                MissionItem.VehicleAction.TAKEOFF  
                                    ))  
                    #print(f"{item.get('params')[4]},{item.get('params')[5]},green,marker")

                if item.get('command')==16:
                    mission_items.append(MissionItem(
                                item.get('params')[4],      # latitude
                                item.get('params')[5],      # longitude
                                item.get('params')[6],  # altitude
                                10,        # speed (m/s)
                                True,              
                                float('nan'),     
                                float('nan'),     
                                MissionItem.CameraAction.NONE,
                                float('nan'),      
                                float('nan'),     
                                float('nan'),      
                                float('nan'),     
                                float('nan'),     
                                MissionItem.VehicleAction.NONE  
                                    ))    
                    #print(f"{item.get('params')[4]},{item.get('params')[5]},red,marker")
                    mission_points.append([item.get('params')[4],item.get('params')[5]])
            elif item.get('type') == 'ComplexItem':
                mission_items.append(MissionItem(
                                item.get('landingApproachCoordinate')[0],      # latitude
                                item.get('landingApproachCoordinate')[1],      # longitude
                                item.get('landingApproachCoordinate')[2],  # altitude
                                10,        # speed (m/s)
                                True,              
                                float('nan'),     
                                float('nan'),     
                                MissionItem.CameraAction.NONE,
                                float('nan'),      
                                float('nan'),     
                                float('nan'),      
                                float('nan'),     
                                float('nan'),     
                                MissionItem.VehicleAction.NONE  
                                    ))
                #print(f"{item.get('landingApproachCoordinate')[0]},{item.get('landingApproachCoordinate')[1]},green,marker")
                mission_items.append(MissionItem(
                                item.get('landCoordinate')[0],      # latitude
                                item.get('landCoordinate')[1],      # longitude
                                item.get('landCoordinate')[2],  # altitude
                                10,        # speed (m/s)
                                True,              
                                float('nan'),     
                                float('nan'),     
                                MissionItem.CameraAction.NONE,
                                float('nan'),      
                                float('nan'),     
                                float('nan'),      
                                float('nan'),     
                                float('nan'),     
                                MissionItem.VehicleAction.LAND 
                                    ))    
                ##print(f"{item.get('landCoordinate')[0]},{item.get('landCoordinate')[1]},green,marker")
        mission_plan = MissionPlan(mission_items)
        print("-- Uploading mission")
        await drone.mission.upload_mission(mission_plan)
        print(f"-- Mission uploaded ")
offboard_points=[]
def calculate_grid_points(grid_size_m):
    data1=read_plan()
    geofence = data1.get('geoFence', {}).get('polygons', [])
    geofence_coordinates = []
    for coordinates in geofence:
        geofence_coordinates=coordinates.get('polygon')
    lat_min, lat_max = min(geofence_coordinates[0][0], geofence_coordinates[2][0]), max(geofence_coordinates[0][0], geofence_coordinates[2][0])
    lon_min, lon_max = min(geofence_coordinates[0][1], geofence_coordinates[1][1]), max(geofence_coordinates[0][1], geofence_coordinates[1][1])

    lat_degree_to_m = 111000
    lon_degree_to_m = 111000 * np.cos(np.radians((lat_min + lat_max) / 2))

    lat_step = grid_size_m / lat_degree_to_m
    lon_step = grid_size_m / lon_degree_to_m


    
    grid_points = []

    lat = lat_min
    while lat <= lat_max:
        lon = lon_min
        while lon <= lon_max:
            grid_points.append((lat, lon))
            #print(f"{lat},{lon},blue,marker")
            lon += lon_step
        lat += lat_step

    # Print the corners
    #print(f"{lat_min},{lon_min},red,marker")
    #print(f"{lat_max},{lon_max},red,marker")

    return grid_points,lat_min,lat_max,lon_min,lon_max

async def generate_thermals(grid_size_m,duration,index):
    duration_sec=duration*60
    grid_points,lat_min_bound,lat_max_bound,lon_min_bound,lon_max_bound = calculate_grid_points(grid_size_m)
    lat_degree_to_m = 111000
    thermal_points=[]
    a=False
    while True:
  
        print(f"index : {index}")
        thermal_points.clear()
        thermal_point_types[index].clear()
        for lat, lon in grid_points:
             
              lon_degree_to_m = 111000 * np.cos(np.radians(lat))

              lat_step = grid_size_m / lat_degree_to_m
              lon_step = grid_size_m / lon_degree_to_m
           
              lat_min = lat
              lon_min = lon
              lat_max = lat + lat_step
              lon_max = lon + lon_step

              if (lat_min_bound <= lat_min <= lat_max_bound and 
                  lon_min_bound <= lon_min <= lon_max_bound
                  ):
                  #print(f"{lat_min},{lon_min},green,marker")
                  lat_min_square=lat_min
                  lon_min_square=lon_min
                  a=True
              else :
                  #print(f"{lat_min},{lon_min},yellow,marker")
                  a=False
              if (
                  lat_min_bound <= lat_max <= lat_max_bound and
                  lon_min_bound <= lon_max <= lon_max_bound):
                      #print(f"{lat_max},{lon_max},green,marker")
                      lat_max_square=lat_max
                      lon_max_square=lon_max
                      a=True
              else:
                      #print(f"{lat_max},{lon_max},yellow,marker")
                      a=False
              if a==True:
                  for _ in range (1):
                      lat_variation = np.random.uniform(lat_min_square, lat_max_square)  
                      lon_variation = np.random.uniform(lon_min_square, lon_max_square)  
                      thermal_points.append((lat_variation,lon_variation))
                      #print(f"{lat_variation},{lon_variation},black,marker")
                  a==False
              #thermal_point_types[index].extend(thermal_points)
        thermal_point_types[index].extend(thermal_points)
        #print(f"thermals: {thermal_point_types[index]}")
        await asyncio.sleep(duration_sec)
        
                
    

          
async def track_thermals(indx,duration):
    index=indx
    while 1 : 
        t=duration*60 
        while t: 
            mins, secs = divmod(t, 60) 
            timer = '{:02d}:{:02d}'.format(mins, secs) 
            await asyncio.sleep(1) 
            t -= 1
            remaining_time[index]=t
            #print(f"type 1: {remaining_time[0]} ¦ type 2: {remaining_time[1]} ¦type 3: {remaining_time[2]}  ")
            #print (f"---------------------------------------------------------------------------------")        
        #print (f"la thermique de duree {duration} est termine ")
        

async def update_thermal_points():
    while True:
        #print (f"tab 1 :{thermal_point_types[0]}")
        #print (f"tab 2 :{thermal_points_type_2_generated}")
        #print (f"tab 3 :{thermal_points_type_3_generated}")
        #print (f"---------------------------------------------------------------------------------")
        print(f"Thermal points in circle: {thermal_in_circle}")
        print("-----------------------------------")
        await asyncio.sleep(2)
        
        #print(f"type 1: {remaining_time[1]} \n type 2: {remaining_time[2]} \n type 3: {remaining_time[3]}")
def midpoint(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    d_lon = lon2 - lon1
    x = math.cos(lat2) * math.cos(d_lon)
    y = math.cos(lat2) * math.sin(d_lon)
    lat_mid = math.atan2(math.sin(lat1) + math.sin(lat2),
                         math.sqrt((math.cos(lat1) + x) ** 2 + y ** 2))
    lon_mid = lon1 + math.atan2(y, math.cos(lat1) + x)
    return math.degrees(lat_mid), math.degrees(lon_mid)
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c  # Distance in meters

def radius_of_circle(lat1, lon1, lat2, lon2):
    return haversine(lat1, lon1, lat2, lon2) / 2

def is_within_circle(lat_center, lon_center, lat3, lon3, radius):
    distance = haversine(lat_center, lon_center, lat3, lon3)
    return distance <= radius

async def thermal_in_circle_check(drone):
    while True:
        async for position in drone.telemetry.position():
            thermal_in_circle.clear()
            uav_lat = position.latitude_deg
            uav_long = position.longitude_deg
            lat_center, lon_center = midpoint(uav_lat, uav_long, 34.20733471, 108.8008996)
            radius = radius_of_circle(uav_lat, uav_long, 34.20733471, 108.8008996)

            #print(f"UAV Position: {uav_lat}, {uav_long}, Circle Center: {lat_center}, {lon_center}, Radius: {radius}")

        
            for thermal1, thermal2, thermal3 in zip(thermal_points_type_1_generated, thermal_points_type_2_generated, thermal_points_type_3_generated):
           
                if is_within_circle(lat_center, lon_center, thermal1[0], thermal1[1], radius):
                    distance_uav_thermal1 = haversine(uav_lat, uav_long, thermal1[0], thermal1[1])
                    distance_thermal1_mission_point = haversine(thermal1[0], thermal1[1], 34.20733471, 108.8008996)
                    total_distance1 = distance_thermal1_mission_point + distance_uav_thermal1
                    time_to_reach1 = distance_uav_thermal1 / speed
                    time_to_reach_min1 = time_to_reach1 / 60
                    exploration_time1=duration_thermal_type_1_in_min-time_to_reach_min1
                    thermal_in_circle.append([thermal1[0], thermal1[1], time_to_reach_min1, total_distance1,exploration_time1])

    
                if is_within_circle(lat_center, lon_center, thermal2[0], thermal2[1], radius):
                    distance_uav_thermal2 = haversine(thermal2[0], thermal2[1], 34.206617099999995, 108.8047153)
                    distance_thermal2_mission_point = haversine(thermal2[0], thermal2[1], 34.20733471, 108.8008996)
                    total_distance2 = distance_thermal2_mission_point + distance_uav_thermal2
                    time_to_reach2 = distance_uav_thermal2 / speed
                    time_to_reach_min2 = time_to_reach2 / 60
                    exploration_time2=duration_thermal_type_2_in_min-time_to_reach_min2
                    thermal_in_circle.append([thermal2[0], thermal2[1], time_to_reach_min2, total_distance2,exploration_time2])

     
                if is_within_circle(lat_center, lon_center, thermal3[0], thermal3[1], radius):
                    distance_uav_thermal3 = haversine(thermal3[0], thermal3[1], 34.206617099999995, 108.8047153)
                    distance_thermal3_mission_point = haversine(thermal3[0], thermal3[1], 34.20733471, 108.8008996)
                    total_distance3 = distance_thermal3_mission_point + distance_uav_thermal3
                    time_to_reach3 = distance_uav_thermal3 / speed
                    time_to_reach_min3 = time_to_reach3 / 60
                    exploration_time3=duration_thermal_type_3_in_min-time_to_reach_min3
                    thermal_in_circle.append([thermal3[0], thermal3[1], time_to_reach_min3, total_distance3,exploration_time3])

async def mission_offboard_switcher(drone):
    exploitable_points=[]
    for thermal in thermal_in_circle:
        if thermal[4]>=desired_exploration_time_min:
            exploitable_points.append([thermal[0],thermal[1],thermal[3],thermal[4]])
    minimal_path_point=min(exploitable_points)
    await drone.mission.pause_mission()
    await offboard_mode(minimal_path_point[0],minimal_path_point[1])

async def offboard_mode(drone,lat,long): 
    latitude = np.array(lat)
    longitude = np.array(long)
    alt=50
    ned_coordinates = navpy.lla2ned(latitude, longitude, alt, lat_ref, lon_ref, alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84')
    await drone.offboard.start()
    await drone.offboard.set_position_ned(PositionNedYaw(ned_coordinates[0,0], ned_coordinates[0, 1], ned_coordinates[0, 2], 0.0))
    await wait_until_position_reached(drone, latitude[0], longitude[0],alt, 0.01)
    await continue_to_next_point()
async def continue_to_next_point():
    print()

async def wait_until_position_reached(drone, lat,long,alt, tolerance):
    async for position in drone.telemetry.position():
        dist = mpu.haversine_distance((float(position.latitude_deg), float(position.longitude_deg)), (float(lat), float(long)))
        dist_float=float(dist)
        if dist_float <= tolerance:  
                print(f"Reached target position with tolerance: {tolerance} meters")
                break

async def termination (drone):
    async for mission_progress in drone.mission.mission_progress():
        current_index = mission_progress.current+1
        print(f"mission: {mission_progress.current+1} / {mission_progress.total}")
        if current_index==mission_progress.total:
            #await drone.action.land()
            #print ("landed")
            #await asyncio.sleep(2)
            #await drone.action.kill()
            #print ("disarmed")
            break
    return(mission_progress)
async def track_mission (drone):
    async for mission_progress in drone.mission.mission_progress():
        current_index = mission_progress.current+1
        print(f"mission: {mission_progress.current+1} / {mission_progress.total}")
        return mission_progress.current+1

if __name__ == "__main__":
    asyncio.run(run())

