from __future__ import print_function
import gmaps
import numpy as np
import googlemaps
import json
from googleplaces import GooglePlaces
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# input the places of interest (POI)

# YHA London Central Hostel
# Coca-Cola London Eye
# St. Paul's Cathedral
# Leadenhall Market
# The National Gallery
# Big Ben
# Buckingham Palace
# Waterloo Station


places = 'Inorbit Mall', 'Oberoi Mall', 'Infiniti Mall', 'Subway', 'Natraj Market','LandMark', 'Flags', 'Privilege In'

# the region
Location='Mumbai'

# choose a mode
Mode = "walking"  # "driving", "walking", "bicycling", "transit"

# get Google API key from following website: 
# https://developers.google.com/maps/documentation/distance-matrix/start#get-a-key
password = "AIzaSyBM8UC4aDTqAriv05bI2mgEGaAax9Lo-sw"


# get the location of POI


class GoogleMaps(object):

    def __init__(self, password):

        self._GOOGLE_MAPS_KEY = password
        self._Google_Places = GooglePlaces(self._GOOGLE_MAPS_KEY)
        self._Google_Geocod = googlemaps.Client(key=self._GOOGLE_MAPS_KEY)

    def _text_search(self, query, language=None, location=None):
        text_query_result = self._Google_Places.text_search(query=query, language=language, location=location)
        return text_query_result.places

    def _reverse_geocode(self, lat, lng, language=None):
        list_reverse_geocode_result = self._Google_Geocod.reverse_geocode((lat, lng), language=language)
        return list_reverse_geocode_result

    def _return_reverse_geocode_info(self, lat, lng, language=None):
        list_reverse_geocode = self._reverse_geocode(lat, lng, language=language)
        if list_reverse_geocode:
            city = ''
            pincode = ''
            route = ''
            neighborhood = ''
            sublocality = ''
            administrative_area_level_1 = ''
            country = ''
            street_number = ''
            
            formatted_address = list_reverse_geocode[0]['formatted_address']
            for address_info in list_reverse_geocode[0]['address_components']:
                if 'locality' in address_info['types']:
                    city = address_info['long_name']
                elif 'postal_code' in address_info['types']:
                    pincode = address_info['long_name']
                elif 'route' in address_info['types']:
                    route = address_info['long_name']
                elif 'neighborhood' in address_info['types']:
                    neighborhood = address_info['long_name']
                elif 'sublocality' in address_info['types']:
                    sublocality = address_info['long_name']
                elif 'administrative_area_level_1' in address_info['types']:
                    administrative_area_level_1 = address_info['long_name']
                elif 'country' in address_info['types']:
                    country = address_info['long_name']
                elif 'street_number' in address_info['types']:
                    street_number = address_info['long_name']
            return {'city': city, 'pincode': pincode, 'route': route, 'neighborhood': neighborhood,
                    'sublocality': sublocality, 'administrative_area_level_1': administrative_area_level_1,
                    'country': country, 'formatted_address': formatted_address, 'street_number': street_number}
        else:
            return None

    def get_pincode_city(self, lat, lng, language=None):
        reverse_geocode_info = self._return_reverse_geocode_info(lat, lng, language=language)
        if reverse_geocode_info:
            return {'city': reverse_geocode_info['city'], 'pincode': reverse_geocode_info['pincode']}
        else:
            return None

    def get_address_recommendation(self, query, language=None, location=None):
        return_size = 1 # 5
        list_return_info = list()
        list_places_text_search_result = self._text_search(query=query, language=language, location=location)
        
        if len(list_places_text_search_result) > return_size:
            list_places_text_search_result = list_places_text_search_result[:return_size]
        for place in list_places_text_search_result:
            result_geocode = self._return_reverse_geocode_info(place.geo_location['lat'], place.geo_location['lng'], language=language)
            if result_geocode:
                result_geocode['formatted_address'] = '{} {}'.format(place.name, result_geocode['formatted_address'])
                result_geocode['place_name'] = place.name
                result_geocode['lat'] = '{}'.format(place.geo_location['lat'])
                result_geocode['lng'] = '{}'.format(place.geo_location['lng'])
                list_return_info.append(result_geocode)
        return list_return_info



# get the lat and lng of places

lat = []
lng = []

google_maps = GoogleMaps(password)
for place in places:
    print(place)
    result = google_maps.get_address_recommendation(query=place, language='en', location=Location)
    lat = np.append(lat, result[0]["lat"])
    lng = np.append(lng, result[0]["lng"])
lat = lat.astype(float)
lng = lng.astype(float)
lat, lng



# calculate the dist_matrix
# distance unit: meter


gmaps = googlemaps.Client(key=password)

dist_matrix = []

for i in range(len(places)):
    for j in range(len(places)):
        x = (lat[i], lng[i])
        y = (lat[j], lng[j])
        directions_result = gmaps.directions(x,y,
                                    mode=Mode,
                                    avoid="ferries",
                                    )
        dist_matrix.append(directions_result[0]['legs'][0]['distance']['value'])
dist_matrix = np.reshape(dist_matrix, (len(places), len(places)))
# dist_matrix.astype(int)
dist_matrix


# In[21]:


# convert the dist_matrix to a symmetrical matrix

dist_matrix = np.asmatrix(dist_matrix)

for i in range(0, len(places), 1):
    for j in range(i+1, len(places), 1):
        dist_matrix[j,i] = dist_matrix[i,j]
dist_matrix = np.asarray(dist_matrix)
dist_matrix


# TSP Solver

"""Simple travelling salesman problem between cities."""




def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dist_matrix
    data['city_names'] = places
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def print_solution(manager, routing, assignment):
    """Prints assignment on console."""
    print('Total distance: {} meters'.format(assignment.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Index:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)
    
    
def return_indexes(routing, assignment):
    index = routing.Start(0)
    indexes = []
    while not routing.IsEnd(index):
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        indexes = np.append(indexes, index)
    return indexes
    

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        print_solution(manager, routing, assignment)
        indexes = return_indexes(routing, assignment)
    return indexes


if __name__ == '__main__':
    Index = main()


# sorting the lat and lng according to the order of tour

new_lat = [lat[0]]
new_lng = [lng[0]]
for i in range(len(places)-1):
    index = Index[i].astype(int)
    new_lat = np.append(new_lat, lat[index])
    new_lng = np.append(new_lng, lng[index])
new_lat = np.append(new_lat, lat[0])
new_lng = np.append(new_lng, lng[0])
# new_lat = new_lat.tolist()
# new_lng = new_lng.tolist()
new_lat, new_lng


# draw lines and points (option)


gmaps.configure(api_key=password)

# fix too many tool box
fig = gmaps.figure(center=(np.mean(new_lat.astype(float)),np.mean(new_lng.astype(float))), zoom_level=12)


fig.add_layer(gmaps.drawing_layer(features=[
    gmaps.Marker((new_lat[i], new_lng[i]), label='%s' % (i+1)) for i in range(len(places))  
]))

fig.add_layer(gmaps.drawing_layer(features=[
    gmaps.Line((new_lat[i], new_lng[i]), (new_lat[i+1], new_lng[i+1]), stroke_weight=3.0) for i in range(len(places))
]))   
fig



gmaps.configure(api_key=password)

fig = gmaps.figure(center=(np.mean(new_lat.astype(float)),np.mean(new_lng.astype(float))), zoom_level=12)

# fix too many tool box
fig.add_layer(gmaps.drawing_layer(features=[
    gmaps.Marker((new_lat[i], new_lng[i]), label='%s' % (i+1)) for i in range(len(places))  
]))

for i in range(len(places)):
    fig.add_layer(
        gmaps.directions.Directions((new_lat[i], new_lng[i]), (new_lat[i+1], new_lng[i+1]),mode='WALKING',show_markers=False)
    )

fig



print('legend: \n')
for i in range(len(places)):
    print(i+1, ' = ', places[i])




