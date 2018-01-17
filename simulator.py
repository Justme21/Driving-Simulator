import map_builder

num_junctions = 4
num_roads = 3
num_cars = 1

road_angles = [180,0,50]
road_lengths = [30,30,30]

junc_pairs = [(0,3),(3,1),(2,3)]

cars,junctions,roads = map_builder.build_map(num_junctions,num_roads,num_cars,road_angles,\
                                             road_lengths,junc_pairs)

map_builder.print_contents(junctions[0])
print("\n\n")
for entry in cars:
    entry.print_status()
