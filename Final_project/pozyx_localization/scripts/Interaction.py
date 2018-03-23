"""
A class for determining if a person is interacting with an object.

TODO generalize for any tag labels
"""
from math import sqrt
from queue import Queue

pokemon_locs = {
        "demo": {"x": 3300, "y": 610},
        "pikachu" : {"x": 8690, "y": 1700},
        "dragonite" : {"x": 21000, "y": -1000},
        "mew": {"x": 42800, "y": 1900}
        }

class Interaction(object):

	def __init__(self, tags, dist=400, num_items=10):
		# taggedObjects[category][tag_id][name, x, y, interactions, int_counts]
		self.__taggedObjects = self.__initTagDicts(tags, num_items)
		self.__dist = dist
		self.__q_size = num_items

	def __initTagDicts(self, tags, num_items):
		d = {}
		for tag_id, tag_name, tag_cat in tags:
			if tag_cat not in d:
				d[tag_cat] = {}
			d[tag_cat][tag_id] = {"name": tag_name,
								  "x": None, "y": None,
								  "interactions": Queue(maxsize=num_items),
								  "int_counts": {}}
		return d
        def updateLocation(self, tag_id, tag_cat, new_pos):
            self.__taggedObjects[tag_cat][tag_id]["x"] = new_pos.x
            self.__taggedObjects[tag_cat][tag_id]["y"] = new_pos.y

	def findInteractions(self, rospub):
                interactionFound = (False, None) #(whether or not pokemon was caught, which pokemon)
		# Update interation counts for each person
		for human, h_dic in self.__taggedObjects["Robot"].items():
                        if h_dic["x"]:
			        # Keep a list of all possible interactions at this time
			        h_ints = []
                                for pokemon, loc_dic in pokemon_locs.items():
                                    dist = sqrt((h_dic["x"] - loc_dic["x"])**2 +
                                            (h_dic["y"] - loc_dic["y"])**2)
                                    rospub.publish(pokemon + ": " + str(dist))
                                    #print dist
                                    if dist < self.__dist:
                                        h_ints.append(pokemon)
                                        if pokemon not in h_dic["int_counts"]:
                                            h_dic["int_counts"][pokemon] = 0
                                        h_dic["int_counts"][pokemon] += 1
                                h_dic["interactions"].put(h_ints)
		# Consider someone to be interacting with an object if they are close
		# to that object for at least half of the previous q_size timesteps
		for human, h_dic in self.__taggedObjects["Robot"].items():
			for item, cnt in h_dic["int_counts"].items():
				if cnt > 0.5*self.__q_size:
                                        interactionFound = (True, item)
					print "{} may be interacting with {}".format(h_dic["name"], item)
                                            #self.__taggedObjects["Pokemon"][item]["name"])

		# Remove old interactions from the queues and update counts
		for human, h_dic in self.__taggedObjects["Robot"].items():
			if h_dic["interactions"].full():
				oldInts = h_dic["interactions"].get(block=False)
				for t in oldInts:
					h_dic["int_counts"][t] -= 1

		print "Done finding interactions"
                if interactionFound[0]:
                    rospub.publish("CAUGHT " + interactionFound[1])
