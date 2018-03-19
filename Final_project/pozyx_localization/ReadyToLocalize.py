#!/usr/bin/env python2
"""
Modified from The Pozyx ready to localize tutorial (c) Pozyx Labs

This script filters the data read in from the Pozyx and prints it.

TODO: Support definition of tags, tag names, and tag labels in an external file

Before running, make sure:
 - Anchors are calibrated, 
 - A tag is connected to the laptop via USB
 - Other tags are powered and in the "tags" list below
"""
from copy import deepcopy
from pypozyx import POZYX_POS_ALG_UWB_ONLY, POZYX_3D, POZYX_ANCHOR_SEL_AUTO, POZYX_SUCCESS, Coordinates, SingleRegister

#import FilterData as fd
#import Interaction as interaction

class ReadyToLocalize(object):
    """ Continuously calls the Pozyx positioning function
        and prints its position.
    """

    def __init__(self, pozyx, tags, anchors, filter_data, interact, rospub,
            algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000):
        self.pozyx = pozyx
        self.tags = tags
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height

        self.filter_data = filter_data
        self.interact = interact
        self.ros_publisher = rospub

    def setup(self):
        """ Sets up the Pozyx for positioning by calibrating its anchor list """
        #print("------------POZYX POSITIONING V1.1 -------------")
        #print("NOTES: ")
        #print("- No parameters required.")
        #print()
        #print("- System will auto start configuration")
        #print()
        #print("- System will auto start positioning")
        #print("------------POZYX POSITIONING V1.1 --------------")
        #print()
        print "START Ranging: "
        self.setAnchorsManual()
        self.printPublishAnchorConfiguration()

    def loop(self):
        """ Performs positioning and displays/exports the results. """
        for i, (tag, _, tag_cat) in enumerate(self.tags):
            position = Coordinates()
            status = self.pozyx.doPositioning(position, self.dimension, 
                                    self.height, self.algorithm, remote_id=tag)

            if status == POZYX_SUCCESS:
                origpos = deepcopy(position)
                position.x, position.y, position.z = \
                                            self.filter_data[i].filt(position)
                self.printPublishPosition(tag, position, origpos)
                self.interact.updateLocation(tag, tag_cat, position)
            else:
                self.printPublishErrorCode("positioning", tag)
        
        # Check for any interactions between SquirtleBot and Pokemon
        # Just checking for existence of interaction should be fine since Pokemon are spaced out
        self.interact.findInteractions(self.ros_publisher)

    def printPublishPosition(self, tag, position, origpos):
        """ Prints the Pozyx's position before and after filtering. """
        print "POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x{}".format(tag), pos=origpos)        

        print ("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z} "
                            "FILTERED").format("0x{}".format(tag), pos=position)  

    def setAnchorsManual(self):
        """ Adds the manually measured anchors to the Pozyx's 
            device list one for one."""
        for tag, _, _ in self.tags:
            status = self.pozyx.clearDevices(tag)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag)
            if len(self.anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(
                                    POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
            self.printPublishConfigurationResult(status, tag)

    def printPublishConfigurationResult(self, status, tag_id):
        """ Prints and potentially publishes the anchor configuration result 
            in a human-readable way. """
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print "Configuration of tag {}: success".format(hex(tag_id))
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPublishErrorCode(self, operation, network_id):
        """ Prints the Pozyx's error. """
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print "Error {} on ID {}, error code {}".format(
                  operation, "0x{}".format(network_id), str(error_code))
        else:
            # should only happen when unable to communicate w/ remote Pozyx
            self.pozyx.getErrorCode(error_code)
            print "Error {} local error code {}".format(operation, str(error_code))

    def printPublishAnchorConfiguration(self):
        """ Prints the anchor configuration. """
        for anchor in self.anchors:
            print "ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos))
            
"""
if __name__ == "__main__":
    # Shortcut to not have to find out the port yourself
    serial_port = get_serial_ports()[0].device

    # (device_id, x, y, z)
    anchors_simple = [(0x6E30, 0, 0, 1334),
                      (0x6969, 2921, 0, 1334),
                      (0x6E69, 3148, 4617, 0),
                      (0x6E52, 6342, 4424, 1334)]
    anchors = []
    for anc_id, x, y, z in anchors_simple:
        anchors.append(DeviceCoordinates(anc_id, 1, Coordinates(x, y, z)))

    # (tag_id, name, category)
    tags = [(0x6E22, "SquirtleBot", "Robot"),
            (0x6E13, "Pikachu", "Pokemon"), 
            (0x6E5A, "Dragonite", "Pokemon"),
            (0x0000, "Mew", "Pokemon")]
            #(0x0000, "Trainer", "Human")]

    algorithm = POZYX_POS_ALG_UWB_ONLY  # Positioning algorithm to use
    dimension = POZYX_3D                # Positioning dimension
    height = 1000                       # Height of device (2.5D positioning)

    # Create a new FilterData instance for each tag
    # Filter using past 10 points; Ignore points 500 mm away from previous point
    filter_data = [fd.FilterData(10, 500) for i in range(len(tags))]
    # Used for determining interaction between tags
    # 400 mm between tags suggests interaction; use the last 10 top interactions
    interact = interaction.Interaction(tags, 400, 10)

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, tags, anchors, filter_data, interact,
                                        algorithm, dimension, height)
    r.setup()    

    while True:
        r.loop()
"""
