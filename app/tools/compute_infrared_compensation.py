#!/usr/bin/env python

import csv
import argparse
import os
import sys

import numpy as np
import airsim


def compute_infrared_correction(destination, camera_name):
    """
    Compute the transfer dynamics from segmentation id in a currently running
    UE4 game and the value in the infrared image and save it to file.
    """
    # Parse destination
    path = destination
    if destination.endswith('.csv'):
        path = os.path.dirname(destination)
    else:
        destination = os.path.join(path, 'infrared_corrections.csv')
    if not os.path.isdir(path):
        os.makedirs(path)

    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Compute
    print "Computing infrared corrections:"
    with open(destination, 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["MeshID", "InfraRedID"])
        requests = [
            airsim.ImageRequest(str(camera_name), airsim.ImageType.Infrared,
                                False, False)
        ]
        for i in range(256):
            client.simSetSegmentationObjectID(r"[\w]*", i, True)
            responses = client.simGetImages(requests)
            response = responses[0]
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
            writer.writerow([i, img1d[0]])

            # Display Progress
            progress = i / 255.0
            progress_disc = int(progress * 50)
            sys.stdout.write('\r')
            sys.stdout.write("Progress: [%-50s] %.2f%%" %
                             ('=' * progress_disc, progress * 100))
            sys.stdout.flush()
    print "\nSaved infrared corrections in '%s'." % destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d",
                        "--destination",
                        dest='d',
                        help="Path or filename, ending in .csv, where to save "
                        "the infrared corrections.",
                        default=os.getcwd())
    parser.add_argument("-c",
                        "--camera_name",
                        dest='c',
                        help="Name of the camera to use in AirSim.",
                        default="")
    args = parser.parse_args()
    compute_infrared_correction(args.d, args.c)
