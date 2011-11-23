#!/usr/bin/env python
import ecto
import sys
import couchdb
from ecto_opencv.highgui import imshow, ImageSaver
from ecto_object_recognition import capture
from ecto_object_recognition.object_recognition_db import ObjectDbParameters
from object_recognition import models, dbtools
import ecto_opencv
import os

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Train MyAlgorithm on views from the database.')
    parser.add_argument('objects', metavar='OBJECT', type=str, nargs='+',
                   help='Object ids to train.')
    dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

args = parse_args()
db = dbtools.init_object_databases(couchdb.Server(args.db_root))

for object_id in args.objects:
    #get a list of observation ids for a particular object id.
    obs_ids = models.find_all_observations_for_object(db, object_id)

    if not obs_ids:
        print 'No observations found for object %s.' % object_id
        continue

    plasm = ecto.Plasm()
    params = ObjectDbParameters(dict(type=args.db_type, root=args.db_root, collection=args.db_collection))
    #the db_reader transforms observation id into a set of image,depth,mask,K,R,T
    db_reader = capture.ObservationReader("db_reader", db_params=params)
    #this iterates over all of the observation ids.
    observation_dealer = ecto.Dealer(tendril=db_reader.inputs.at('observation'), iterable=obs_ids)

    plasm.connect(observation_dealer[:] >> db_reader['observation'])

    path_names = [ 'image', 'depth', 'mask']
    for path_name in path_names:
        path = os.path.join(object_id, path_name)
        if not os.path.exists(path):
            os.makedirs(path)
        writer_image = ImageSaver(filename_format=os.path.join(path, '%05d.png'))[:]
        plasm.connect(db_reader[path_name] >> (writer_image, imshow(name=path_name)['image']))

    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

    # write files listing the produced files
    for path_name in path_names:
        file_object = open(os.path.join(os.getcwd(), object_id, path_name + '.txt'), 'w')
        for file_name in [ os.path.join(os.getcwd(), object_id, path_name, file_name)
                          for file_name in os.listdir(os.path.join(object_id, path_name))
                          if file_name.endswith('png') ]:
            file_object.write(file_name + '\n')
        file_object.close

    #After done execution upload the resulting model to the db....
