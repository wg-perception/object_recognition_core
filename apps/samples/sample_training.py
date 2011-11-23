#!/usr/bin/env python
import ecto
import sys
import couchdb
from ecto_opencv.highgui import imshow, ImageSaver
from ecto_object_recognition import capture
from ecto_object_recognition.object_recognition_db import ObjectDbParameters
from object_recognition import models, dbtools
import ecto_opencv


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

    writer_image = ImageSaver(filename_format='images/%05d.png')[:]
    writer_depth = ImageSaver(filename_format='depth/%05d.png')[:]
    writer_mask = ImageSaver(filename_format='mask/%05d.png')[:]

    #visualization
    plasm.connect(db_reader['image'] >> (writer_image, imshow(name='image')['image']),
                  db_reader['mask'] >> (writer_mask, imshow(name='mask')['image']),
                  db_reader['depth'] >> (writer_depth, imshow(name='depth')['image'])
                  )

    sched = ecto.schedulers.Singlethreaded(plasm)
    sched.execute()

    #After done execution upload the resulting model to the db....

