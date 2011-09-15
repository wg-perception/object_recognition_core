#!/usr/bin/env python
import object_recognition
from object_recognition import dbtools, models
import couchdb
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Delete sessions from the database.')
    parser.add_argument('sessions', metavar='SESSION', type=str, nargs='+',
                   help='Sessions to delete.')
    object_recognition.dbtools.add_db_options(parser)
    args = parser.parse_args()
    return args

def delete_observations(dbs, session_ids, commit):
        sessions = dbs['sessions']
        observations = dbs['observations']
        for id in session_ids:
            session = models.Session.load(sessions, id)
            for observation in models.Observation.by_session_id(observations, key=session.id):
                print 'deleting observation: ', observation.id
                if commit:
                    observations.delete(observation)
            print 'deleting session: ', id
            if  commit:
                sessions.delete(session)
if __name__ == "__main__":
    args = parse_args()
    couch = couchdb.Server(args.db_root)
    dbs = dbtools.init_object_databases(couch)
    delete_observations(dbs, args.sessions, args.commit)
    if not args.commit:
        print 'just kidding. --commit to actually do it.'
