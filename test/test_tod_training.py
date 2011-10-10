#!/usr/bin/env python

import ecto
from ecto_object_recognition import capture
from object_recognition.tod.trainer import Trainer

db_reader = capture.ObservationReader("db_reader")
source_plasm = ecto.Plasm()
observation_dealer = ecto.Dealer(tendril=db_reader.inputs.at('observation'), iterable=[])
source_plasm.connect(observation_dealer[:] >> db_reader['observation'])

T = Trainer(source=db_reader, source_plasm=source_plasm)

print T.__doc__
