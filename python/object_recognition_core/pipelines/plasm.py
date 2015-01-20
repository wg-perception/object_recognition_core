'''
Loaders for all object recognition pipelines
'''
from object_recognition_core.io.voter import VoterBase
from object_recognition_core.utils.find_classes import find_cell
import ecto
import sys
import traceback

class OrkPlasmError(RuntimeError):
    pass

def connect_cells(cell1, cell2, plasm):
    """
    Given two cells, connect them with all the possible tendrils in the plasm
    """
    for key in set(cell1.outputs.keys()).intersection(cell2.inputs.keys()):
        plasm.connect(cell1[key] >> cell2[key])

def create_plasm(ork_params):
    """
    Function that returns a plasm corresponding to the input arguments
    
    :param ork_params: a dictionary of the parameters of the different cells as explained in the documentation.
        Each key is a unique identifier of a cell (a cell being a SourceBase, SinkBase, PipelineBase, VoterBase,
        anything) and each key is a dictionary with the following keys: 'module' (a string to define the Python
        module where to find the cell), 'type' (the class name of the cell), 'inputs' and/or 'outputs' (a list
        of identifiers to know what to link the cell to) and 'parameters' (a dictionary of parameters to call
        the constructor of the cell with)
    """
    cells = {}
    voter_n_inputs = {}
    # first, find the classes of the cells and figure out the voters
    for cell_name, parameters in ork_params.items():
        if 'module' not in parameters:
            raise OrkPlasmError('You need a "module" parameter to define where your cell "%s" is.' % cell_name)
        if 'type' not in parameters:
            raise OrkPlasmError('You need a "type" parameter to define what your cell "%s" is.' % cell_name)
        cell_class = find_cell([parameters['module']], parameters['type'])

        if issubclass(cell_class, VoterBase):
            voter_n_inputs[cell_name] = 0
            # do not instantiate the voters yet 
            cells[cell_name] = cell_class
        else:
            # instantiate the cell
            try:
                if 'parameters' in parameters:
                    cells[cell_name] = cell_class(cell_name, **parameters['parameters'])
                else:
                    cells[cell_name] = cell_class(cell_name)
            except TypeError as err:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                err = traceback.format_exception(exc_type, exc_value, exc_traceback)
                raise OrkPlasmError('Could not initialize cell "%s" because of: %s' % (cell_name, ''.join(err)))

    # Figure out the number of inputs to each voter
    for cell_name, parameters in ork_params.items():
        if cell_name not in voter_n_inputs:
            for potential_voter_name in parameters.get('outputs', []):
                if potential_voter_name in voter_n_inputs:
                    voter_n_inputs[potential_voter_name] += 1

    # instantiate the voters
    for cell_name, n_inputs in voter_n_inputs.items():
        cells[cell_name] = cells[cell_name](cell_name=cell_name, n_inputs=n_inputs, **ork_params[cell_name])

    # build the plasm with all the connections
    plasm = ecto.Plasm()
    already_processed_connections = set()
    for cell_name, cell in cells.items():
        plasm.insert(cell)
        # link to inputs ...
        for input_name in ork_params[cell_name].get('inputs', []):
            connection = (input_name, cell_name)
            # ... but only once
            if connection in already_processed_connections:
                continue
            if input_name not in cells:
                raise OrkPlasmError('You need a cell of name "%s" as it is an input to "%s".' % 
                                    (input_name, cell_name))
            connect_cells(cells[input_name], cell, plasm)
            already_processed_connections.add(connection)
        # link to outputs ...
        for output_name in ork_params[cell_name].get('outputs', []):
            connection = (cell_name, output_name)
            # ... but only once
            if connection in already_processed_connections:
                continue
            if output_name not in cells:
                raise OrkPlasmError('You need a cell of name "%s" as it is an output to "%s".' % 
                                    (output_name, cell_name))
            connect_cells(cell, cells[output_name], plasm)
            already_processed_connections.add(connection)

    # make sure each cell is present in at least one connection
    if len(cells.keys()) > 1:
        for cell_name in cells.keys():
            if not any([connection[0]==cell_name or connection[1]==cell_name
                   for connection in already_processed_connections]):
                raise OrkPlasmError('Cell "%s" is not connected to any other cell.' % cell_name)

        if not already_processed_connections:
            raise OrkPlasmError('There are no connections in your graph.')

    return plasm
