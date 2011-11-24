from object_recognition.pipelines.training import find_training_pipelines

training_pipelines = find_training_pipelines(['object_recognition'])
for X in training_pipelines:
    print X


