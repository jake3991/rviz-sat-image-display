import yaml

def load_config(file_path):
    '''Load up a yaml file and return it as a dictionary 
    file_path: the path to the relevant yaml file
    return: a dictionary of the yaml file
    '''

    #open the file
    with open(file_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader) #read it

        #convert the list of dictionaries returned into a simple dict
        result = {}
        for d in config:
            result.update(d)
        config = result
        
    return config

def write_config(config, file_path):
    '''Writes the dictionary to a yaml file
    config: python dictionary, the configuration
    file_path: the path/name of the file
    '''

    #dump the configuration file into yaml
    with open(file_path, 'w') as file:
        yaml.dump(config, file)
