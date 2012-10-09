#!/usr/bin/env python
"""
This script test the robustness of the pipeline to different kinds of config files
"""

import yaml
from object_recognition_core.utils.training_detection_args import common_parse_config_file, common_parse_config_string, OrkConfigurationError

if __name__ == '__main__':
    # Check that the proper error is thrown when the file does not exist
    try:
        common_parse_config_file('/dghllsjdghilsdfjhglsdjhgdfjlsghdlsfjhasdf')
    except OrkConfigurationError:
        pass
    except e:
        raise RuntimeError(e)

    # Now, check a few problematic strings
    for config_string in [
# non-yaml string
"""
lkdg
random_crap:
""",
# string with unknown keys
"""
custom: 'hello'
""",
# empty parameters
"""
""",
# module required elements are missing
"""
pipeline:
  type: whatever
"""
]:
        try:
             common_parse_config_string(config_string)
        except yaml.scanner.ScannerError:
             continue
        except OrkConfigurationError:
             continue
