import logging
from pathlib import Path

from Pegasus.api import

logging.basicConfig(level=logging.DEBUG)


# PROPERTIES
props = Properties()
props["pegasus.monitored.encoding"] = "json"

props.write()
##### TODO: configure condor.request props

# REPLICA CATALOG (INPUT FILES)


# TRANSFORMATION CATALOG (EXECUTABLES)


# WORKFLOW SPECIFICATION
