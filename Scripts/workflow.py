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
simulationSpec = File("../cav_highwaymerging.pbs")
rc = ReplicaCatalog()\
    .add_replica("local",simulationSpec,Path(".").resolve() \ "cav_highwaymerging.pbs")
    

# TRANSFORMATION CATALOG (EXECUTABLES)


# WORKFLOW SPECIFICATION
