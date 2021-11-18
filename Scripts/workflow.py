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

## Batch of Simulations [JOB]
simulation = Transformation(
    "simulation",
    site="condorpool",
    pfn="/usr/bin/pegasus-keg",
    is_stageable=False,
    arch=Arch.X86_64,
    os_type=OS.LINUX
)

## Merge Output [JOB]
merge = Transformation(
    "merge",
    site="condorpool",
    pfn="/usr/bin/pegasus-keg",
    is_stageable=False,
    arch=Arch.X86_64,
    os_type=OS.LINUX
)

## ML Preprocessing [JOB]
ml_preprocessing = Transformation(
    "ml_preprocesssing",
    site="condorpool",
    pfn="/usr/bin/pegasus-keg",
    is_stageable=False,
    arch=Arch.X86_64,
    os_type=OS.LINUX
)

## ML Processing [JOB]
ml_processing = Transformation(
    "ml_processing",
    site="condorpool",
    pfn="/usr/bin/pegasus-keg",
    is_stageable=False,
    arch=Arch.X86_64,
    os_type=OS.LINUX
)



# WORKFLOW SPECIFICATION
