import logging
from pathlib import Path

from Pegasus.api import *

logging.basicConfig(level=logging.DEBUG)


# PROPERTIES
props = Properties()
props["pegasus.monitored.encoding"] = "json"

props.write()
##### TODO: configure condor.request props

# REPLICA CATALOG (INPUT FILES)
simulationSpec = File("./highwaymerge.job")
rc = ReplicaCatalog()\
    .add_replica("local",simulationSpec,Path(".").resolve() / "highwaymerge.job")\


# TRANSFORMATION CATALOG (EXECUTABLES)

## sbatch
sbatch = Transformation(
                "sbatch",
                site="condorpool",
                pfn="/usr/bin/pegasus-keg",
                is_stageable=False,
                arch=Arch.X86_64,
                os_type=OS.LINUX
            )

## qsub
qsub = Transformation(
                "qsub",
                site="condorpool",
                pfn="/usr/bin/pegasus-keg",
                is_stageable=False,
                arch=Arch.X86_64,
                os_type=OS.LINUX
            )

## sh
sh = Transformation(
                "sh",
                site="condorpool",
                pfn="/usr/bin/pegasus-keg",
                is_stageable=False,
                arch=Arch.X86_64,
                os_type=OS.LINUX
            )

## Rscript
Rscript = Transformation(
                "Rscript",
                site="condorpool",
                pfn="/usr/bin/pegasus-keg",
                is_stageable=False,
                arch=Arch.X86_64,
                os_type=OS.LINUX
            )
## Creating Transformation Catalog
tc = TransformationCatalog()\
    .add_transformations(sbatch,qsub,sh,Rscript)
    .write()

# WORKFLOW SPECIFICATION
wf = Workflow("webots.hpc")

simulationOutput = File("output.csv")
job_simulation = Job(sbatch)\
    .add_args("-a", "sbatch", "-T", "3", "-i", simulationSpec, "-o", simulationOutput)\
    .add_inputs(simulationSpec)\
    .add_outputs(simulationOutput)

wf.add_jobs(job_simulation)

try:
    wf.write()
    wf.graph(include_files=True,label="xform-id",output="graph.png")
except PegasusClientError as e:
    print(e)

try:
    wf.plan(submit=True)\
        .wait()
except PegasusClientError as e:
    print(e)

try:
    wf.statistics()
except PegasusClientError as e:
    print(e)
